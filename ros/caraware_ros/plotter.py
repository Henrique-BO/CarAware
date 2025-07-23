import rclpy
import threading
import csv
import matplotlib.pyplot as plt
import numpy as np
import zmq

from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Trigger

class Plotter(Node):
    def __init__(self):
        super().__init__('plotter')
        self.declare_parameter('frames', ['base_link', 'model_frame'])
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('obs_port', 5000)

        self.frames = self.get_parameter('frames').get_parameter_value().string_array_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.obs_port = self.get_parameter('obs_port').get_parameter_value().integer_value

        if not self.frames:
            self.get_logger().info('No frames specified. Plotter will not plot anything.')
            self.frames = []
        else:
            self.get_logger().info(f'Plotting frames: {self.frames} relative to {self.world_frame}')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.times = []
        self.trajectories = {frame: [] for frame in self.frames}
        self.observations = []
        # self.plot_thread = threading.Thread(target=self.plot_trajectories, daemon=True)
        # self.plot_thread.start()
        self.timer = self.create_timer(0.05, self.update_trajectories)
        self.save_service = self.create_service(
            Trigger,
            'save_trajectories',
            self.save_trajectories_callback
        )

        # SUB socket for receiving observations
        context = zmq.Context()
        self.sub_socket = context.socket(zmq.SUB)
        obs_addr = f"tcp://localhost:{self.obs_port}"
        self.sub_socket.connect(obs_addr)
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, '')  # Subscribe to all topics
        print(f"Listening for observations on {obs_addr}")

        self.observation = [np.nan] * 11
        self.obs_thread = threading.Thread(target=self.observation_listener, daemon=True)
        self.obs_thread.start()

    def observation_listener(self):
        try:
            while True:
                # Wait for observations from the client
                self.observation = self.sub_socket.recv_json()['observations']
        except KeyboardInterrupt:
            print("Shutting down observation listener...")
        finally:
            self.sub_socket.close()

    def update_trajectories(self):
        if not self.frames:
            return

        t = rclpy.time.Time()
        now = self.get_clock().now()
        now = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
        self.times.append(now)
        self.observations.append(self.observation)
        for frame in self.frames:
            try:
                trans = self.tf_buffer.lookup_transform(self.world_frame, frame, t)
                x = trans.transform.translation.x
                y = trans.transform.translation.y
            except Exception as e:
                x = np.nan
                y = np.nan
            self.trajectories[frame].append((x, y))

    def plot_trajectories(self):
        if not self.frames:
            return
        plt.ion()
        fig, ax = plt.subplots()
        colors = plt.cm.get_cmap('tab10', len(self.frames))
        while rclpy.ok():
            ax.clear()
            for idx, frame in enumerate(self.frames):
                traj = np.array(self.trajectories[frame])
                if len(traj) > 0:
                    # traj[:, 1] is x, traj[:, 2] is y
                    ax.plot(traj[:, 1], traj[:, 2], label=frame, color=colors(idx))
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_title('2D Trajectories')
            ax.legend()
            ax.grid(True)
            plt.pause(0.1)
        plt.ioff()
        plt.show()

    # Save the trajectories to a csv file
    def save_trajectories(self, filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            columns = ['time']
            for frame in self.trajectories.keys():
                columns.append(f"{frame}_x")
                columns.append(f"{frame}_y")
            columns.extend([
                "EKF_x",
                "EKF_y",
                "acc_x",
                "acc_y",
                "acc_z",
                "gyro_x",
                "gyro_y",
                "gyro_z",
                "yaw",
                "speed",
                "steering_angle",
            ])
            writer.writerow(columns)

            # Iterate over each timestamp and the corresponding trajectory points for all frames
            for t, traj_points, observation in zip(self.times, zip(*self.trajectories.values()), self.observations):
                row = [t]
                # traj_points is a tuple with one (x, y) tuple per frame at this timestamp
                for coords in traj_points:
                    x, y = coords
                    row.extend([x, y])
                row.extend(observation)
                writer.writerow(row)
        self.get_logger().info(f"Trajectories saved to {filename}")

    def save_trajectories_callback(self, request, response):
        filename = f"/root/carla_ws/src/CarAware/notebooks/data/trajectories.csv"
        self.save_trajectories(filename)
        response.success = True
        response.message = f"Trajectories saved to {filename}"

        # Reset trajectories and times after saving
        self.times = []
        self.trajectories = {frame: [] for frame in self.frames}
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Plotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()