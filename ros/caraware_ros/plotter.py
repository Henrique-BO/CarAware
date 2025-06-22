import rclpy
import threading
import csv
import matplotlib.pyplot as plt
import numpy as np

from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class Plotter(Node):
    def __init__(self):
        super().__init__('plotter')
        self.declare_parameter('frames', ['base_link'])
        self.declare_parameter('world_frame', 'map')
        self.frames = self.get_parameter('frames').get_parameter_value().string_array_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        if not self.frames:
            self.get_logger().info('No frames specified. Plotter will not plot anything.')
            self.frames = []
        else:
            self.get_logger().info(f'Plotting frames: {self.frames} relative to {self.world_frame}')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.times = []
        self.trajectories = {frame: [] for frame in self.frames}
        self.plot_thread = threading.Thread(target=self.plot_trajectories, daemon=True)
        self.plot_thread.start()
        self.timer = self.create_timer(0.05, self.update_trajectories)

    def update_trajectories(self):
        if not self.frames:
            return

        t = self.get_clock().now()
        now = t.seconds_nanoseconds()[0] + t.seconds_nanoseconds()[1] * 1e-9
        self.times.append(now)
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
    def save_trajectories(self, filename='trajectories.csv'):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            columns = ['time']
            for frame in self.trajectories.keys():
                columns.append(f"{frame}_x")
                columns.append(f"{frame}_y")
            writer.writerow(columns)
            # Iterate over each timestamp and the corresponding trajectory points for all frames
            for t, traj_points in zip(self.times, zip(*self.trajectories.values())):
                row = [t]
                # traj_points is a tuple with one (x, y) tuple per frame at this timestamp
                for coords in traj_points:
                    x, y = coords
                    row.extend([x, y])
                writer.writerow(row)

def main(args=None):
    rclpy.init(args=args)
    node = Plotter()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.save_trajectories()
        pass
    node.destroy_node()
    rclpy.shutdown()