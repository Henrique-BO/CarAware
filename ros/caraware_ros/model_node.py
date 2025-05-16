#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import time
import rclpy
import transforms3d
import zmq
import threading

from rclpy.node import Node
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformListener, Buffer

IMU_TOPIC = '/imu/data'
ACKERMANN_TOPIC = '/carla/EGO_1/Speed_SAS'
ODOMETRY_TOPIC = '/odometry/filtered'

OUTPUT_TOPIC = '/model/prediction'

class ModelNode(Node):
    def __init__(self):
        super().__init__('model_node')

        # Declare ROS 2 parameters
        self.declare_parameter('model_port', 5000)
        self.declare_parameter('frame_id', 'model_frame')
        self.declare_parameter('publish_rate', 50.0)

        # Retrieve parameter values
        self.model_port = self.get_parameter('model_port').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # ROS 2 subscriptions
        self.imu_sub = self.create_subscription(Imu, IMU_TOPIC, self.imu_callback, 10)
        self.ack_sub = self.create_subscription(AckermannDriveStamped, ACKERMANN_TOPIC, self.ack_callback, 10)
        self.odo_sub = self.create_subscription(Odometry, ODOMETRY_TOPIC, self.odo_callback, 10)

        # ROS 2 publishers
        self.position_pub_ = self.create_publisher(PointStamped, OUTPUT_TOPIC, 10)

        # TF2 broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Data storage
        self.imu_data = None
        self.ack_data = None
        self.ekf_position = None
        self.base_link_orientation = None

        # REQ socket for receiving model predictions
        context = zmq.Context()
        self.pred_socket = context.socket(zmq.REQ)
        addr = f"tcp://localhost:{self.model_port}"
        self.pred_socket.connect(addr)
        self.timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.timer_period, self.process_data)
        self.get_logger().info(f'Model predictions requested on {addr}')

        # Initialize real-time plotting
        self.errors = []
        self.timestamps = []
        self.plot_thread = threading.Thread(target=self.plot_error, daemon=True)
        self.plot_thread.start()

        self.get_logger().info('Model node initialized.')

    def imu_callback(self, msg):
        """
        Callback for IMU data.
        """
        roll, pitch, yaw = transforms3d.euler.quat2euler([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        yaw = np.rad2deg(yaw)
        self.imu_data = {
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'yaw': yaw,
        }

    def ack_callback(self, msg):
        """
        Callback for ackermann data.
        """
        self.ack_data = {
            'speed': msg.drive.speed,
            'steering_angle': msg.drive.steering_angle
        }

    def odo_callback(self, msg):
        """
        Callback for filtered state estimate.
        """
        try:
            # Get the estimated position in the CARLA (map) frame
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.ekf_position = [
                transform.transform.translation.x,
                transform.transform.translation.y
            ]
            self.base_link_orientation = transform.transform.rotation
        except Exception as e:
            self.get_logger().warn(f"Could not get map->base_link transform: {e}")

    def process_data(self):
        """
        Process IMU and odometry data, send it to the model server, and publish the prediction.
        """
        if self.imu_data is None or self.ack_data is None or \
            self.ekf_position is None or self.base_link_orientation is None:
            return

        # Prepare input data for the model
        input_data = {
            'observations': [
                *self.ekf_position,
                *self.imu_data['linear_acceleration'],
                *self.imu_data['angular_velocity'],
                self.imu_data['yaw'],
                self.ack_data['speed'],
                self.ack_data['steering_angle']
            ]
        }

        # Send data to the model server
        self.pred_socket.send_json(input_data)

        # # Wait for the prediction
        output_data = self.pred_socket.recv_json()

        # Publish the prediction
        position_msg = PointStamped()
        position_msg.header.stamp = self.get_clock().now().to_msg()
        position_msg.header.frame_id = 'map'
        position_msg.point.x, position_msg.point.y = output_data['prediction']
        self.position_pub_.publish(position_msg)

        # Publish as transform
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = 'map'
        transform_msg.child_frame_id = self.frame_id
        transform_msg.transform.translation.x = position_msg.point.x
        transform_msg.transform.translation.y = position_msg.point.y
        transform_msg.transform.translation.z = 0.0
        transform_msg.transform.rotation = self.base_link_orientation
        self.tf_broadcaster.sendTransform(transform_msg)

        # Plot the prediction error in real time
        try:
            transform = self.tf_buffer.lookup_transform('map', 'EGO_1/IMU', rclpy.time.Time())
            gt = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y
            ])
            error = np.linalg.norm(np.array([position_msg.point.x, position_msg.point.y]) - gt)

            self.errors.append(error)
            self.timestamps.append(self.get_clock().now().nanoseconds * 1e-9)  # Convert to seconds

            if len(self.errors) > 1000:
                self.errors.pop(0)
                self.timestamps.pop(0)
        except Exception as e:
            self.get_logger().warn(f"Could not get map->base_link transform: {e}")

    def plot_error(self):
        """
        Real-time plot of prediction error.
        """
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_title("Prediction Error Over Time")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (m)")
        ax.grid()
        line, = ax.plot([], [], 'r-')

        while rclpy.ok():
            if self.timestamps and self.errors:
                line.set_xdata(self.timestamps)
                line.set_ydata(self.errors)
                ax.relim()
                ax.autoscale_view()
                plt.draw()
                plt.pause(0.1)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    model_node = ModelNode()
    try:
        rclpy.spin(model_node)
    except KeyboardInterrupt:
        model_node.get_logger().info('Shutting down node...')
    finally:
        model_node.socket.close()
        model_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()