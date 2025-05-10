#!/usr/bin/env python3
import socket
import json
import numpy as np
import rclpy
import transforms3d
import time

from rclpy.node import Node
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PointStamped

class ModelNode(Node):
    def __init__(self):
        super().__init__('model_node')

        # Declare ROS 2 parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('ackermann_topic', '/ackermann_drive')
        self.declare_parameter('publish_rate', 50.0)

        # Retrieve parameter values
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        ackermann_topic = self.get_parameter('ackermann_topic').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # ROS 2 subscriptions
        self.subscription_imu = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.subscription_ack = self.create_subscription(AckermannDriveStamped, ackermann_topic, self.ack_callback, 10)

        # ROS 2 publishers
        self.pose_pub_ = self.create_publisher(PoseStamped, 'vehicle_pose', 10)
        self.position_pub_ = self.create_publisher(PointStamped, 'vehicle_position', 10)

        # Data storage
        self.imu_data = None
        self.ack_data = None

        # Socket configuration
        self.server_host = '127.0.0.1'  # Replace with the actual server IP
        self.server_port = 5000         # Port used by serve_model.py
        self.socket = self.connect_to_server()

        self.timer_period = 1.0 / 50  # 50 Hz
        self.timer = self.create_timer(self.timer_period, self.process_data)
        self.get_logger().info('Model node initialized.')

    def connect_to_server(self):
        """
        Establish a connection to the model server.
        """
        attempt = 0
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.server_host, self.server_port))
                self.get_logger().info('Connected to model server.')
                return sock
            except Exception as e:
                self.get_logger().error(f'Failed attempt {attempt} to connect to server: {e}')
                attempt += 1
                time.sleep(1)

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

    def process_data(self):
        """
        Process IMU and odometry data, send it to the model server, and publish the prediction.
        """
        if self.imu_data is None or self.ack_data is None:
            return

        # Prepare input data for the model
        input_data = {
            'state': [
                *self.imu_data['linear_acceleration'],
                *self.imu_data['angular_velocity'],
                self.imu_data['yaw'],
                self.ack_data['speed'],
                self.ack_data['steering_angle']
            ]
        }

        try:
            # Send data to the model server
            self.socket.send(json.dumps(input_data).encode('utf-8'))

            # Receive the prediction from the server
            response = self.socket.recv(1024).decode('utf-8')
            prediction = json.loads(response)

            # Publish the prediction
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x, pose_msg.pose.position.y = prediction['action']
            pose_msg.pose.position.z = 0.0
            self.pose_pub_.publish(pose_msg)

            position_msg = PointStamped()
            position_msg.header = pose_msg.header
            position_msg.point = pose_msg.pose.position
            self.position_pub_.publish(position_msg)
        except Exception as e:
            self.get_logger().error(f'Error during communication with server: {e}')

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