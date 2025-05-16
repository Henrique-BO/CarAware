#!/usr/bin/env python3
import rclpy
import zmq
import threading

from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from robot_localization.srv import SetPose
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped

ODOMETRY_TOPIC = '/odometry/filtered'

class EkfToZmq(Node):
    def __init__(self):
        super().__init__('ekf_to_zmq')

        # Declare ROS 2 parameters
        self.declare_parameter('ekf_port', 5001)
        self.declare_parameter('reset_port', 5002)

        # Retrieve parameter values
        self.ekf_port = self.get_parameter('ekf_port').get_parameter_value().integer_value
        self.reset_port = self.get_parameter('reset_port').get_parameter_value().integer_value

        # ROS 2 subscriptions
        self.odo_sub = self.create_subscription(Odometry, ODOMETRY_TOPIC, self.odo_callback, 10)

        # TF2 broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # PUB socket for sending odometry data
        context = zmq.Context()
        self.pub_socket = context.socket(zmq.PUB)
        pub_addr = f"tcp://*:{self.ekf_port}"
        self.pub_socket.bind(pub_addr)
        self.get_logger().info(f"Publishing odometry data on {pub_addr}")

        # REP socket for handling reset requests
        self.rep_socket = context.socket(zmq.REP)
        rep_addr = f"tcp://*:{self.reset_port}"
        self.rep_socket.bind(rep_addr)
        self.get_logger().info(f"Listening for reset requests on {rep_addr}")

        # Start ZMQ reset handler in a separate thread
        self.zmq_thread = threading.Thread(target=self.handle_zmq_requests, daemon=True)
        self.zmq_thread.start()

        # ROS 2 service for resetting EKF
        self.reset_service = self.create_service(
            Trigger,
            '/reset_ekf',
            self.reset_ekf_callback
        )

    def odo_callback(self, msg):
        try:
            # Get the estimated position in the CARLA (map) frame
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            data = {
                'position': [
                    transform.transform.translation.x,
                    transform.transform.translation.y
                ]
            }
            self.pub_socket.send_json(data)
        except Exception as e:
            self.get_logger().error(f"Failed to send odometry data: {e}")

    def handle_zmq_requests(self):
        while rclpy.ok():
            try:
                # Wait for a reset request
                request = self.rep_socket.recv_json()
                if request.get('command') == 'reset':
                    self.get_logger().info('Received EKF reset request through ZMQ')
                    self.reset_ekf()
                    self.rep_socket.send_json({'status': 'success'})
                else:
                    self.rep_socket.send_json({'status': 'unknown_command'})
            except Exception as e:
                self.get_logger().error(f"Failed to handle ZMQ request: {e}")

    def reset_ekf(self):
        try:
            # Get the current groung truth transform
            transform = self.tf_buffer.lookup_transform('map', 'EGO_1/IMU', rclpy.time.Time())
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.pose.position.x = transform.transform.translation.x
            pose.pose.pose.position.y = transform.transform.translation.y
            pose.pose.pose.position.z = transform.transform.translation.z
            pose.pose.pose.orientation = transform.transform.rotation

            # Use default covariance
            pose.pose.covariance = [1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 1e-9, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 1e-9, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 1e-9, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 1e-9, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 1e-9]


            self.get_logger().info(f"Resetting EKF with pose: {pose}")

            # Call the /set_pose service
            client = self.create_client(SetPose, '/set_pose')
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('SetPose service not available')
                return

            request = SetPose.Request()
            request.pose = pose
            future = client.call_async(request)
            future.add_done_callback(self.set_pose_callback_async)
        except Exception as e:
            self.get_logger().error(f"Failed to reset EKF: {e}")

    def set_pose_callback_async(self, future):
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info('EKF reset successfully')
            else:
                self.get_logger().error('Failed to reset EKF')
        except Exception as e:
            self.get_logger().error(f"Failed to reset EKF asynchronously: {e}")

    def reset_ekf_callback(self, request, response):
        self.get_logger().info('Received EKF reset request through ROS 2 service')
        self.reset_ekf()
        response.success = True
        response.message = 'EKF reset successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = EkfToZmq()
    rclpy.spin(node)
    rclpy.shutdown()