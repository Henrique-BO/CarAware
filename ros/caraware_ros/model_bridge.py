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
from robot_localization.srv import SetPose
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped

IMU_TOPIC = '/imu/data'
ACKERMANN_TOPIC = '/ackermann_drive'
ODOMETRY_TOPIC = '/odometry/filtered'

OUTPUT_TOPIC = '/model/prediction'

class ModelBridge(Node):
    def __init__(self):
        super().__init__('model_bridge')

        # Declare ROS 2 parameters
        self.declare_parameter('obs_port', 5000)
        self.declare_parameter('pred_port', 5001)
        self.declare_parameter('reset_port', 5002)
        self.declare_parameter('frame_id', 'model_frame')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('plot_error', True)

        # Retrieve parameter values
        self.obs_port = self.get_parameter('obs_port').get_parameter_value().integer_value
        self.pred_port = self.get_parameter('pred_port').get_parameter_value().integer_value
        self.reset_port = self.get_parameter('reset_port').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.plot_error = self.get_parameter('plot_error').get_parameter_value().bool_value

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

        # ZMQ PUB socket for streaming observations
        context = zmq.Context()
        self.obs_socket = context.socket(zmq.PUB)
        self.obs_addr = f"tcp://*:{self.obs_port}"
        self.obs_socket.bind(self.obs_addr)
        # self.obs_timer = self.create_timer(1.0 / self.publish_rate, self.send_observations)
        self.period = 1.0 / self.publish_rate
        self.obs_timer = self.create_timer(self.period, self.send_observations)
        self.get_logger().info(f'Streaming observations on {self.obs_addr} at {self.publish_rate} Hz')

        # ZMQ SUB socket for receiving predictions
        self.pred_socket = context.socket(zmq.SUB)
        self.pred_addr = f"tcp://localhost:{self.pred_port}"
        self.pred_socket.connect(self.pred_addr)
        self.pred_socket.setsockopt_string(zmq.SUBSCRIBE, '')  # Subscribe to all topics
        self.pred_thread = threading.Thread(target=self.receive_predictions, daemon=True)
        self.pred_thread.start()
        self.get_logger().info(f'Receiving predictions on {self.pred_addr}')

        # ZMQ REP socket for handling reset requests
        context = zmq.Context()
        self.reset_socket = context.socket(zmq.REP)
        self.reset_addr = f"tcp://*:{self.reset_port}"
        self.reset_socket.bind(self.reset_addr)
        self.reset_thread = threading.Thread(target=self.handle_reset_requests, daemon=True)
        self.reset_thread.start()
        self.get_logger().info(f"Listening for reset requests on {self.reset_addr}")

        # ROS 2 service for resetting EKF
        self.reset_service = self.create_service(
            Trigger,
            '/reset_ekf',
            self.reset_ekf_callback
        )

        # Initialize real-time plotting
        self.errors = []
        self.kf_errors = []
        self.timestamps = []
        if self.plot_error:
            self.plot_thread = threading.Thread(target=self.plot, daemon=True)
            self.plot_thread.start()

        self.get_logger().info('Model node initialized.')

    def imu_callback(self, msg):
        """
        Callback for IMU data.
        """
        roll, pitch, yaw = transforms3d.euler.quat2euler([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        yaw = np.rad2deg(yaw)
        assert -180 <= yaw <= 180, "Yaw angle out of bounds"
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
            # EKF position estimate in the CARLA (map) frame
            self.ekf_position = [
                transform.transform.translation.x,
                -transform.transform.translation.y
            ]
            # self.get_logger().info(f"EKF position: {self.ekf_position}")
            self.base_link_orientation = transform.transform.rotation
        except Exception as e:
            self.get_logger().warn(f"Could not get map->base_link transform: {e}")

    def send_observations(self):
        """
        Process IMU and odometry data and stream observations.
        """
        if self.imu_data is None or self.ack_data is None or \
            self.ekf_position is None or self.base_link_orientation is None:
            # self.get_logger().warn("Waiting for data...")
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

        # Stream observations via ZMQ PUB socket
        self.obs_socket.send_json(input_data)

    def receive_predictions(self):
        """
        Continuously receive predictions from the ZMQ SUB socket and publish them.
        """
        while rclpy.ok():
            try:
                # Wait for the prediction
                output_data = self.pred_socket.recv_json()

                # Publish the prediction
                stamp = self.get_clock().now().to_msg()
                position_msg = PointStamped()
                position_msg.header.stamp = stamp
                position_msg.header.frame_id = 'map'
                position_msg.point.x = output_data['prediction'][0]
                position_msg.point.y = -output_data['prediction'][1]
                self.position_pub_.publish(position_msg)

                # Publish as transform
                transform_msg = TransformStamped()
                transform_msg.header.stamp = stamp
                transform_msg.header.frame_id = 'map'
                transform_msg.child_frame_id = self.frame_id
                transform_msg.transform.translation.x = position_msg.point.x
                transform_msg.transform.translation.y = position_msg.point.y
                transform_msg.transform.translation.z = 0.0
                transform_msg.transform.rotation = self.base_link_orientation
                self.tf_broadcaster.sendTransform(transform_msg)

                # Plot the prediction error in real time
                try:
                    gt_transform = self.tf_buffer.lookup_transform('map', 'EGO_1/IMU', rclpy.time.Time())
                    gt = np.array([
                        gt_transform.transform.translation.x,
                        gt_transform.transform.translation.y
                    ])

                    error = np.linalg.norm(np.array([position_msg.point.x, position_msg.point.y]) - gt)

                    kf_transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    kf = np.array([
                        kf_transform.transform.translation.x,
                        kf_transform.transform.translation.y
                    ])
                    kf_error = np.linalg.norm(kf - gt)

                    self.errors.append(error)
                    self.kf_errors.append(kf_error)
                    self.timestamps.append(self.get_clock().now().nanoseconds * 1e-9)  # Convert to seconds

                    if len(self.errors) > 1000:
                        self.errors.pop(0)
                        self.kf_errors.pop(0)
                        self.timestamps.pop(0)
                except Exception as e:
                    self.get_logger().warn(f"Could not get map->base_link transform: {e}")
            except Exception as e:
                self.get_logger().error(f"Error during ZMQ communication: {e}")

    def plot(self):
        """
        Real-time plot of prediction error.
        """
        fig, ax = plt.subplots()
        ax.set_title("Prediction Error Over Time")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (m)")
        ax.grid()
        line, = ax.plot([], [], 'r-', label='Model')
        line_kf, = ax.plot([], [], 'b-', label='EKF')
        ax.legend()

        while rclpy.ok():
            if self.timestamps and self.errors:
                line.set_xdata(self.timestamps)
                line.set_ydata(self.errors)

                line_kf.set_xdata(self.timestamps)
                line_kf.set_ydata(self.kf_errors)
                
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw_idle()
            plt.pause(0.1)
            time.sleep(0.1)

    def reset_ekf(self):
        """
        Reset the EKF using the current ground truth transform.
        """
        try:
            # Call the /calculate_map_to_odom service
            client = self.create_client(Trigger, '/calculate_map_to_odom')
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('CalculateMapToOdom service not available')
                return

            self.get_logger().info("Calling /calculate_map_to_odom service to reset EKF")
            request = Trigger.Request()
            future = client.call_async(request)
            future.add_done_callback(self.set_pose_callback_async)
        except Exception as e:
            self.get_logger().error(f"Failed to reset EKF: {e}")

    def set_pose_callback_async(self, future):
        """
        Callback for handling the result of the asynchronous EKF reset request.
        """
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info('EKF reset successfully')
            else:
                self.get_logger().error('Failed to reset EKF')
        except Exception as e:
            self.get_logger().error(f"Failed to reset EKF asynchronously: {e}")

    def reset_ekf_callback(self, request, response):
        """
        Handle EKF reset requests through the ROS 2 service.
        """
        self.get_logger().info('Received EKF reset request through ROS 2 service')
        self.reset_ekf()
        response.success = True
        response.message = 'EKF reset successfully'
        return response

    def handle_reset_requests(self):
        """
        Continuously handle reset requests from the ZMQ REP socket.
        """
        while rclpy.ok():
            try:
                # Wait for a reset request
                request = self.reset_socket.recv_json()
                if request.get('command') == 'reset':
                    self.get_logger().info('Received EKF reset request through ZMQ')
                    self.reset_ekf()
                    self.reset_socket.send_json({'status': 'success'})
                else:
                    self.reset_socket.send_json({'status': 'unknown_command'})
            except Exception as e:
                self.get_logger().error(f"Failed to handle ZMQ reset request: {e}")

def main(args=None):
    rclpy.init(args=args)
    model_bridge = ModelBridge()
    try:
        rclpy.spin(model_bridge)
    except KeyboardInterrupt:
        model_bridge.get_logger().info('Shutting down node...')
    finally:
        model_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
