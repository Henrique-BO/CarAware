import rclpy
import numpy as np
import transforms3d

from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped

from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger

IMU_TOPIC_INF = "/imu/data_in"
IMU_TOPIC_OUT = "/imu/data_out"

ACKERMANN_TOPIC_IN = "/speed_sas/ackermann_in"
TWIST_TOPIC_OUT = "/speed_sas/twist_out"

class CarlaRepublisher(Node):
    def __init__(self):
        super().__init__('carla_republisher')

        # Declare and get parameters
        self.declare_parameter('imu_frame_id', 'imu_frame')
        self.declare_parameter('twist_frame_id', 'twist_frame')
        self.declare_parameter('wheelbase', 2.0)

        self.imu_frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value
        self.twist_frame_id = self.get_parameter('twist_frame_id').get_parameter_value().string_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            IMU_TOPIC_INF,
            self.imu_callback,
            10
        )
        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            ACKERMANN_TOPIC_IN,
            self.ackermann_callback,
            10
        )

        # Publishers
        self.imu_publisher = self.create_publisher(Imu, IMU_TOPIC_OUT, 10)
        self.twist_publisher = self.create_publisher(TwistWithCovarianceStamped, TWIST_TOPIC_OUT, 10)

        # TF2 broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)

        # Services
        self.service = self.create_service(Trigger, 'calculate_map_to_odom', self.map_to_odom_callback)
        self.get_logger().info('Service `calculate_map_to_odom` ready')

        self.get_logger().info('CarlaRepublisher node initialized')


    def imu_callback(self, msg):
        msg.header.frame_id = self.imu_frame_id
        self.imu_publisher.publish(msg)

    def ackermann_callback(self, msg):
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header = msg.header
        twist_msg.header.frame_id = self.twist_frame_id
        twist_msg.twist.twist.linear.x = msg.drive.speed
        twist_msg.twist.twist.angular.z = \
            msg.drive.speed / self.wheelbase * np.tan(msg.drive.steering_angle)
        twist_msg.twist.covariance = \
                [0.1**2,0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0,   0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0,   0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0,   0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0,   0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0,   0.0, 0.0, 0.0, 0.0, 0.05**2]
        self.twist_publisher.publish(twist_msg)

    def map_to_odom_callback(self, request, response):
        try:
            # Lookup transforms
            t_map_to_imu = self.tf_buffer.lookup_transform('map', 'EGO_1/IMU', rclpy.time.Time())
            t_odom_to_base = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())

            tf_map_to_imu = self.transform_to_matrix(t_map_to_imu)
            tf_odom_to_base = self.transform_to_matrix(t_odom_to_base)

            tf_base_to_odom = np.linalg.inv(tf_odom_to_base)

            # map -> odom = map -> imu * base_link -> odom
            tf_map_to_odom = np.dot(tf_map_to_imu, tf_base_to_odom)

            # Extract components
            translation = tf_map_to_odom[:3, 3]
            rotation_matrix = tf_map_to_odom[:3, :3]
            quaternion = transforms3d.quaternions.mat2quat(rotation_matrix)  # [w, x, y, z]

            # Broadcast static transform
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = float(translation[0])
            t.transform.translation.y = float(translation[1])
            t.transform.translation.z = float(translation[2])
            t.transform.rotation.x = float(quaternion[1])
            t.transform.rotation.y = float(quaternion[2])
            t.transform.rotation.z = float(quaternion[3])
            t.transform.rotation.w = float(quaternion[0])

            self.broadcaster.sendTransform(t)
            self.get_logger().info('Published map->odom static transform')

            response.success = True
            response.message = 'map->odom transform calculated and published'
            return response

        except Exception as e:
            self.get_logger().error(f'Failed to compute map->odom transform: {e}')
            response.success = False
            response.message = str(e)
            return response

    def transform_to_matrix(self, transform: TransformStamped):
        trans = transform.transform.translation
        rot = transform.transform.rotation

        translation = np.array([trans.x, trans.y, trans.z])
        quaternion = np.array([rot.w, rot.x, rot.y, rot.z])  # w first for transforms3d

        rotation_matrix = transforms3d.quaternions.quat2mat(quaternion)
        tf_matrix = np.eye(4)
        tf_matrix[:3, :3] = rotation_matrix
        tf_matrix[:3, 3] = translation

        return tf_matrix

def main(args=None):
    rclpy.init(args=args)
    node = CarlaRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()