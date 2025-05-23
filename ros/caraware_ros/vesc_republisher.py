import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped

VESC_TOPIC_IN = "/vesc/sensors/core"
SERVO_TOPIC_IN = "/vesc/sensors/servo_position_command"

TWIST_TOPIC_OUT = "/twist"
ACKERMANN_TOPIC_OUT = "/ackermann_drive"

class VescRepublisher(Node):
    def __init__(self):
        super().__init__('vesc_republisher')

        # Declare parameters
        self.declare_parameter('speed_to_erpm_gain', 4000.0)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter('steering_angle_to_servo_gain', 1.0)
        self.declare_parameter('steering_angle_to_servo_offset', 0.0)
        self.declare_parameter('wheelbase', 0.2)
        self.declare_parameter('frame_id', 'base_link')

        # Get parameters
        self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').get_parameter_value().double_value
        self.speed_to_erpm_offset = self.get_parameter('speed_to_erpm_offset').get_parameter_value().double_value
        self.steering_angle_to_servo_gain = self.get_parameter('steering_angle_to_servo_gain').get_parameter_value().double_value
        self.steering_angle_to_servo_offset = self.get_parameter('steering_angle_to_servo_offset').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Subscribers
        self.vesc_sub = self.create_subscription(VescStateStamped, VESC_TOPIC_IN, self.vesc_callback, 10)
        self.servo_sub = self.create_subscription(Float64, SERVO_TOPIC_IN, self.servo_cmd_callback, 10)

        # Publishers
        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, TWIST_TOPIC_OUT, 10)
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, ACKERMANN_TOPIC_OUT, 10)

        # Initialize variables
        self.vesc_speed = 0.0
        self.servo_position = 0.0

        self.get_logger().info("VescRepublisher has been started.")

    def vesc_callback(self, msg: VescStateStamped):
        self.vesc_speed = msg.state.speed
        self.publish()

    def servo_cmd_callback(self, msg: Float64):
        self.servo_position = msg.data
        self.publish()

    def publish(self):
        speed = (self.vesc_speed - self.speed_to_erpm_offset) / self.speed_to_erpm_gain
        steering_angle = (self.servo_position - self.steering_angle_to_servo_offset) / self.steering_angle_to_servo_gain
        stamp = self.get_clock().now().to_msg()

        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = stamp
        ackermann_msg.header.frame_id = self.frame_id
        ackermann_msg.drive.speed = speed
        ackermann_msg.drive.steering_angle = steering_angle
        self.ackermann_pub.publish(ackermann_msg)

        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = stamp
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.twist.linear.x = speed
        twist_msg.twist.twist.angular.z = speed / self.wheelbase * np.tan(steering_angle)
        twist_msg.twist.covariance = \
            [0.1**2,0.0, 0.0, 0.0, 0.0, 0.0,
             0.0,   0.0, 0.0, 0.0, 0.0, 0.0,
             0.0,   0.0, 0.0, 0.0, 0.0, 0.0,
             0.0,   0.0, 0.0, 0.0, 0.0, 0.0,
             0.0,   0.0, 0.0, 0.0, 0.0, 0.0,
             0.0,   0.0, 0.0, 0.0, 0.0, 0.05**2]
        self.twist_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VescRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()