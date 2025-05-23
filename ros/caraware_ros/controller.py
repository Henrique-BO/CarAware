import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener

class MotorController(Node):
    def __init__(self):
        super().__init__('controller')

        # Declare parameters
        self.declare_parameter('frame_id', 'model_frame')
        self.declare_parameter('angular_proportional', 0.4)
        self.declare_parameter('linear_proportional', 1.0)
        self.declare_parameter('max_lin_vel', 0.2) # m/s
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('speed_to_erpm_gain', 1.0)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter('steering_angle_to_servo_gain', 1.0)
        self.declare_parameter('steering_angle_to_servo_offset', 0.0)
        self.declare_parameter('wheelbase', 0.2)

        # Get parameters
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.angular_proportional = self.get_parameter('angular_proportional').get_parameter_value().double_value
        self.linear_proportional = self.get_parameter('linear_proportional').get_parameter_value().double_value
        self.max_lin_vel = self.get_parameter('max_lin_vel').get_parameter_value().double_value
        self.invert_angular = self.get_parameter('invert_angular').get_parameter_value().bool_value
        self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').get_parameter_value().double_value
        self.speed_to_erpm_offset = self.get_parameter('speed_to_erpm_offset').get_parameter_value().double_value
        self.steering_angle_to_servo_gain = self.get_parameter('steering_angle_to_servo_gain').get_parameter_value().double_value
        self.steering_angle_to_servo_offset = self.get_parameter('steering_angle_to_servo_offset').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 1)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Publishers
        self.lin_pub = self.create_publisher(
            Float64, '/vesc/commands/motor/duty_cycle', 3)
        self.ang_pub = self.create_publisher(
            Float64, '/vesc/commands/servo/position', 3)

        self.goal = PoseStamped()
        self.odom_msg = Odometry()

        self.timer = self.create_timer(0.5, self.spin)
        self.get_logger().info('Ready to follow goal')

    def goal_callback(self, goal_msg: PoseStamped):
        try:
            goal_msg.header.stamp = rclpy.time.Time().to_msg()
            self.goal = self.buffer.transform(goal_msg, self.frame_id)
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")

    def spin(self):
        delta_x = self.goal.pose.position.x
        delta_y = self.goal.pose.position.y

        distance = math.sqrt(delta_x**2 + delta_y**2)
        theta = math.atan2(delta_y, delta_x)

        linear_action = min(abs(self.max_lin_vel), abs(self.linear_proportional * distance))
        if delta_x < 0:
            linear_action *= -1
            theta = math.atan2(delta_y, -delta_x)

        angular_action = self.angular_proportional * theta
        angular_action = angular_action / 2 + 0.5

        if self.invert_angular:
            angular_action *= -1

        lin_msg = Float64()
        ang_msg = Float64()
        lin_msg.data = linear_action * self.speed_to_erpm_gain + self.speed_to_erpm_offset
        ang_msg.data = angular_action * self.steering_angle_to_servo_gain + self.steering_angle_to_servo_offset

        self.lin_pub.publish(lin_msg)
        self.ang_pub.publish(ang_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
