import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_geometry_msgs import do_transform_pose

class GoalController(Node):
    def __init__(self):
        super().__init__('goal_controller')

        # Declare parameters
        self.declare_parameter('frame_id', 'model_frame')
        self.declare_parameter('angular_proportional', 0.4)
        self.declare_parameter('linear_proportional', 1.0)
        self.declare_parameter('max_lin_vel', 0.2) # m/s
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('threshold', 0.1)

        # Get parameters
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.angular_proportional = self.get_parameter('angular_proportional').get_parameter_value().double_value
        self.linear_proportional = self.get_parameter('linear_proportional').get_parameter_value().double_value
        self.max_lin_vel = self.get_parameter('max_lin_vel').get_parameter_value().double_value
        self.invert_angular = self.get_parameter('invert_angular').get_parameter_value().bool_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 1)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Publishers
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped, '/ackermann_cmd', 3)

        self.goal = PoseStamped()
        self.odom_msg = Odometry()
        self.reached = False

        self.timer = self.create_timer(0.5, self.spin)
        self.get_logger().info('Ready to follow goal')


    def goal_callback(self, goal_msg: PoseStamped):
        self.goal = goal_msg
        self.goal.header.stamp = self.get_clock().now().to_msg()
        self.reached = False

    def spin(self):
        try:
            transform = self.buffer.lookup_transform(
                self.frame_id,
                self.goal.header.frame_id,
                rclpy.time.Time())
            delta = do_transform_pose(self.goal.pose, transform)
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")
            self.ackermann_pub.publish(AckermannDriveStamped())
            return

        delta_x = delta.position.x
        delta_y = delta.position.y
        self.get_logger().info(f"Goal position: {delta_x}, {delta_y}")

        distance = math.sqrt(delta_x**2 + delta_y**2)
        theta = math.atan2(delta_y, delta_x)

        self.get_logger().info(f"Distance: {distance}, Angle: {theta}")

        if distance < self.threshold:
            if not self.reached:
                self.get_logger().info("Goal reached")
                self.reached = True
            self.ackermann_pub.publish(AckermannDriveStamped())
            return


        linear_action = min(abs(self.max_lin_vel), abs(self.linear_proportional * distance))
        if delta_x < 0:
            linear_action *= -1
            theta = math.atan2(delta_y, -delta_x)

        angular_action = self.angular_proportional * theta

        if self.invert_angular:
            angular_action *= -1

        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.speed = linear_action
        ackermann_msg.drive.steering_angle = angular_action

        self.ackermann_pub.publish(ackermann_msg)

        self.get_logger().info(f"Publishing: speed={linear_action}, steering_angle={angular_action}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
