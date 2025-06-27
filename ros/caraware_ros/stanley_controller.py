import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import Buffer, TransformListener, LookupException
import math

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        self.declare_parameter('k', 1.0)  # gain parameter
        self.declare_parameter('velocity', 2.0)  # constant speed
        self.declare_parameter('frame_id', 'model_frame')  # vehicle frame ID

        self.k = self.get_parameter('k').value
        self.velocity = self.get_parameter('velocity').value
        self.vehicle_frame_id = self.get_parameter('frame_id').value

        self.path = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, '/path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.feedback_pub = self.create_publisher(Int32, '/closest_index_feedback', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

    def orientation_to_yaw(self, orientation):
        return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                          1.0 - 2.0 * (orientation.y**2 + orientation.z**2))

    def path_callback(self, msg):
        self.path = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
                self.orientation_to_yaw(pose.pose.orientation)
            ) for pose in msg.poses]

        # self.get_logger().info(f"Received path with {len(self.path)} points")
        # self.get_logger().info(f"{self.path}")

    def control_loop(self):
        if not self.path:
            self.cmd_pub.publish(AckermannDriveStamped())  # Publish empty command if no path
            return

        # Get current vehicle position and orientation
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', self.vehicle_frame_id, rclpy.time.Time())
            px = transform.transform.translation.x
            py = transform.transform.translation.y
            yaw = self.orientation_to_yaw(transform.transform.rotation)
        except LookupException:
            self.get_logger().warn('Transform not available')
            return

        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        x_closest, y_closest, yaw_closest = self.path[0]
        for i, (x_path, y_path, yaw_path) in enumerate(self.path):
            dist = math.hypot(px - x_path, py - y_path)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                x_closest, y_closest, yaw_closest = x_path, y_path, yaw_path

        idx = min(closest_idx + 5, len(self.path) - 1)
        x_closest, y_closest, yaw_closest = self.path[idx]

        # Compute heading error
        heading_error = self.angle_difference(yaw_closest, yaw)

        # Compute cross track error
        dx = x_closest - px
        dy = y_closest - py
        perp_error = dx * math.sin(yaw_closest) - dy * math.cos(yaw_closest)

        # Stanley control
        steer = heading_error #+ math.atan2(self.k * perp_error, self.velocity)
        # self.get_logger().info(
        #     f"Heading error: {math.degrees(heading_error):.2f}°, "
        #     f"Perpendicular error: {perp_error:.2f} m, "
        #     f"CTE angle: {math.degrees(math.atan2(self.k * perp_error, self.velocity)):.2f}°, "
        #     f"Steering angle: {math.degrees(steer):.2f}°, "
        # )

        # Publish Ackermann command
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.speed = self.velocity
        ackermann_msg.drive.steering_angle = steer
        self.cmd_pub.publish(ackermann_msg)

        # Publish feedback to planner
        self.feedback_pub.publish(Int32(data=closest_idx))


    def angle_difference(self, angle1, angle2):
        diff = (angle1 - angle2 + math.pi) % (2 * math.pi) - math.pi
        return diff


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
