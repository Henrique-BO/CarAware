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

        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.feedback_pub = self.create_publisher(Int32, '/closest_index_feedback', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

    def path_callback(self, msg):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def control_loop(self):
        if not self.path:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', self.vehicle_frame_id, rclpy.time.Time())
            px = transform.transform.translation.x
            py = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
        except LookupException:
            self.get_logger().warn('Transform not available')
            return

        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0

        for i, (x, y) in enumerate(self.path):
            dist = math.hypot(px - x, py - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Publish feedback to planner
        self.feedback_pub.publish(Int32(data=closest_idx))

        # Compute heading error
        if closest_idx < len(self.path) - 1:
            x1, y1 = self.path[closest_idx]
            x2, y2 = self.path[closest_idx + 1]
        else:
            x1, y1 = self.path[closest_idx - 1]
            x2, y2 = self.path[closest_idx]

        path_yaw = math.atan2(y2 - y1, x2 - x1)
        heading_error = self.normalize_angle(path_yaw - yaw)

        # Compute cross track error
        dx = x1 - px
        dy = y1 - py
        perp_error = dx * math.sin(path_yaw) - dy * math.cos(path_yaw)

        # Stanley control
        steer = heading_error + math.atan2(self.k * perp_error, self.velocity)

        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.speed = self.velocity
        ackermann_msg.drive.steering_angle = steer
        self.cmd_pub.publish(ackermann_msg)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
