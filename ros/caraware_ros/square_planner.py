# planner_node.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
import math
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

class RoundedSquarePlanner(Node):
    def __init__(self):
        super().__init__('rounded_square_planner')
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('side_length', 10.0)
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('horizon', 30)

        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.side = self.get_parameter('side_length').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.horizon = self.get_parameter('horizon').get_parameter_value().integer_value

        self.vehicle_pose = None
        self.closest_index = 0

        self.full_path = []
        self.last_published_start_index = 0
        self.last_path_stamp = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.feedback_sub = self.create_subscription(Int32, '/closest_index_feedback', self.feedback_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.timer = self.create_timer(0.2, self.publish_horizon)

        self.create_service(Trigger, 'generate_plan', self.handle_plan_request)

    def feedback_callback(self, msg):
        adjusted_index = self.last_published_start_index + msg.data
        if adjusted_index < len(self.full_path):
            self.closest_index = adjusted_index
            self.get_logger().debug(f"Adjusted closest index: {self.closest_index}")
        else:
            self.get_logger().warn(f"Feedback index out of bounds: {adjusted_index}")

    def get_vehicle_pose(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Failed to get transform: {e}")
            return None

    def handle_plan_request(self, request, response):
        self.vehicle_pose = self.get_vehicle_pose()
        if self.vehicle_pose is None:
            self.get_logger().warn("Failed to get vehicle pose.")
            response.success = False
            response.message = "Failed to get vehicle pose."
            return response

        self.full_path = self.generate_rounded_square_path(self.vehicle_pose)
        self.closest_index = 0
        self.get_logger().info("Generated rounded square plan.")
        response.success = True
        response.message = "Plan generated successfully."
        return response

    def interpolate_segment(self, start, end):
        x0, y0 = start
        x1, y1 = end
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(2, int(dist / self.resolution))
        return [(x0 + (x1 - x0) * i / steps, y0 + (y1 - y0) * i / steps) for i in range(steps)]

    def arc(self, cx, cy, start_angle, end_angle, radius):
        steps = max(4, int(abs(end_angle - start_angle) * radius / self.resolution))
        return [(cx + radius * math.cos(a), cy + radius * math.sin(a))
                for a in [start_angle + (end_angle - start_angle) * i / steps for i in range(steps)]]

    def generate_rounded_square_path(self, start_pose):
        path = []
        r = self.radius
        L = self.side
        cx, cy = start_pose.pose.position.x, start_pose.pose.position.y

        # Bottom side
        path += self.interpolate_segment((cx + r, cy), (cx + L - r, cy))
        path += self.arc(cx + L - r, cy + r, -math.pi/2, 0.0, r)

        # Right side
        path += self.interpolate_segment((cx + L, cy + r), (cx + L, cy + L - r))
        path += self.arc(cx + L - r, cy + L - r, 0.0, math.pi/2, r)

        # Top side
        path += self.interpolate_segment((cx + L - r, cy + L), (cx + r, cy + L))
        path += self.arc(cx + r, cy + L - r, math.pi/2, math.pi, r)

        # Left side
        path += self.interpolate_segment((cx, cy + L - r), (cx, cy + r))
        path += self.arc(cx + r, cy + r, math.pi, 3 * math.pi / 2, r)

        return path

    def publish_horizon(self):
        if not self.full_path or self.vehicle_pose is None:
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        start_index = self.closest_index
        end_index = min(start_index + self.horizon, len(self.full_path))

        for x, y in self.full_path[start_index:end_index]:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.last_published_start_index = start_index
        self.last_path_stamp = path_msg.header.stamp


def main(args=None):
    rclpy.init(args=args)
    node = RoundedSquarePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
