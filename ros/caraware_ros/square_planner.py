# planner_node.py
import rclpy
import math

from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
from transforms3d.euler import euler2quat, quat2euler

class SquarePlanner(Node):
    def __init__(self):
        super().__init__('square_planner')
        self.declare_parameter('radius', 10.0)
        self.declare_parameter('side_length', 100.0)
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('horizon', 30)
        self.declare_parameter('frame_id', 'base_link')

        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.side = self.get_parameter('side_length').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.horizon = self.get_parameter('horizon').get_parameter_value().integer_value
        self.vehicle_frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.vehicle_pose = None
        self.closest_index = 0

        self.full_path = Path()
        self.last_published_start_index = 0
        self.last_path_stamp = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.feedback_sub = self.create_subscription(Int32, '/closest_index_feedback', self.feedback_callback, 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.full_path_pub = self.create_publisher(Path, '/full_path', 10)

        self.timer = self.create_timer(0.2, self.publish_horizon)

        self.create_service(Trigger, 'generate_plan', self.handle_plan_request)
        self.get_logger().info("Square Planner Node Initialized")

    def feedback_callback(self, msg):
        adjusted_index = self.last_published_start_index + msg.data
        if adjusted_index >= 0:
            self.closest_index = min(adjusted_index, len(self.full_path.poses) - 1)
            self.get_logger().info(f"Adjusted closest index: {self.closest_index}")
        else:
            self.get_logger().warn(f"Feedback index out of bounds: {adjusted_index}")

    def get_vehicle_pose(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform('odom', self.vehicle_frame_id, rclpy.time.Time())
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

        # Generate the path as a nav_msgs/Path message
        self.full_path = self.generate_square_path(self.vehicle_pose)
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

    def transform_point(self, x_rel, y_rel, pose):
        q = pose.pose.orientation
        _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
        x_base = pose.pose.position.x + math.cos(yaw) * x_rel - math.sin(yaw) * y_rel
        y_base = pose.pose.position.y + math.sin(yaw) * x_rel + math.cos(yaw) * y_rel
        return x_base, y_base

    def generate_square_path(self, start_pose):
        r = self.radius
        L = self.side

        segments = []
        segments += self.interpolate_segment((r, 0), (L - r, 0))
        # segments += self.arc(L - r, r, -math.pi / 2, 0.0, r)
        # segments += self.interpolate_segment((L, r), (L, L - r))
        # segments += self.arc(L - r, L - r, 0.0, math.pi / 2, r)
        # segments += self.interpolate_segment((L - r, L), (r, L))
        # segments += self.arc(r, L - r, math.pi / 2, math.pi, r)
        # segments += self.interpolate_segment((0, L - r), (0, r))
        # segments += self.arc(r, r, math.pi, 3 * math.pi / 2, r)

        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        prev_x = prev_y = None
        for x_rel, y_rel in segments:
            x_map, y_map = self.transform_point(x_rel, y_rel, start_pose)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x_map
            pose.pose.position.y = y_map

            if prev_x is not None and prev_y is not None:
                dx = x_map - prev_x
                dy = y_map - prev_y
                yaw = math.atan2(dy, dx)
                q = euler2quat(0, 0, yaw)
                pose.pose.orientation.x = q[1]
                pose.pose.orientation.y = q[2]
                pose.pose.orientation.z = q[3]
                pose.pose.orientation.w = q[0]
            else:
                pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)
            prev_x, prev_y = x_map, y_map

        return path_msg

    def publish_horizon(self):
        if not self.full_path.poses or self.vehicle_pose is None:
            self.full_path_pub.publish(Path())
            self.path_pub.publish(Path())
            return

        # Reset the path to null if the vehicle has reached the end
        if self.closest_index >= len(self.full_path.poses):
            self.get_logger().info("Vehicle has reached the end of the path.")
            self.full_path = Path()
            self.full_path_pub.publish(self.full_path)
            self.path_pub.publish(Path())
            return

        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        start_index = self.closest_index
        end_index = min(start_index + self.horizon, len(self.full_path.poses))
        path_msg.poses = self.full_path.poses[start_index:end_index]

        self.path_pub.publish(path_msg)
        self.full_path_pub.publish(self.full_path)

        self.last_published_start_index = start_index
        self.last_path_stamp = path_msg.header.stamp


def main(args=None):
    rclpy.init(args=args)
    node = SquarePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
