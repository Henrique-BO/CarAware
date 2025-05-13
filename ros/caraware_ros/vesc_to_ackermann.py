import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from vesc_msgs.msg import VescStateStamped
from ackermann_msgs.msg import AckermannDriveStamped

class VescToAckermannNode(Node):
    def __init__(self):
        super().__init__('vesc_to_ackermann')
        self.subscription = self.create_subscription(
            VescStateStamped,
            '/vesc/sensors/core',
            self.vesc_callback,
            10
        )
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            'ackermann_drive',
            10
        )
        self.get_logger().info("VescToAckermannNode has been started.")

    def vesc_callback(self, msg: VescStateStamped):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header = msg.header
        ackermann_msg.drive.speed = msg.state.speed
        ackermann_msg.drive.steering_angle = msg.state.duty_cycle  # TODO check

        self.publisher.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VescToAckermannNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()