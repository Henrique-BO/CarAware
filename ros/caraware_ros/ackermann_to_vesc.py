import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

class AckermannToVesc(Node):
    def __init__(self):
        super().__init__('ackermann_to_vesc_node')

        # Declare and get parameters
        self.speed_to_duty_cycle_gain = self.declare_parameter('speed_to_duty_cycle_gain', 1.0).value
        self.speed_to_duty_cycle_offset = self.declare_parameter('speed_to_duty_cycle_offset', 0.0).value
        self.steering_to_servo_gain = self.declare_parameter('steering_angle_to_servo_gain', 1.0).value
        self.steering_to_servo_offset = self.declare_parameter('steering_angle_to_servo_offset', 0.0).value

        # Publishers
        self.duty_cycle_pub = self.create_publisher(Float64, '/vesc/commands/motor/duty_cycle', 10)
        self.servo_pub = self.create_publisher(Float64, '/vesc/commands/servo/position', 10)

        # Subscriber
        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            'ackermann_cmd',
            self.ackermann_cmd_callback,
            10
        )

    def ackermann_cmd_callback(self, msg):
        # Calculate duty cycle
        duty_cycle_msg = Float64()
        duty_cycle_msg.data = self.speed_to_duty_cycle_gain * msg.drive.speed + self.speed_to_duty_cycle_offset

        # Calculate servo position
        servo_msg = Float64()
        servo_msg.data = self.steering_to_servo_gain * msg.drive.steering_angle + self.steering_to_servo_offset

        # Publish messages
        self.duty_cycle_pub.publish(duty_cycle_msg)
        self.servo_pub.publish(servo_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannToVesc()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
