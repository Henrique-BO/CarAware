import rclpy
import numpy as np
import transforms3d
import argparse

from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class OrientationVizNode(Node):
    def __init__(self, frame_id, child_frame_id):
        super().__init__('orientation_viz')
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id

        # Subscribe to the /tf topic
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == self.frame_id and transform.child_frame_id == self.child_frame_id:
                # Extract quaternion from the transform
                orientation_q = transform.transform.rotation
                quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

                # Convert quaternion to Euler angles
                roll, pitch, yaw = transforms3d.euler.quat2euler(quaternion)
                roll = np.degrees(roll)
                pitch = np.degrees(pitch)
                yaw = np.degrees(yaw)

                # Print the Euler angles
                self.get_logger().info(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

def main(args=None):
    rclpy.init(args=args)

    # Parse arguments for frame_id and child_frame_id
    parser = argparse.ArgumentParser(description='Orientation Visualization Node')
    parser.add_argument('frame_id', type=str, help='The frame_id to filter')
    parser.add_argument('child_frame_id', type=str, help='The child_frame_id to filter')
    parsed_args = parser.parse_args()

    node = OrientationVizNode(parsed_args.frame_id, parsed_args.child_frame_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()