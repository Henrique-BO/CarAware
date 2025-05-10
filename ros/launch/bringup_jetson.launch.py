from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    imu_bringup_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('quark_imu_bringup'),
                'launch',
                'bmi160.launch.yaml'
            )
        )
    )

    motor_bringup_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('quark_motor_bringup'),
                'launch',
                'bringup_ds4.launch.yml'
            )
        )
    )

    vesc_to_ackermann = Node(
        package='caraware_ros',
        executable='vesc_to_ackermann',
        name='vesc_to_ackermann',
        output='screen'
    )

    model_node = Node(
        package='caraware_ros',
        executable='model_node',
        name='model_node',
        output='screen',
        parameters=[
            {'imu_topic': '/imu/data'},
            {'ackermann_topic': '/ackermann_drive'},
            {'publish_rate': 50.0}
        ],
        remappings=[
            ('vehicle_pose', '/model/vehicle_pose')
        ]
    )

    return LaunchDescription([
        imu_bringup_launch,
        motor_bringup_launch,
        vesc_to_ackermann,
        model_node,
    ])
