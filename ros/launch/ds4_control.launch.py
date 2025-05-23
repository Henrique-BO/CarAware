import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_jetson_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('caraware_ros'),
                'launch',
                'bringup_jetson.launch.py'
            )
        )
    )

    ds4_driver = Node(
        package='ds4_driver',
        executable='ds4_driver_node.py',
        name='ds4_driver',
        namespace='ds4',
        output='screen',
        parameters=[],
        respawn=True
    )

    ds4_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ds4_to_imu',
        namespace='ds4',
        output='screen',
        arguments=['0', '0.05', '-0.01', '-1.5707', '0', '1.5707', 'ds4', 'ds4_imu'],
        respawn=True
    )

    ds4_to_vesc = Node(
        package='quark_motor_bringup',
        executable='ds4_to_vesc',
        name='ds4_to_vesc',
        output='screen',
        parameters=[
            {'disable_servo_control': False}
        ],
        respawn=True
    )

    return LaunchDescription([
        bringup_jetson_launch,
        ds4_driver,
        ds4_to_imu,
        ds4_to_vesc,
    ])
