from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    carla_ros_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('carla_ros_bridge'),
                'carla_ros_bridge.launch.py'
            )
        ),
        launch_arguments={
            'carla_host': 'localhost',
            'carla_port': '2000',
            'timeout': '10',
            'passive': 'True',
            'register_all_sensors': 'True'
        }.items()
    )

    model_node = Node(
        package='caraware_ros',
        executable='model_node',
        name='model_node',
        output='screen'
    )

    return LaunchDescription([
        carla_ros_bridge_launch,
        model_node
    ])
