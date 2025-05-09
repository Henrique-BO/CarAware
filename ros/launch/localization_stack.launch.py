from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    imu_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('quark_imu_bringup'),
                'launch',
                'bmi160.launch.py'
            )
        )
    )

    # motor_bringup_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('quark_motor_bringup'),
    #             'launch',
    #             'bringup.launch.py'
    #         )
    #     )
    # )

    model_node = Node(
        package='caraware_ros',
        executable='model_node',
        name='model_node',
        output='screen'
    )

    return LaunchDescription([
        imu_bringup_launch,
        # motor_bringup_launch,
        model_node
    ])
