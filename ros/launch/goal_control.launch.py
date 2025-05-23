import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Parameters
    vehicle_frame_id = 'base_link'
    angular_proportional = 0.4
    linear_proportional = 1.0
    max_lin_vel = 0.2  # m/s

    bringup_jetson_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('caraware_ros'),
                'launch',
                'bringup_jetson.launch.py'
            )
        )
    )

    goal_controller = Node(
        package='caraware_ros',
        executable='goal_controller',
        name='goal_controller',
        output='screen',
        parameters=[
            {'frame_id': vehicle_frame_id},
            {'angular_proportional': angular_proportional},
            {'linear_proportional': linear_proportional},
            {'max_lin_vel': max_lin_vel},
        ],
        remappings=[
            ('/goal_pose', '/goal_pose'),
            ('/ackermann_cmd', '/ackermann_cmd'),
        ]
    )

    return LaunchDescription([
        bringup_jetson_launch,
        goal_controller,
    ])
