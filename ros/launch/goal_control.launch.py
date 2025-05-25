import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    declare_vehicle_frame = DeclareLaunchArgument(
        'vehicle_frame', default_value='base_link', description='Vehicle frame ID'
    )
    declare_kp_linear = DeclareLaunchArgument(
        'kp_linear', default_value='1.0', description='Linear proportional gain'
    )
    declare_kp_angular = DeclareLaunchArgument(
        'kp_angular', default_value='0.4', description='Angular proportional gain'
    )
    declare_max_vel = DeclareLaunchArgument(
        'max_vel', default_value='0.1', description='Maximum linear velocity (m/s)'
    )

    # Parameters
    vehicle_frame = LaunchConfiguration('vehicle_frame')
    kp_linear = LaunchConfiguration('kp_linear')
    kp_angular = LaunchConfiguration('kp_angular')
    max_vel = LaunchConfiguration('max_vel')

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
            {'frame_id': vehicle_frame},
            {'kp_angular': kp_angular},
            {'kp_linear': kp_linear},
            {'max_vel': max_vel},
        ],
        remappings=[
            ('/goal_pose', '/goal_pose'),
            ('/ackermann_cmd', '/vesc/ackermann_cmd'),
        ]
    )

    return LaunchDescription([
        declare_vehicle_frame,
        declare_kp_angular,
        declare_kp_linear,
        declare_max_vel,
        bringup_jetson_launch,
        goal_controller,
    ])
