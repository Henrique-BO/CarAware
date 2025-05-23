import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    declare_vehicle_frame_id = DeclareLaunchArgument(
        'vehicle_frame_id', default_value='base_link', description='Vehicle frame ID'
    )
    declare_angular_proportional = DeclareLaunchArgument(
        'angular_proportional', default_value='0.4', description='Angular proportional gain'
    )
    declare_linear_proportional = DeclareLaunchArgument(
        'linear_proportional', default_value='1.0', description='Linear proportional gain'
    )
    declare_max_lin_vel = DeclareLaunchArgument(
        'max_lin_vel', default_value='0.2', description='Maximum linear velocity (m/s)'
    )
    declare_invert_angular = DeclareLaunchArgument(
        'invert_angular', default_value='false', description='Invert angular control'
    )

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
            {'frame_id': LaunchConfiguration('vehicle_frame_id')},
            {'angular_proportional': LaunchConfiguration('angular_proportional')},
            {'linear_proportional': LaunchConfiguration('linear_proportional')},
            {'max_lin_vel': LaunchConfiguration('max_lin_vel')},
            {'invert_angular': LaunchConfiguration('invert_angular')},
        ],
        remappings=[
            ('/goal_pose', '/goal_pose'),
            ('/ackermann_cmd', '/vesc/ackermann_cmd'),
        ]
    )

    return LaunchDescription([
        declare_vehicle_frame_id,
        declare_angular_proportional,
        declare_linear_proportional,
        declare_max_lin_vel,
        declare_invert_angular,
        bringup_jetson_launch,
        goal_controller,
    ])
