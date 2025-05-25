import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare launch arguments
    declare_k = DeclareLaunchArgument('k', default_value='10.0', description='Gain parameter for Stanley controller')
    declare_velocity = DeclareLaunchArgument('velocity', default_value='0.05', description='Constant speed for Stanley controller')
    declare_vehicle_frame = DeclareLaunchArgument('vehicle_frame', default_value='base_link', description='Vehicle frame ID')

    declare_radius = DeclareLaunchArgument('radius', default_value='0.1', description='Radius of the rounded corners for square planner')
    declare_side = DeclareLaunchArgument('side', default_value='0.6', description='Length of the sides of the square for square planner')
    declare_resolution = DeclareLaunchArgument('resolution', default_value='0.01', description='Interpolation resolution for square planner')
    declare_horizon = DeclareLaunchArgument('horizon', default_value='30', description='Number of points in the horizon for square planner')

    # Get parameters from launch arguments
    k = LaunchConfiguration('k')
    velocity = LaunchConfiguration('velocity')
    vehicle_frame = LaunchConfiguration('vehicle_frame')
    radius = LaunchConfiguration('radius')
    side = LaunchConfiguration('side')
    resolution = LaunchConfiguration('resolution')
    horizon = LaunchConfiguration('horizon')

    bringup_jetson_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('caraware_ros'),
                'launch',
                'bringup_jetson.launch.py'
            )
        )
    )

    stanley_controller_node = Node(
        package='caraware_ros',
        executable='stanley_controller',
        name='stanley_controller',
        output='screen',
        parameters=[
            {'k': k},
            {'velocity': velocity},
            {'frame_id': vehicle_frame},
        ],
        remappings=[
            ('/planned_path', '/planned_path'),
            ('/ackermann_cmd', '/vesc/ackermann_cmd'),
            ('/closest_index_feedback', '/closest_index_feedback'),
        ]
    )

    square_planner_node = Node(
        package='caraware_ros',
        executable='square_planner',
        name='square_planner',
        output='screen',
        parameters=[
            {'radius': radius},
            {'side_length': side},
            {'resolution': resolution},
            {'horizon': horizon},
        ],
        remappings=[
            ('/vehicle_pose', '/vehicle_pose'),
            ('/planned_path', '/planned_path'),
            ('/closest_index_feedback', '/closest_index_feedback'),
        ]
    )

    return LaunchDescription([
        declare_k,
        declare_velocity,
        declare_vehicle_frame,
        declare_radius,
        declare_side,
        declare_resolution,
        declare_horizon,
        bringup_jetson_launch,
        stanley_controller_node,
        square_planner_node,
    ])
