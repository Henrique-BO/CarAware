import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Stanley parameters
    k = 10.0  # gain parameter
    velocity = 0.05  # constant speed
    vehicle_frame_id = 'base_link'  # vehicle frame ID

    # Square planner parameters
    radius = 0.1  # radius of the rounded corners
    side = 0.6  # length of the sides of the square
    resolution = 0.01  # interpolation resolution
    horizon = 30  # number of points in the horizon

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
            {'vehicle_frame_id': vehicle_frame_id},
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
        bringup_jetson_launch,
        stanley_controller_node,
        square_planner_node,
    ])
