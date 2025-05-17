from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    imu_frame_id = 'imu_frame'
    twist_frame_id = 'twist_frame'
    wheelbase = 2.0

    obs_port = 5000
    pred_port = 5001
    reset_port = 5002
    model_rate = 50.0
    model_frame_id = 'model_frame'

    plot_error_arg = DeclareLaunchArgument(
        'plot_error',
        default_value='false',
        description='Plot error in the model node'
    )
    plot_error = LaunchConfiguration('plot_error')

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

    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', imu_frame_id]
    )

    base_to_speed_sas = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', twist_frame_id]
    )

    carla_republisher = Node(
        package='caraware_ros',
        executable='carla_republisher',
        name='carla_republisher',
        output='screen',
        parameters=[
            {'imu_frame_id': imu_frame_id},
            {'twist_frame_id': twist_frame_id},
            {'wheelbase': wheelbase},
            {'use_sim_time': True}
        ],
        remappings=[
            ('/imu/data_in', '/carla/EGO_1/IMU'),
            ('/imu/data_out', '/imu/data'),
            ('/speed_sas/ackermann_in', '/carla/EGO_1/Speed_SAS'),
            ('/speed_sas/twist_out', '/speed_sas/twist'),
        ]
    )

    calculate_map_to_odom = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'service',
                    'call',
                    '/calculate_map_to_odom',
                    'std_srvs/srv/Trigger',
                    '{}'
                ],
                output='screen'
            )
        ]
    )

    robot_localization_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('caraware_ros'),
                'config',
                'ekf.yaml'
            ),
            {'use_sim_time': True}
        ],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/twist', '/speed_sas/twist'),
        ]
    )

    model_node = Node(
        package='caraware_ros',
        executable='model_node',
        name='model_bridge',
        output='screen',
        parameters=[
            {'obs_port': obs_port},
            {'pred_port': pred_port},
            {'reset_port': reset_port},
            {'frame_id': model_frame_id},
            {'publish_rate': model_rate},
            {'plot_error': plot_error},
            {'use_sim_time': True}
        ],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/speed_sas/ackermann', '/carla/EGO_1/Speed_SAS'),
            ('/odometry/filtered', '/odometry/filtered'),
            ('/model/prediction', '/model/prediction')
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(
            get_package_share_directory('caraware_ros'),
            'config',
            'localization.rviz')]
    )

    return LaunchDescription([
        plot_error_arg,
        carla_ros_bridge_launch,
        base_to_imu,
        base_to_speed_sas,
        carla_republisher,
        calculate_map_to_odom,
        robot_localization_ekf,
        model_node,
        rviz,
    ])
