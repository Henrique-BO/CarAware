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

    robot_localization_file_path = os.path.join(
        get_package_share_directory('caraware_ros'),
        'config',
        'ekf.yaml'
    )

    # Start robot localization using an Extended Kalman filter
    robot_localization_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            robot_localization_file_path,
            # {'use_sim_time': True}
        ],
        remappings=[
            ('/carla/EGO_1/IMU', '/carla/EGO_1/IMU'),
            ('/carla/EGO_1/Speed_SAS/twist', '/carla/EGO_1/Speed_SAS/twist'),
        ]
    )

    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'EGO_1/IMU']
    )

    base_to_speed_sas = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'EGO_1/Speed_SAS']
    )

    # model_node = Node(
    #     package='caraware_ros',
    #     executable='model_node',
    #     name='model_node',
    #     output='screen',
    #     parameters=[
    #         {'imu_topic': '/carla/EGO_1/IMU'},
    #         {'ackermann_topic': '/carla/EGO_1/Speed_SAS'},
    #         {'publish_rate': 50.0}
    #     ],
    #     remappings=[
    #         ('vehicle_pose', '/model/vehicle_pose')
    #     ]
    # )

    return LaunchDescription([
        carla_ros_bridge_launch,
        robot_localization_ekf,
        base_to_imu,
        base_to_speed_sas,
        # model_node
    ])
