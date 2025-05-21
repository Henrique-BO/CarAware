from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Parameters
    wheelbase = 0.2  # meters

    imu_bringup_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('quark_imu_bringup'),
                'launch',
                'bmi160.launch.yaml'
            )
        )
    )

    motor_bringup_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('quark_motor_bringup'),
                'launch',
                'bringup_ds4.launch.yml'
            )
        )
    )

    vesc_republisher = Node(
        package='caraware_ros',
        executable='vesc_republisher',
        name='vesc_republisher',
        output='screen',
        parameters=[
            {'wheelbase': wheelbase},
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
            ('/twist', '/twist'),
        ]
    )

    model_node = Node(
        package='caraware_ros',
        executable='model_node',
        name='model_node',
        output='screen',
        parameters=[
            {'imu_topic': '/imu/data'},
            {'ackermann_topic': '/ackermann_drive'},
            {'publish_rate': 50.0}
        ],
        remappings=[
            ('vehicle_pose', '/model/vehicle_pose')
        ]
    )

    motor_controller = Node(
        package='quark_motor_control',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[
        ],
        remappings=[
        ]
    )

    return LaunchDescription([
        imu_bringup_launch,
        motor_bringup_launch,
        vesc_republisher,
        robot_localization_ekf,
        model_node,
        motor_controller,
    ])
