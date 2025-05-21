import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Vehicle parameters
    speed_to_erpm_gain = 4000.0  # Conversion factor from speed to erpm
    speed_to_erpm_offset = 0.0  # Offset for speed to erpm conversion
    steering_angle_to_servo_gain = 1.0  # Conversion factor from steering angle to servo position
    steering_angle_to_servo_offset = 0.0  # Offset for steering angle to servo position
    wheelbase = 0.2  # meters

    # Model bridge parameters
    obs_port = 5000
    pred_port = 5001
    reset_port = 5002
    model_rate = 50.0
    model_frame_id = 'model_frame'

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
            {'speed_to_erpm_gain': speed_to_erpm_gain},
            {'speed_to_erpm_offset': speed_to_erpm_offset},
            {'steering_angle_to_servo_gain': steering_angle_to_servo_gain},
            {'steering_angle_to_servo_offset': steering_angle_to_servo_offset},
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

    model_bridge = Node(
        package='caraware_ros',
        executable='model_bridge',
        name='model_bridge',
        output='screen',
        parameters=[
            {'obs_port': obs_port},
            {'pred_port': pred_port},
            {'reset_port': reset_port},
            {'frame_id': model_frame_id},
            {'publish_rate': model_rate},
            {'plot_error': False},
            {'use_sim_time': True}
        ],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/speed_sas/ackermann', '/carla/EGO_1/Speed_SAS'),
            ('/odometry/filtered', '/odometry/filtered'),
            ('/model/prediction', '/model/prediction')
        ],
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
        model_bridge,
        motor_controller,
    ])
