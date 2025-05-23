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

    # Controller parameters
    controller_frame_id = 'base_link'
    angular_proportional = 0.4
    linear_proportional = 1.0
    max_lin_vel = 0.2  # m/s
    invert_angular = False

    # IMU and VESC parameters
    vesc_frame_id = 'base_link'
    imu_frame_id = 'imu_frame'

    # -90 degrees rotation around the Z-axis
    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '-0.7071', '0.7071', 'base_link', imu_frame_id]
    )

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
            {'frame_id': vesc_frame_id},
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

    controller_node = Node(
        package='caraware_ros',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[
            {'frame_id': controller_frame_id},
            {'angular_proportional': angular_proportional},
            {'linear_proportional': linear_proportional},
            {'max_lin_vel': max_lin_vel},
            {'invert_angular': invert_angular},
            {'speed_to_erpm_gain': speed_to_erpm_gain},
            {'speed_to_erpm_offset': speed_to_erpm_offset},
            {'steering_angle_to_servo_gain': steering_angle_to_servo_gain},
            {'steering_angle_to_servo_offset': steering_angle_to_servo_offset},
            {'wheelbase': wheelbase},
        ],
        remappings=[
            ('/goal_pose', '/goal_pose'),
            ('/vesc/commands/motor/duty_cycle', '/vesc/commands/motor/duty_cycle'),
            ('/vesc/commands/servo/position', '/vesc/commands/servo/position'),
        ]
    )

    return LaunchDescription([
        base_to_imu,
        imu_bringup_launch,
        motor_bringup_launch,
        vesc_republisher,
        robot_localization_ekf,
        model_bridge,
        controller_node,
    ])
