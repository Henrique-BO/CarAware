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
    speed_to_duty_cycle_gain = 1.0  # Conversion factor from speed to erpm
    speed_to_duty_cycle_offset = 0.0  # Offset for speed to erpm conversion
    steering_angle_to_servo_gain = 1.0  # Conversion factor from steering angle to servo position
    steering_angle_to_servo_offset = 0.5  # Offset for steering angle to servo position
    wheelbase = 0.2  # meters

    # Model bridge parameters
    obs_port = 5000
    pred_port = 5001
    reset_port = 5002
    model_rate = 50.0
    model_frame_id = 'model_frame'

    # IMU and VESC parameters
    vesc_frame_id = 'base_link'
    imu_frame_id = 'imu_frame'

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # IMU frame transformation
    # - X right
    # - Y back
    # - Z up
    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.57079632679', '0', '3.14159265359', 'base_link', imu_frame_id]
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

    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        namespace='vesc',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('caraware_ros'),
                'config',
                'vesc_config.yml'
            )
        ]
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
            {'use_sim_time': False},
        ],
        remappings=[
            ('/imu/data', '/imu/data_raw'),
            ('/twist', '/twist'),
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
        ],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/speed_sas/ackermann', '/carla/EGO_1/Speed_SAS'),
            ('/odometry/filtered', '/odometry/filtered'),
            ('/model/prediction', '/model/prediction')
        ],
    )

    ackermann_to_vesc = Node(
        package='caraware_ros',
        executable='ackermann_to_vesc',
        name='ackermann_to_vesc',
        output='screen',
        namespace='/vesc',
        parameters=[
            {'speed_to_duty_cycle_gain': speed_to_duty_cycle_gain},
            {'speed_to_duty_cycle_offset': speed_to_duty_cycle_offset},
            {'steering_angle_to_servo_gain': steering_angle_to_servo_gain},
            {'steering_angle_to_servo_offset': steering_angle_to_servo_offset},
        ],
        remappings=[
            ('/ackermann_cmd', '/ackermann_cmd'),
            ('/vesc/commands/motor/duty_cycle', '/vesc/commands/motor/duty_cycle'),  # Updated topic
            ('/vesc/commands/servo/position', '/vesc/commands/servo/position'),
        ]
    )

    return LaunchDescription([
        map_to_odom,
        base_to_imu,
        imu_bringup_launch,
        vesc_driver_node,
        vesc_republisher,
        robot_localization_ekf,
        model_bridge,
        ackermann_to_vesc,
    ])
