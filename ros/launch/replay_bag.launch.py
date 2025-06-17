import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Model bridge parameters
    obs_port = 5000
    pred_port = 5001
    reset_port = 5002
    model_rate = 50.0
    model_frame_id = 'model_frame_new'

    # Plot params
    world_frame = 'map'
    frames = ['base_link', 'model_frame', 'model_frame_new']

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
            ('/imu/data', '/imu/data_raw'),
            ('/ackermann_drive', '/ackermann_drive'),
            ('/odometry/filtered', '/odometry/filtered'),
            ('/model/prediction', '/model_new/prediction')
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
            'jetson.rviz')]
    )

    plotter = Node(
        package='caraware_ros',
        executable='plotter',
        name='plotter',
        output='screen',
        parameters=[
            {'frames': frames},
            {'world_frame': world_frame}
        ],
    )

    return LaunchDescription([
        model_bridge,
        plotter,
        rviz
    ])
