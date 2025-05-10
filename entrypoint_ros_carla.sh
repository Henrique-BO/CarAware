#!/bin/bash
python3 /root/carla_ws/src/CarAware/wait_for_carla.py

source /opt/ros/$ROS_DISTRO/setup.bash
source /root/carla_ws/install/setup.bash
ros2 launch caraware_ros bringup_carla.launch.py
