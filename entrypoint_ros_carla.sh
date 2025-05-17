#!/bin/bash
python3 /root/carla_ws/src/CarAware/wait_for_carla.py

# Wait to ensure CARLA-CarAware is fully up
sleep 15

source /opt/ros/$ROS_DISTRO/setup.bash
source /root/carla_ws/install/setup.bash
ros2 launch caraware_ros bringup_carla.launch.py $@
