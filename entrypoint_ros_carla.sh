#!/bin/bash
python3 /root/carla_ws/src/CarAware/wait_for_carla.py

source /opt/ros/$ROS_DISTRO/setup.bash
source /root/carla_ws/install/setup.bash
ros2 launch caraware_ros carla_bridge_model.launch.py

# source /opt/ros/$ROS_DISTRO/setup.bash && source /root/carla_ws/install/setup.bash && ros2 launch carla_ros_bridge carla_ros_bridge.launch.py carla_host:=localhost carla_port:=2000 timeout:=10 passive:=True register_all_sensors:=True
