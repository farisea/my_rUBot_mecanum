#!/bin/bash
set -e

# Default communication variables if they were not injected by docker compose
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-6}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

cd /root/ROS2_rUBot_mecanum_ws

exec ros2 launch my_robot_bringup my_robot_bringup_hw.launch.py