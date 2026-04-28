#!/bin/bash
set -e

# ROS 2 base environment
source /opt/ros/humble/setup.bash

# Orbbec workspace
if [ -f /root/orbbec_ws/install/setup.bash ]; then
  source /root/orbbec_ws/install/setup.bash
fi

# Main robot workspace
if [ -f /root/ROS2_rUBot_mecanum_ws/install/setup.bash ]; then
  source /root/ROS2_rUBot_mecanum_ws/install/setup.bash
fi

exec "$@"