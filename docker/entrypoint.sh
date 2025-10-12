#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace if it exists
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

exec "$@"
