#!/bin/bash
# Source ROS 2 Humble environment
source /opt/ros/humble/setup.bash
# Build the workspace with symlink install for rapid iteration
colcon build --symlink-install
