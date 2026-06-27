#!/bin/bash
# Pure height‑only takeoff for Pluto drone
# Sources ROS 2 environment
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/dell/OneDrive/Desktop/dashboard/pluto_ros2_package

# Ensure the dashboard node is running (it provides the /drone_command topic)
# Arm the drone with a neutral RC (roll/pitch/yaw/aux all neutral)
ros2 topic pub -1 /drone_command custom_msgs/msg/PlutoMsg "rc_roll:1500 rc_pitch:1500 rc_yaw:1500 rc_throttle:1300 rc_aux1:1500 rc_aux2:1500 rc_aux3:1000 rc_aux4:1200 command_type:0"
# Small pause to let the flight controller register the arm state
sleep 1
# Gradually increase throttle to lift the drone – keep other channels neutral
for th in {1300..1800..50}; do
  ros2 topic pub -1 /drone_command custom_msgs/msg/PlutoMsg "rc_roll:1500 rc_pitch:1500 rc_yaw:1500 rc_throttle:${th} rc_aux1:1500 rc_aux2:1500 rc_aux3:1000 rc_aux4:1500 command_type:0"
  sleep 0.2
done
# Hold at the final throttle for a few seconds (drone will stabilize at altitude)
sleep 5
# Optionally land later with a separate script
