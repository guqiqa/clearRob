#!/bin/bash
# deploy.sh — run me on the device
pkill -9 -f ros2 2>/dev/null
systemctl stop LawnMower 2>/dev/null
sleep 1
cd /root/cleaning_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select remote_driver --symlink-install 2>&1 | tail -3
source install/setup.bash
ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:=false > /tmp/phase1_live.log 2>&1 &
echo "Phase 1 launched. Check: tail -f /tmp/phase1_live.log"
