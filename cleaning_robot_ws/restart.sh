#!/bin/bash
pkill -9 -f ros2 2>/dev/null
sleep 1
cd /root/cleaning_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select remote_driver --symlink-install 2>&1 | tail -5
rm -f /tmp/phase1_
source install/setup.bash
nohup ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:=false > /tmp/phase1_restart.log 2>&1 &
sleep 8
echo "=== BUILD DONE ==="
grep -E 'Auto-calibrat|STANDBY|ERROR|died' /tmp/phase1_restart.log
