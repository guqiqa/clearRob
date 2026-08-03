#!/bin/bash
# Auto-deploy script — runs on device
pkill -9 -f ros2 2>/dev/null
systemctl stop LawnMower 2>/dev/null
sleep 1
cd /root/cleaning_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select remote_driver --symlink-install > /tmp/build.log 2>&1
source install/setup.bash
rm -f /tmp/phase1_auto.log
nohup ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:=false > /tmp/phase1_auto.log 2>&1 &
echo "PID=$!"
sleep 2
echo "=== BUILD ==="
tail -3 /tmp/build.log
echo "=== PHASE1 ==="
sleep 5
grep -E "Calibrat|STANDBY|ERROR|FAIL" /tmp/phase1_auto.log 2>/dev/null | head -5
