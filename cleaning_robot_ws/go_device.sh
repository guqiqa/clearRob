#!/bin/bash
# Run this on the device to deploy and launch Phase 1
pkill -9 -f ros2 2>/dev/null
systemctl stop LawnMower 2>/dev/null
sleep 1

cd /root/cleaning_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select remote_driver --symlink-install 2>&1 | tail -3

source install/setup.bash
rm -f /tmp/phase1_test.log
nohup ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:=false > /tmp/phase1_test.log 2>&1 &
echo "Launched PID=$!"

sleep 8
echo ""
echo "=== Calibration ==="
grep "Calibrat" /tmp/phase1_test.log
echo ""
echo "=== Nodes ==="
grep "STANDBY" /tmp/phase1_test.log
echo ""
echo "=== Watch channels (Ctrl+C to stop) ==="
sleep 2
grep --line-buffered "SBUS:" /tmp/phase1_test.log | tail -f
