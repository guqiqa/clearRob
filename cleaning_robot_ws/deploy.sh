#!/bin/bash
# Phase 1 Setup — run this on the device
set -e
echo "=== Step 1: Install pip packages ==="
pip3 install /tmp/pyserial-3.5-py2.py3-none-any.whl /tmp/python_can-4.6.1-py3-none-any.whl 2>&1 | grep -v "^Requirement\|^WARNING"

# Verify
python3 -c "import serial; print('pyserial OK')" || echo "pyserial FAILED"
python3 -c "import can; print('python-can OK')" || echo "python-can FAILED"

echo ""
echo "=== Step 2: Build workspace ==="
cd /root/cleaning_robot_ws
source /opt/ros/humble/setup.bash
colcon build --executor sequential --symlink-install 2>&1 | tail -15

echo ""
echo "=== Step 3: Launch Phase 1 ==="
source install/setup.bash
systemctl stop LawnMower 2>/dev/null
pkill -9 -f ros2 2>/dev/null
sleep 1
ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:=false > /tmp/phase1.log 2>&1 &
echo "Launched PID=$!"
sleep 15
echo ""
echo "=== Step 4: Status ==="
grep "Calibrat\|→" /tmp/phase1.log 2>/dev/null | head -5
echo ""
grep "ERROR\|FATAL" /tmp/phase1.log 2>/dev/null | head -5 || echo "No errors"
echo ""
echo "Done! Check: tail -f /tmp/phase1.log"
