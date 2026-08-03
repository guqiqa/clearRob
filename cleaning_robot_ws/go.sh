#!/bin/bash
# Phase 1 — Final Launcher (offline, no colcon, no ros2 launch needed)
# Run: bash /tmp/go.sh

set -e
WS="/root/cleaning_robot_ws"

# Fix entry points for imu/rtk (still mangled)
cat > "$WS/install/imu_driver/lib/imu_driver/imu_node" << 'EOF'
#!/usr/bin/env python3
import sys; sys.path.insert(0,'/root/cleaning_robot_ws/src/cleaning_robot_common'); sys.path.insert(0,'/root/cleaning_robot_ws/src')
import rclpy; rclpy.init(args=sys.argv)
from imu_driver.imu_node import main; main()
EOF

cat > "$WS/install/rtk_driver/lib/rtk_driver/rtk_node" << 'EOF'
#!/usr/bin/env python3
import sys; sys.path.insert(0,'/root/cleaning_robot_ws/src/cleaning_robot_common'); sys.path.insert(0,'/root/cleaning_robot_ws/src')
import rclpy; rclpy.init(args=sys.argv)
from rtk_driver.rtk_node import main; main()
EOF
chmod +x "$WS"/install/*/lib/*/*

# Setup
mkdir -p /root/.ros/log
pkill -9 -f ros2 2>/dev/null || true
systemctl stop LawnMower 2>/dev/null || true
sleep 1

# Launch
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="$WS/install/chassis_driver:$WS/install/remote_driver:$WS/install/imu_driver:$WS/install/rtk_driver:$WS/install/master_bridge:$WS/install/cleaning_robot_common:$WS/install/cleaning_robot_bringup:/opt/ros/humble"

ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:=false > /tmp/phase1_final.log 2>&1 &
echo "Launched PID=$!"
sleep 10
echo ""
echo "=== STATUS ==="
grep -E "port_bridge|Calibrat|→|ERROR.*died" /tmp/phase1_final.log 2>/dev/null | head -10
echo ""
echo "Monitor: tail -f /tmp/phase1_final.log"
echo "Check:  source /opt/ros/humble/setup.bash && ros2 topic echo /master/state --once"
