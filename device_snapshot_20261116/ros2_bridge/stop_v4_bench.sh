#!/bin/bash
set +u
source /root/nav_ws/env.sh
timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
bash /root/ros2_bridge/stop_nav.sh >/dev/null 2>&1
pkill -f RemoteControl_WheelPID_V2 2>/dev/null
pkill -f sbus_emu 2>/dev/null
pkill -f flat_ground_grid 2>/dev/null
pkill -f demo_mission_manager 2>/dev/null
sleep 1
pgrep -af "RemoteControl|sbus_emu|navigation_node|safety_supervisor|flat_ground_grid" | grep -v pgrep || echo "[v4] all stopped"
