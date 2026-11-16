#!/bin/bash
# v4 台架链路：C++ 底盘直接消费 /safety/safe_motion_command（不启动 Python wheel_speed_node/hardware_adapter）
set +u
source /root/nav_ws/env.sh
BRIDGE=/root/ros2_bridge
BASE=/root/chassis_ros2_nav_v4
pkill -f RemoteControl_WheelPID_V2 2>/dev/null
pkill -f wheel_speed_node 2>/dev/null
pkill -f hardware_adapter 2>/dev/null
pkill -f flat_ground_grid 2>/dev/null
pkill -f demo_mission_manager 2>/dev/null
pkill -f sbus_emu 2>/dev/null
bash "$BRIDGE/stop_nav.sh" >/dev/null 2>&1
sleep 1
ip link set can0 down 2>/dev/null
ip link set can0 type can bitrate 500000 2>/dev/null
ip link set can0 up 2>/dev/null
ip link set can0 txqueuelen 1000 2>/dev/null
echo "[v4] can0: $(ip -br link show can0 2>/dev/null | awk '{print $2, $3}')"
rm -f /tmp/sbus_emu_path
setsid nohup python3 "$BRIDGE/sbus_emu.py" > /tmp/sbus_v4.log 2>&1 < /dev/null &
sleep 2
PTS=$(cat /tmp/sbus_emu_path 2>/dev/null)
if [ -z "$PTS" ]; then echo "[v4] SBUS emu failed"; exit 1; fi
echo "[v4] sbus=$PTS"
sed "s#<SBus_Port>[^<]*</SBus_Port>#<SBus_Port>$PTS</SBus_Port>#" "$BASE/src/config.xml" > "$BASE/src/config_emu.xml"
setsid nohup "$BASE/build/RemoteControl_WheelPID_V2" "$BASE/src/config_emu.xml" > /tmp/v4_chassis.log 2>&1 < /dev/null &
sleep 5
if ! pgrep -f "RemoteControl_WheelPID_V2 $BASE/src/config_emu.xml" >/dev/null; then echo "[v4] chassis driver failed"; tail -20 /tmp/v4_chassis.log; exit 1; fi
setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/nav_hw_dry.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 "$BRIDGE/flat_ground_grid.py" > /tmp/flat_grid.log 2>&1 < /dev/null &
sleep 1
timeout 10 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
timeout 10 ros2 node list
echo "[v4] chain ready: chassis manual, safety disabled, zero speed"
