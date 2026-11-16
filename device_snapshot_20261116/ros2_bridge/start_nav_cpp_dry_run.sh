#!/bin/bash
# 正式 C++ 底盘 + 导航台架链路（过渡 Python 速度节点不再使用）
set +u
. /root/nav_ws/env.sh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
BRIDGE=/root/ros2_bridge
pkill -f hardware_adapter 2>/dev/null
pkill -f wheel_speed_node 2>/dev/null
pkill -f RemoteControl_WheelPID_V2 2>/dev/null
pkill -f sbus_emu 2>/dev/null
bash "$BRIDGE/stop_nav.sh" >/dev/null 2>&1
sleep 1
cd /root/chassis_ros2_nav_v3
rm -f /tmp/sbus_emu_path
setsid nohup python3 "$BRIDGE/sbus_emu.py" > /tmp/sbus_cpp.log 2>&1 < /dev/null &
for i in $(seq 1 30); do [ -s /tmp/sbus_emu_path ] && break; sleep 0.1; done
PTS=$(cat /tmp/sbus_emu_path 2>/dev/null)
sed "s#<SBus_Port>[^<]*</SBus_Port>#<SBus_Port>$PTS</SBus_Port>#" config.xml > config_emu.xml
setsid nohup ./build/RemoteControl_WheelPID_V2 config_emu.xml > /tmp/chassis_cpp.log 2>&1 < /dev/null &
sleep 4
setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/nav_cpp_dry.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 /root/nav_ws/scripts/hardware_adapter.py > /tmp/hw_adapter_cpp.log 2>&1 < /dev/null &
sleep 2
timeout 10 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" > /dev/null 2>&1
timeout 10 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" > /dev/null 2>&1
echo STARTED