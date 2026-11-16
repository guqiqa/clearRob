#!/bin/bash
# 反馈丢失注入测试（第二轮）：锁存→清除→恢复运动
set +u
source /root/nav_ws/env.sh
BRIDGE=/root/ros2_bridge; BASE=/root/chassis_ros2_nav_v4; OUT=/tmp/fi2_result.txt
: > $OUT
pkill -f RemoteControl_WheelPID_V2 2>/dev/null; pkill -f sbus_emu 2>/dev/null; pkill -f pub_safe_cmd 2>/dev/null
sleep 1
ip link set can0 down 2>/dev/null; ip link set can0 type can bitrate 500000 2>/dev/null; ip link set can0 up 2>/dev/null
rm -f /tmp/sbus_emu_path
setsid nohup python3 "$BRIDGE/sbus_emu.py" > /tmp/fi2_sbus.log 2>&1 < /dev/null &
sleep 2
PTS=$(cat /tmp/sbus_emu_path)
sed "s#<SBus_Port>[^<]*</SBus_Port>#<SBus_Port>$PTS</SBus_Port>#" "$BASE/src/config.xml" > "$BASE/src/config_emu.xml"
setsid nohup "$BASE/build/RemoteControl_WheelPID_V2" "$BASE/src/config_emu.xml" --inject-feedback-loss-after-s 8 --inject-feedback-loss-duration-s 4 > /tmp/v4_fi2.log 2>&1 < /dev/null &
sleep 3
timeout 10 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
setsid nohup python3 "$BRIDGE/pub_safe_cmd.py" 0.10 60 > /tmp/fi2_pub.log 2>&1 < /dev/null &
sleep 7
timeout 8 ros2 topic echo /chassis/state --once > /tmp/fi2_latched.txt 2>&1
timeout 8 ros2 topic echo /odom --once > /tmp/fi2_odom_latched.txt 2>&1
echo "[latched] state:" | tee -a $OUT
grep -E "control_mode|motion_allowed|feedback_valid|fault_reason|linear_velocity" /tmp/fi2_latched.txt | tee -a $OUT
echo "--- wait injection end, then mode off/on while publisher still active ---" | tee -a $OUT
sleep 4
timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
sleep 1
timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
sleep 4
timeout 8 ros2 topic echo /chassis/state --once > /tmp/fi2_cleared.txt 2>&1
timeout 8 ros2 topic echo /odom --once > /tmp/fi2_odom_after.txt 2>&1
echo "[cleared] state:" | tee -a $OUT
grep -E "control_mode|motion_allowed|feedback_valid|fault_reason|linear_velocity" /tmp/fi2_cleared.txt | tee -a $OUT
echo "odom latched/after:" | tee -a $OUT
grep -A2 "position:" /tmp/fi2_odom_latched.txt | head -3 | tee -a $OUT
grep -A2 "position:" /tmp/fi2_odom_after.txt | head -3 | tee -a $OUT
echo "--- v4 latch/velocity log ---" | tee -a $OUT
grep -E "TEST INJECTION|safety latch|ROS_VELOCITY" /tmp/v4_fi2.log | tail -8 | tee -a $OUT
timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
pkill -f pub_safe_cmd 2>/dev/null; pkill -f RemoteControl_WheelPID_V2 2>/dev/null; pkill -f sbus_emu 2>/dev/null
echo "=== fi2 done ===" | tee -a $OUT
