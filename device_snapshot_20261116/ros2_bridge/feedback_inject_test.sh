#!/bin/bash
# 独立反馈丢失测试：保持 CAN，强制 feedback 无效；验证锁存与清除
set +u
source /root/nav_ws/env.sh
BRIDGE=/root/ros2_bridge
BASE=/root/chassis_ros2_nav_v4
OUT=/tmp/feedback_inject_result.txt
: > $OUT
pkill -f RemoteControl_WheelPID_V2 2>/dev/null
pkill -f sbus_emu 2>/dev/null
pkill -f sweep_driver 2>/dev/null
sleep 1
ip link set can0 down 2>/dev/null
ip link set can0 type can bitrate 500000 2>/dev/null
ip link set can0 up 2>/dev/null
rm -f /tmp/sbus_emu_path
setsid nohup python3 "$BRIDGE/sbus_emu.py" > /tmp/fi_sbus.log 2>&1 < /dev/null &
sleep 2
PTS=$(cat /tmp/sbus_emu_path 2>/dev/null)
sed "s#<SBus_Port>[^<]*</SBus_Port>#<SBus_Port>$PTS</SBus_Port>#" "$BASE/src/config.xml" > "$BASE/src/config_emu.xml"
setsid nohup "$BASE/build/RemoteControl_WheelPID_V2" "$BASE/src/config_emu.xml" --inject-feedback-loss-after-s 8 --inject-feedback-loss-duration-s 4 > /tmp/v4_fi.log 2>&1 < /dev/null &
T0=$(date +%s)
sleep 3
timeout 10 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
echo "0.10" > /tmp/sweep_speed
setsid nohup python3 "$BRIDGE/sweep_driver.py" 24 > /tmp/fi_driver.log 2>&1 < /dev/null &
sleep 6
timeout 8 ros2 topic echo /chassis/state --once > /tmp/fi_state_latched.txt 2>&1
timeout 8 ros2 topic echo /odom --once > /tmp/fi_odom_latched.txt 2>&1
echo "[t+9s] latched state:" | tee -a $OUT
grep -E "control_mode|motion_allowed|feedback_valid|fault_reason|command_age|linear_velocity" /tmp/fi_state_latched.txt | tee -a $OUT
sleep 6
echo "[t+15s] injection ended" | tee -a $OUT
timeout 8 ros2 topic echo /chassis/state --once > /tmp/fi_state_after.txt 2>&1
grep -E "control_mode|feedback_valid|fault_reason" /tmp/fi_state_after.txt | tee -a $OUT
echo "--- clear latch: mode off -> on (driver still publishing) ---" | tee -a $OUT
timeout 8 ros2 topic echo /odom --once > /tmp/fi_odom_before_clear.txt 2>&1
timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
sleep 1
timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
sleep 4
timeout 8 ros2 topic echo /chassis/state --once > /tmp/fi_state_cleared.txt 2>&1
grep -E "control_mode|motion_allowed|feedback_valid|fault_reason|linear_velocity" /tmp/fi_state_cleared.txt | tee -a $OUT
timeout 8 ros2 topic echo /odom --once > /tmp/fi_odom_recovered.txt 2>&1
echo "odom before/after clear:" | tee -a $OUT
grep -A2 "position:" /tmp/fi_odom_before_clear.txt | head -3 | tee -a $OUT
grep -A2 "position:" /tmp/fi_odom_recovered.txt | head -3 | tee -a $OUT
echo "--- v4 injection/latch log ---" | tee -a $OUT
grep -E "TEST INJECTION|safety latch|SAFETY_STATE" /tmp/v4_fi.log | tail -8 | tee -a $OUT
echo "--- v4 ROS_VELOCITY after recovery ---" | tee -a $OUT
grep "ROS_VELOCITY" /tmp/v4_fi.log | tail -3 | tee -a $OUT
timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
pkill -f sweep_driver 2>/dev/null
pkill -f RemoteControl_WheelPID_V2 2>/dev/null
pkill -f sbus_emu 2>/dev/null
echo "=== feedback inject test done ===" | tee -a $OUT
