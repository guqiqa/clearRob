#!/bin/bash
# 开环直线测试（绕过导航追踪）：固定 0.12 m/s 前进约 8 s，记录 IMU-odom 航向与四轮转速
set +u
source /root/nav_ws/env.sh
RES=/tmp/openloop_straight.txt
CSV=/tmp/openloop_straight.csv
: > $RES
log() { echo "$*" | tee -a $RES; }
cleanup() {
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f sweep_driver 2>/dev/null
  pkill -f record_state_odom 2>/dev/null
  sleep 1
}
fail() { log "FAIL: $*"; cleanup; exit 1; }
log "=== open-loop straight test $(date +%F_%T) ==="
echo "重启 v4 以清零 IMU 航向（3 s 校准，保持车静止）..." | tee -a $RES
systemctl restart laifei-chassis-v4.service
sleep 9
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "v4 not active"
pkill -f navigation_hardware_dry_run.launch.py 2>/dev/null
pkill -f navigation_node 2>/dev/null
pkill -f safety_supervisor_node 2>/dev/null
pkill -f demo_mission_manager 2>/dev/null
sleep 1
timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
setsid nohup python3 /root/ros2_bridge/record_state_odom.py "$CSV" 20 > /tmp/ol_rec.log 2>&1 < /dev/null &
sleep 1
echo 0.12 > /tmp/sweep_speed
setsid nohup python3 /root/ros2_bridge/sweep_driver.py 12 > /tmp/ol_driver.log 2>&1 < /dev/null &
sleep 8
echo 0 > /tmp/sweep_speed
sleep 2
pkill -f sweep_driver 2>/dev/null
sleep 1
pkill -f record_state_odom 2>/dev/null
cleanup
python3 /root/ros2_bridge/analyze_heading_diag.py "$CSV" >> $RES 2>&1
log "=== done:$RES ==="
