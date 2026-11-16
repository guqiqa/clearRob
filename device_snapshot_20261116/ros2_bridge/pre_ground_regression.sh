#!/bin/bash
# 落地前全量台架回归（systemd v4 常驻；轮子必须离地）
set +u
source /root/nav_ws/env.sh
R=/tmp/pre_ground_regression_20261024.txt
D=/tmp/pre_ground_done.txt
LOG=/tmp/pre_ground_regression.log
rm -f $D; : > $R
log() { echo "$*" | tee -a $R >> $LOG; }
fail() { log "FAIL: $*"; echo "FAIL" > $D; exit 1; }
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "v4 unit not active"
pgrep -f "wheel_speed_node|hardware_adapter" >/dev/null && fail "forbidden CAN node running"
log "=== pre-ground regression $(date +%F_%T) ==="

log "--- A. speed sweep x3 ---"
for rep in 1 2 3; do
  timeout 300 bash /root/ros2_bridge/speed_sweep_systemd.sh "preA_rep${rep}" 2>&1 | tail -2 | tee -a $R
done

log "--- B. bench missions (F/B x3 + rectangle) ---"
timeout 600 bash /root/ros2_bridge/bench_missions_systemd.sh 2>&1 | grep -E "\[task\]|\[missions\]" | tee -a $R

log "--- C. stability (idle 5 min + 10 cycles) ---"
timeout 900 bash /root/ros2_bridge/stability_systemd.sh 2>&1 | tail -20 | tee -a $R

log "--- D. feedback injection ---"
systemctl stop laifei-chassis-v4.service
systemctl stop laifei-sbus-emu.service
sleep 2
timeout 150 bash /root/ros2_bridge/feedback_inject_test2.sh > /tmp/inject_out.log 2>&1
grep -E "latched|cleared|CHASSIS_FEEDBACK_LOST|SAFE_COMMAND_TIMEOUT|PASS|FAIL" /tmp/inject_out.log | tail -12 | tee -a $R
systemctl start laifei-sbus-emu.service
sleep 2
systemctl start laifei-chassis-v4.service
sleep 7
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "v4 unit failed to restart after injection"
ST=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
echo "$ST" | grep -q "control_mode: MANUAL" || fail "post-injection state not MANUAL"
log "[post] v4 active MANUAL"

log "=== PRE_GROUND_REGRESSION_DONE $(date +%F_%T) ==="
echo "PASS" > $D
