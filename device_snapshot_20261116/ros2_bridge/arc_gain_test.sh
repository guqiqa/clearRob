#!/bin/bash
# 15A 弧线转向增益测试（人工监护；不要用 start_v4_bench.sh）
# 目的：在 15 A 电流上限下，测 v=0.08 时 w=±0.15 / ±0.30 的实际 yaw 响应，
#       与 wheel 派生角速度比较，得到转向效率，供导航参数（速度/半径/电流）决策。
# 注意：每段 6 s、共 4 段，总运动约 24 s；需要约 2x2 m 空地、人持急停。
set +u
source /root/nav_ws/env.sh
BRIDGE=/root/ros2_bridge
RES=/tmp/arc_gain_test.txt
CSV=/tmp/arc_gain_test.csv
: > "$RES"
log() { echo "$*" | tee -a "$RES"; }
cleanup() {
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f record_state_odom 2>/dev/null
  sleep 0.5
}
fail() { log "FAIL: $*"; cleanup; exit 1; }
set_mode() {
  local want=$1 out
  out=$(timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: $want}" 2>&1)
  echo "$out" | grep -Eqi "success.*true" || { log "SERVICE_FAIL set_mode($want)"; return 1; }
  return 0
}
phase() {
  local w=$1 dur=$2 tag=$3 yaw
  set_mode true || fail "enable mode ($tag)"
  sleep 0.4
  python3 "$BRIDGE/arc_cmd.py" 0.08 "$w" "$dur" "$tag" > "/tmp/arc_${tag}.log" 2>&1 || fail "arc_cmd $tag failed"
  sleep 0.7
  yaw=$(python3 "$BRIDGE/print_yaw_once.py" 2>/dev/null | sed -n 's/^yaw_deg=//p')
  [ -n "$yaw" ] || fail "no yaw for $tag"
  echo "$yaw"
}
trap cleanup EXIT

log "=== 15A arc gain test $(date +%F_%T) ==="
log "protocol: v=0.08 m/s; w=+0.15,-0.15,+0.30,-0.30 each 6 s; wheel current cap 1500 (=15A)"
log "需要约 2x2 m 空地、人持急停；总运动约 24 s"

pkill -f navigation_hardware_dry_run.launch.py 2>/dev/null
pkill -f navigation_node 2>/dev/null
pkill -f safety_supervisor_node 2>/dev/null
pkill -f demo_mission_manager 2>/dev/null
pkill -f hardware_adapter 2>/dev/null
pkill -f wheel_speed_node 2>/dev/null
pkill -f flat_ground_grid 2>/dev/null
sleep 1
SAFEP=$(timeout 8 ros2 topic info /safety/safe_motion_command 2>/dev/null | awk '/Publisher count/ {print $3}')
CMDP=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/ {print $3}')
log "precheck: safe_cmd_publishers=$SAFEP cmd_vel_publishers=$CMDP"
[ "$SAFEP" = "0" ] || fail "stray SafeMotionCommand publisher(s)=$SAFEP"
[ "$CMDP" = "0" ] || fail "/cmd_vel publishers=$CMDP"
timeout 8 ros2 service list 2>/dev/null | grep -q /chassis/set_autonomous_mode || fail "mode service missing"
STATE=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
echo "$STATE" | grep -q "emergency_stop: false" || fail "emergency_stop true or no state"
echo "$STATE" | grep -q "feedback_valid: true" || fail "wheel feedback invalid"
echo "$STATE" | grep -q "can_healthy: true" || fail "CAN unhealthy"
log "precheck: estop=false feedback_valid=true can_healthy=true"

setsid nohup python3 "$BRIDGE/record_state_odom.py" "$CSV" 120 > /tmp/arc_gain_rec.log 2>&1 < /dev/null &
sleep 1
set_mode true || fail "enable mode (yaw0)"
sleep 0.4
Y0=$(python3 "$BRIDGE/print_yaw_once.py" 2>/dev/null | sed -n 's/^yaw_deg=//p')
[ -n "$Y0" ] || fail "no yaw0"
log "YAW0=$Y0"

Y1=$(phase +0.15 6.0 g_p015) || fail "phase +0.15 failed"; log "YAW_P015=$Y1"
Y2=$(phase -0.15 6.0 g_m015) || fail "phase -0.15 failed"; log "YAW_M015=$Y2"
Y3=$(phase +0.30 6.0 g_p030) || fail "phase +0.30 failed"; log "YAW_P030=$Y3"
Y4=$(phase -0.30 6.0 g_m030) || fail "phase -0.30 failed"; log "YAW_M030=$Y4"

D1=$(awk -v a="$Y0" -v b="$Y1" 'BEGIN{printf "%.2f", b-a}')
D2=$(awk -v a="$Y1" -v b="$Y2" 'BEGIN{printf "%.2f", b-a}')
D3=$(awk -v a="$Y2" -v b="$Y3" 'BEGIN{printf "%.2f", b-a}')
D4=$(awk -v a="$Y3" -v b="$Y4" 'BEGIN{printf "%.2f", b-a}')
log "DELTA_P015=$D1 DELTA_M015=$D2 DELTA_P030=$D3 DELTA_M030=$D4"
log "（DELTA_POS 为正 = 物理左转/CCW，符合 ROS 约定）"

set_mode false >/dev/null 2>&1
sleep 0.5
pkill -f record_state_odom 2>/dev/null
sleep 0.5
python3 "$BRIDGE/analyze_turn_sign.py" "$CSV" >> "$RES" 2>&1
log "=== done: $RES csv=$CSV ==="
exit 0
