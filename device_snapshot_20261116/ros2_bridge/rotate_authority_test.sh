#!/bin/bash
# 原地转向能力测试：v≈0.0005 触发 v4 的 rotate 分支（Rotate_Max_Current=1500，即 15A）。
# w=+0.30 / -0.30 各 4 s；若 |dyaw|<8 deg，自动升级到 w=±0.60 各 3 s。
# 安全：原地旋转、速度慢；测试期间必须有人持急停、场地清空。
set +u
source /root/nav_ws/env.sh
BRIDGE=/root/ros2_bridge
RES=/tmp/rotate_authority_test.txt
CSV=/tmp/rotate_authority_test.csv
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
  if ! echo "$out" | grep -Eqi "success.*true"; then
    log "SERVICE_FAIL set_mode($want): $(echo "$out" | tail -2 | tr '\n' ' ')"
    return 1
  fi
  return 0
}
rotate_phase() {
  local w=$1 dur=$2 tag=$3
  set_mode true || fail "enable mode ($tag)"
  sleep 0.3
  python3 "$BRIDGE/arc_cmd.py" 0.0005 "$w" "$dur" "$tag" > "/tmp/rotate_${tag}.log" 2>&1 || fail "arc_cmd $tag failed"
  sleep 0.5
  python3 "$BRIDGE/print_yaw_once.py" 2>/dev/null | sed -n 's/^yaw_deg=//p'
}
trap cleanup EXIT

log "=== rotate authority test $(date +%F_%T) ==="
log "protocol: v=0.0005 (rotate branch, Rotate_Max_Current=1500=15A); w=+0.30/-0.30 4s; escalate to +/-0.60 only if |dyaw|<8deg"

# 0) 重启 v4（IMU 3s 静止校准、yaw 归零）
systemctl restart laifei-chassis-v4.service
sleep 9
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "v4 not active"
log "v4 restarted and active"

# 1) 停 nav/safety，确保只有本测试发布 SafeMotionCommand
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

# 2) 连续记录 state+odom
setsid nohup python3 "$BRIDGE/record_state_odom.py" "$CSV" 90 > /tmp/rotate_rec.log 2>&1 < /dev/null &
sleep 1

set_mode true || fail "enable mode (yaw0)"
sleep 0.5
YAW0=$(python3 "$BRIDGE/print_yaw_once.py" 2>/dev/null | sed -n 's/^yaw_deg=//p')
[ -n "$YAW0" ] || fail "no /odom yaw for yaw0"
log "YAW0=$YAW0"

Y30P=$(rotate_phase +0.30 4.0 rot_p30); log "YAW_P30=$Y30P"
Y30M=$(rotate_phase -0.30 4.0 rot_m30); log "YAW_M30=$Y30M"
D30P=$(awk -v a="$YAW0" -v b="$Y30P" 'BEGIN{printf "%.2f", b-a}')
D30M=$(awk -v a="$Y30P" -v b="$Y30M" 'BEGIN{printf "%.2f", b-a}')
log "D30_PLUS=$D30P D30_MINUS=$D30M"
ABS30=$(awk -v d="$D30P" 'BEGIN{printf "%.2f", (d<0?-d:d)}')

if awk -v a="$ABS30" 'BEGIN{exit !(a < 8.0)}'; then
  log "w=0.30 原地转动不足 (|D30_PLUS|=$ABS30 deg < 8)，升级到 w=+/-0.60"
  Y60P=$(rotate_phase +0.60 3.0 rot_p60); log "YAW_P60=$Y60P"
  Y60M=$(rotate_phase -0.60 3.0 rot_m60); log "YAW_M60=$Y60M"
  D60P=$(awk -v a="$Y30M" -v b="$Y60P" 'BEGIN{printf "%.2f", b-a}')
  D60M=$(awk -v a="$Y60P" -v b="$Y60M" 'BEGIN{printf "%.2f", b-a}')
  log "D60_PLUS=$D60P D60_MINUS=$D60M"
  ABS60=$(awk -v d="$D60P" 'BEGIN{printf "%.2f", (d<0?-d:d)}')
  if awk -v a="$ABS60" 'BEGIN{exit !(a >= 8.0)}'; then
    log "VERDICT=INPLACE_ROTATION_OK_AT_W060 (15A 下 w=0.60 可原地转，|D60_PLUS|=$ABS60 deg)"
  else
    log "VERDICT=INPLACE_ROTATION_FAILED (15A 下 w=0.60 仍转不动，|D60_PLUS|=$ABS60 deg)"
  fi
else
  if awk -v d="$D30P" 'BEGIN{exit !(d > 0)}'; then
    log "VERDICT=INPLACE_ROTATION_OK (15A 下 w=+0.30 使 yaw 增大 $D30P deg，方向符合 ROS 约定)"
  else
    log "VERDICT=INPLACE_ROTATION_REVERSED (15A 下 w=+0.30 使 yaw 变化 $D30P deg，方向与 ROS 约定相反)"
  fi
fi

set_mode false >/dev/null 2>&1
sleep 0.5
pkill -f record_state_odom 2>/dev/null
sleep 0.5
python3 "$BRIDGE/analyze_rotate_test.py" "$CSV" >> "$RES" 2>&1
log "=== done: $RES csv=$CSV ==="
exit 0
