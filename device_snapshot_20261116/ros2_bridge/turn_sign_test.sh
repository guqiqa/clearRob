#!/bin/bash
# 转向符号测试（turning-sign test）
# 目的：验证 ROS 角速度 w 的符号与 /odom yaw 方向是否符合 ROS 约定：
#   w > 0（CCW/左转）=> /odom yaw 增大；w < 0 => yaw 减小。
# 方法：固定 v=+0.08 m/s，依次下发
#   (a) w=0    1.5 s（前向基线，确认 v 与轮速符号）
#   (b) w=+0.15 3 s
#   (c) w=-0.15 3 s
# 记录 /odom + /chassis/state；用 print_yaw_once.py 采样 yaw0/yaw_plus/yaw_minus。
# 安全：v<=0.08 m/s、|w|<=0.15 rad/s，总位移 <0.7 m；测试期间必须有人持急停。
# 仅允许 systemd 版 v4；不得调用 start_v4_bench.sh（会 pkill 并自启 v4）。
set +u
source /root/nav_ws/env.sh
BRIDGE=/root/ros2_bridge
RES=/tmp/turn_sign_test.txt
CSV=/tmp/turn_sign_test.csv
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
trap cleanup EXIT

log "=== turn sign test $(date +%F_%T) ==="
log "protocol: v=+0.08; forward 1.5s, w=+0.15 3s, w=-0.15 3s; /odom yaw by ROS convention"

# 0) 前置：确认只有 v4 底盘进程（禁止 v3 / wheel_speed_node / hardware_adapter）
CHASSIS_PROCS=$(pgrep -af 'RemoteControl_WheelPID_V2' | tr '\n' ';')
NPROC=$(pgrep -fc 'RemoteControl_WheelPID_V2')
log "chassis_procs(before restart)=$CHASSIS_PROCS"
[ "$NPROC" = "1" ] || fail "expected exactly 1 chassis process, got $NPROC: $CHASSIS_PROCS"

# 1) 重启 v4：IMU 静止校准 3 s（sleep 9 留余量），odom 归零
systemctl restart laifei-chassis-v4.service
sleep 9
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "v4 unit not active"
log "v4 restarted and active"

# 2) 停 nav/safety，保证 /safety/safe_motion_command 只有本测试发布
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
[ "$CMDP" = "0" ] || fail "/cmd_vel publishers=$CMDP (must be 0)"
timeout 8 ros2 service list 2>/dev/null | grep -q /chassis/set_autonomous_mode || fail "mode service missing"

# 3) 底盘预检：急停/反馈/CAN
STATE=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
echo "$STATE" | grep -q "emergency_stop: false" || fail "emergency_stop true or no /chassis/state"
echo "$STATE" | grep -q "feedback_valid: true" || fail "wheel feedback invalid"
echo "$STATE" | grep -q "can_healthy: true" || fail "CAN unhealthy"
log "precheck: estop=false feedback_valid=true can_healthy=true"

# 4) 连续记录 /chassis/state + /odom
setsid nohup python3 "$BRIDGE/record_state_odom.py" "$CSV" 60 > /tmp/turn_sign_rec.log 2>&1 < /dev/null &
sleep 1

# 5) yaw0；每次 arc 前重开自主模式，清除 SAFE_COMMAND_TIMEOUT 锁存
set_mode true || fail "enable autonomous mode (yaw0)"
sleep 0.5
YAW0=$(python3 "$BRIDGE/print_yaw_once.py" 2>/dev/null | sed -n 's/^yaw_deg=//p')
[ -n "$YAW0" ] || fail "no /odom yaw for yaw0"
log "YAW0=$YAW0"

# 6a) 前向基线：v=+0.08, w=0, 1.5 s
set_mode true || fail "enable autonomous mode (forward)"
sleep 0.3
python3 "$BRIDGE/arc_cmd.py" 0.08 0.0 1.5 turn_sign_forward > /tmp/turn_sign_forward.log 2>&1 || fail "forward jog failed"
sleep 0.5
YAW_FWD=$(python3 "$BRIDGE/print_yaw_once.py" 2>/dev/null | sed -n 's/^yaw_deg=//p')
log "YAW_FORWARD=$YAW_FWD"

# 6b) w=+0.15：ROS 约定应使 yaw 增大
set_mode true || fail "enable autonomous mode (+w)"
sleep 0.3
python3 "$BRIDGE/arc_cmd.py" 0.08 +0.15 3.0 turn_sign_plus > /tmp/turn_sign_plus.log 2>&1 || fail "arc +w failed"
sleep 0.5
YAW_PLUS=$(python3 "$BRIDGE/print_yaw_once.py" 2>/dev/null | sed -n 's/^yaw_deg=//p')
[ -n "$YAW_PLUS" ] || fail "no /odom yaw for yaw_plus"
log "YAW_PLUS=$YAW_PLUS"

# 6c) w=-0.15：应使 yaw 减小
set_mode true || fail "enable autonomous mode (-w)"
sleep 0.3
python3 "$BRIDGE/arc_cmd.py" 0.08 -0.15 3.0 turn_sign_minus > /tmp/turn_sign_minus.log 2>&1 || fail "arc -w failed"
sleep 0.5
YAW_MINUS=$(python3 "$BRIDGE/print_yaw_once.py" 2>/dev/null | sed -n 's/^yaw_deg=//p')
[ -n "$YAW_MINUS" ] || fail "no /odom yaw for yaw_minus"
log "YAW_MINUS=$YAW_MINUS"

# 7) 恢复 MANUAL，停记录器
set_mode false >/dev/null 2>&1
sleep 0.5
pkill -f record_state_odom 2>/dev/null
sleep 0.5

# 8) 数值判读
D_PLUS=$(awk -v a="$YAW0" -v b="$YAW_PLUS" 'BEGIN{printf "%.2f", b-a}')
D_MINUS=$(awk -v a="$YAW0" -v b="$YAW_MINUS" 'BEGIN{printf "%.2f", b-a}')
log "D_PLUS=$D_PLUS D_MINUS=$D_MINUS"
python3 "$BRIDGE/analyze_turn_sign.py" "$CSV" >> "$RES" 2>&1
ABS_PLUS=$(awk -v d="$D_PLUS" 'BEGIN{printf "%.2f", (d<0?-d:d)}')
if awk -v d="$ABS_PLUS" 'BEGIN{exit !(d < 5.0)}'; then
  log "VERDICT=INCONCLUSIVE_WEAK_YAW (|D_PLUS|=$ABS_PLUS deg < 5 deg): 车身没有实际转动（轮子离地/打滑或 yaw 未跟随），本次不能判定 w 符号；请确认轮子着地后重跑"
elif awk -v d="$D_PLUS" 'BEGIN{exit !(d > 5.0)}'; then
  log "VERDICT=SIGN_OK (w=+0.15 raised /odom yaw by $D_PLUS deg; ROS convention satisfied)"
else
  log "VERDICT=SIGN_REVERSED (w=+0.15 changed /odom yaw by $D_PLUS deg; expected > +5)"
fi
log "=== done: $RES csv=$CSV ==="
exit 0
