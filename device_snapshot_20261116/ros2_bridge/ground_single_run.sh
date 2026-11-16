#!/bin/bash
# 单次地面运动 + 卷尺标定（用法: ground_single_run.sh BACKWARD 1.0）
set +u
source /root/nav_ws/env.sh
TASK=${1:-FORWARD}
DIST=${2:-1.0}
RES=/tmp/ground_single_result.txt
: > $RES
log() { echo "$*" | tee -a $RES; }
cleanup() {
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f demo_mission_manager 2>/dev/null
  pkill -f navigation_hardware_dry_run.launch.py 2>/dev/null
  pkill -f navigation_node 2>/dev/null
  pkill -f safety_supervisor_node 2>/dev/null
  pkill -f flat_ground_grid 2>/dev/null
  sleep 1
}
fail() { log "FAIL: $*"; cleanup; exit 1; }
pose() {
  O=$(timeout 6 ros2 topic echo /odom --once 2>/dev/null)
  X=$(echo "$O" | grep -A2 "position:" | grep "x:" | head -1 | awk '{print $2}')
  Y=$(echo "$O" | grep -A3 "position:" | grep "y:" | head -1 | awk '{print $2}')
  OZ=$(echo "$O" | grep -A4 "orientation:" | grep "z:" | head -1 | awk '{print $2}')
  OW=$(echo "$O" | grep -A5 "orientation:" | grep "w:" | head -1 | awk '{print $2}')
  YAW=$(python3 -c "import math; print(round(math.degrees(2*math.atan2(float(\"$OZ\"),float(\"$OW\"))),2))" 2>/dev/null)
  echo "$X $Y $YAW"
}

log "=== ground single run: $TASK $DIST m  $(date +%F_%T) ==="
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "chassis unit not active"
pgrep -f "wheel_speed_node|hardware_adapter" >/dev/null && fail "forbidden CAN node running"
ST=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
echo "$ST" | grep -q "control_mode: MANUAL" || fail "chassis not MANUAL"
CMDP=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/ {print $3}')
[ "$CMDP" = "0" ] || fail "/cmd_vel publishers=$CMDP"
echo "准备：车${TASK}方向留足 ${DIST} m + 1 m 余量；地面标记起点；一人持急停。"
read -r -p "确认安全，输入 GO 开始（其他键中止）: " ANS
[ "$ANS" = "GO" ] || { echo "已中止，未运动"; exit 0; }

setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/gs_nav.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 /root/ros2_bridge/flat_ground_grid.py > /tmp/gs_grid.log 2>&1 < /dev/null &
sleep 1
timeout 8 ros2 param set /navigation_node profile GROUND_DEMO_BENCH_STABLE >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1

setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file /root/nav_ws/config/ground_demo_bench_stable.yaml -p task:=$TASK -p distance_m:=$DIST > /tmp/gs_mission.log 2>&1 < /dev/null &
sleep 4
timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
timeout 8 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
sleep 1
read X0 Y0 YAW0 <<< "$(pose)"
log "start x=$X0 y=$Y0 yaw=$YAW0"
timeout 8 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1
FINAL=""; for k in $(seq 1 120); do
  st=$(timeout 5 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
  case "$st" in COMPLETED|ABORTED|STOPPED|TIMEOUT|CHASSIS_FAULT|ESTOP) FINAL=$st; break;; esac
  sleep 1
done
sleep 1
read X1 Y1 YAW1 <<< "$(pose)"
timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
pkill -f demo_mission_manager 2>/dev/null
cleanup
ODOM=$(python3 -c "import math; print(round(math.hypot(float(\"$X1\")-float(\"$X0\"), float(\"$Y1\")-float(\"$Y0\")),4))" 2>/dev/null)
log "mission=$FINAL odom_displacement=$ODOM m  start=($X0,$Y0,$YAW0) end=($X1,$Y1,$YAW1)"

echo "请用卷尺量：起点标记 -> 机器人最终位置（沿运动方向）的实际距离（米）"
read -r -p "实测物理距离(m): " PHYS
SCALE=$(awk "BEGIN{printf \"%.4f\", ($PHYS)/($ODOM)}" 2>/dev/null)
log "physical=$PHYS m odom=$ODOM m physical/odom=$SCALE"
echo "参考:" | tee -a $RES
echo "  6寸/减速比5.0 理论: odom/物理约1.26 => odom=$DIST m 时物理应约 $(awk "BEGIN{printf \"%.3f\", $DIST/1.262}") m" | tee -a $RES
echo "  当前配置(0.10m/5.2)理论: odom=物理 => 物理应约 $DIST m" | tee -a $RES
echo "  实测/$ODOM=$SCALE" | tee -a $RES
log "结果文件: /tmp/ground_single_result.txt"
