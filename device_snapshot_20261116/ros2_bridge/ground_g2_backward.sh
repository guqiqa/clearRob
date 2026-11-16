#!/bin/bash
# G2 地面测试：直线后退 0.3 m ×3（0.12 m/s，轮子着地，交互确认后执行）
set +u
source /root/nav_ws/env.sh
RES=/tmp/g2_backward_result.txt
LOG=/tmp/g2_backward.log
: > $RES; : > $LOG
log() { echo "$*" | tee -a $RES >> $LOG; }
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

log "=== G2 backward ground test $(date +%F_%T) ==="
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "chassis unit not active"
pgrep -f "wheel_speed_node|hardware_adapter" >/dev/null && fail "forbidden CAN node running"
ST=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
echo "$ST" | grep -q "control_mode: MANUAL" || fail "chassis not MANUAL before test"
CMDP=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/ {print $3}')
[ "$CMDP" = "0" ] || fail "/cmd_vel publishers=$CMDP (expect 0)"
echo "前检查 OK: MANUAL / 零速 / cmd_vel 0 发布者"
echo "安全确认：车后方已清空、人员持急停、后退方向无人。"
read -r -p "输入 GO 开始 G2（其他任意键中止）: " ANS
[ "$ANS" = "GO" ] || { echo "已中止，未执行运动"; exit 0; }

setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/g2_nav.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 /root/ros2_bridge/flat_ground_grid.py > /tmp/g2_grid.log 2>&1 < /dev/null &
sleep 1
timeout 8 ros2 param set /navigation_node profile GROUND_DEMO_BENCH_STABLE >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1

for i in 1 2 3; do
  log "--- run $i/3: BACKWARD 0.3 m @0.12 ---"
  pkill -f demo_mission_manager 2>/dev/null; sleep 1
  setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file /root/nav_ws/config/ground_demo_bench_stable.yaml -p task:=BACKWARD -p distance_m:=0.3 > /tmp/g2_mission.log 2>&1 < /dev/null &
  sleep 4
  timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  timeout 8 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  sleep 1
  read X0 Y0 YAW0 <<< "$(pose)"; log "start x=$X0 y=$Y0 yaw=$YAW0"
  timeout 8 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1
  FINAL=""; for k in $(seq 1 90); do
    st=$(timeout 5 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
    case "$st" in COMPLETED|ABORTED|STOPPED|TIMEOUT|CHASSIS_FAULT|ESTOP) FINAL=$st; break;; esac
    sleep 1
  done
  sleep 1
  read X1 Y1 YAW1 <<< "$(pose)"
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f demo_mission_manager 2>/dev/null
  DIST=$(python3 -c "import math; print(round(math.hypot(float(\"$X1\")-float(\"$X0\"), float(\"$Y1\")-float(\"$Y0\")),4))" 2>/dev/null)
  log "run=$i final=$FINAL x0=$X0 y0=$Y0 yaw0=$YAW0 -> x1=$X1 y1=$Y1 yaw1=$YAW1 distance=$DIST"
  [ "$FINAL" = "COMPLETED" ] || fail "run $i final=$FINAL"
  sleep 2
done

cleanup
ST=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
echo "$ST" | grep -q "control_mode: MANUAL" || fail "final state not MANUAL"
log "=== G2 RESULT: PASS ==="
log "结果文件: /tmp/g2_backward_result.txt"
