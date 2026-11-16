#!/bin/bash
# 单次地面标定运动（非交互）：仅执行一次指定任务，打印 odom 前后位姿；物理距离由人工卷尺测量
# 用法: ground_calib_move.sh BACKWARD 1.0
set +u
source /root/nav_ws/env.sh
TASK=${1:-BACKWARD}
DIST=${2:-1.0}
RES=/tmp/ground_calib_move_${TASK}_${DIST}.txt
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
log "=== ground calib move $TASK $DIST m $(date +%F_%T) ==="
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "v4 unit not active"
pgrep -f "wheel_speed_node|hardware_adapter" >/dev/null && fail "forbidden CAN node running"
CMDP=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/ {print $3}')
[ "$CMDP" = "0" ] || fail "/cmd_vel publishers=$CMDP"
setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/gcm_nav.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 /root/ros2_bridge/flat_ground_grid.py > /tmp/gcm_grid.log 2>&1 < /dev/null &
sleep 1
timeout 8 ros2 param set /navigation_node profile GROUND_DEMO_BENCH_STABLE >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1
setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file /root/nav_ws/config/ground_demo_bench_stable.yaml -p task:=$TASK -p distance_m:=$DIST > /tmp/gcm_mission.log 2>&1 < /dev/null &
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
ODOM=$(python3 -c "import math; print(round(math.hypot(float(\"$X1\")-float(\"$X0\"), float(\"$Y1\")-float(\"$Y0\")),4))" 2>/dev/null)
log "final=$FINAL end x=$X1 y=$Y1 yaw=$YAW1 odom_displacement=$ODOM"
cleanup
log "state: $(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null | grep control_mode)"
log "=== done:$RES ==="
