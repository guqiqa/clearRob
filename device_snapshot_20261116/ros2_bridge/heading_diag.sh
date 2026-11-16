#!/bin/bash
# 直线/航向诊断：跑一次任务并记录四轮 rpm + odom，输出航向漂移与左右轮差异
set +u
source /root/nav_ws/env.sh
TASK=${1:-BACKWARD}
DIST=${2:-1.0}
TAG=${TASK}_${DIST}
RES=/tmp/heading_diag_${TAG}.txt
CSV=/tmp/heading_diag_${TAG}.csv
: > $RES
log() { echo "$*" | tee -a $RES; }
cleanup() {
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f demo_mission_manager 2>/dev/null
  pkill -f record_state_odom 2>/dev/null
  pkill -f navigation_hardware_dry_run.launch.py 2>/dev/null
  pkill -f navigation_node 2>/dev/null
  pkill -f safety_supervisor_node 2>/dev/null
  pkill -f flat_ground_grid 2>/dev/null
  sleep 1
}
fail() { log "FAIL: $*"; cleanup; exit 1; }
log "=== heading diag $TASK $DIST m $(date +%F_%T) ==="
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "v4 unit not active"
CMDP=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/ {print $3}')
[ "$CMDP" = "0" ] || fail "/cmd_vel publishers=$CMDP"
setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/hd_nav.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 /root/ros2_bridge/flat_ground_grid.py > /tmp/hd_grid.log 2>&1 < /dev/null &
sleep 1
timeout 8 ros2 param set /navigation_node profile GROUND_DEMO_BENCH_STABLE >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_angular_z_radps 0.50 >/dev/null 2>&1
setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file /root/nav_ws/config/ground_demo_bench_stable.yaml -p task:=$TASK -p distance_m:=$DIST > /tmp/hd_mission.log 2>&1 < /dev/null &
sleep 4
timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
timeout 8 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
sleep 1
setsid nohup python3 /root/ros2_bridge/record_state_odom.py "$CSV" 120 > /tmp/hd_rec.log 2>&1 < /dev/null &
sleep 1
timeout 8 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1
FINAL=""; for k in $(seq 1 120); do
  st=$(timeout 5 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
  case "$st" in COMPLETED|ABORTED|STOPPED|TIMEOUT|CHASSIS_FAULT|ESTOP) FINAL=$st; break;; esac
  sleep 1
done
sleep 2
pkill -f record_state_odom 2>/dev/null
cleanup
log "final=$FINAL csv=$CSV"
python3 /root/ros2_bridge/analyze_heading_diag.py "$CSV" >> $RES 2>&1
log "=== done:$RES ==="
