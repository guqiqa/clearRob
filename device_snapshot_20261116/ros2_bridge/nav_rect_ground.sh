#!/bin/bash
# 导航矩形地面测试：一次 RECTANGLE_LOOP（需约 side+0.8 m 的方形空间），记录 CSV 与闭合误差。
# 用法: bash nav_rect_ground.sh [SIDE_M] [CORNER_RADIUS_M] [SPEED_MPS]
#   默认 2.0 0.8 0.10
set +u
source /root/nav_ws/env.sh
BRIDGE=/root/ros2_bridge
SIDE=${1:-2.0}
RADIUS=${2:-0.8}
SPEED=${3:-0.10}
TAG=rect_${SIDE}_${RADIUS}_${SPEED}
RES=/tmp/nav_rect_${TAG}.txt
CSV=/tmp/nav_rect_${TAG}.csv
: > "$RES"
log() { echo "$*" | tee -a "$RES"; }
cleanup() {
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f record_state_odom 2>/dev/null
  pkill -f demo_mission_manager 2>/dev/null
  pkill -f navigation_hardware_dry_run.launch.py 2>/dev/null
  pkill -f navigation_node 2>/dev/null
  pkill -f safety_supervisor_node 2>/dev/null
  pkill -f flat_ground_grid 2>/dev/null
  sleep 1
}
fail() { log "FAIL: $*"; cleanup; exit 1; }

log "=== nav rectangle $(date +%F_%T) side=$SIDE radius=$RADIUS speed=$SPEED ==="
log "需要约 side+0.8 m 的方形空间（含车体），人持急停；预计 60~100 s"
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "v4 not active"
CMDP=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/ {print $3}')
[ "$CMDP" = "0" ] || fail "/cmd_vel publishers=$CMDP"

setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/nav_rect_launch.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 "$BRIDGE/flat_ground_grid.py" > /tmp/nav_rect_grid.log 2>&1 < /dev/null &
sleep 1
timeout 8 ros2 param set /navigation_node profile GROUND_DEMO_BENCH_STABLE >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_linear_x_mps "$SPEED" >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_angular_z_radps 0.50 >/dev/null 2>&1

setsid nohup bash "$BRIDGE/run_demo_mission.sh" --ros-args --params-file /root/nav_ws/config/ground_demo_bench_stable.yaml -p task:=RECTANGLE_LOOP -p side_m:="$SIDE" -p corner_radius_m:="$RADIUS" -p waypoint_step_m:=0.4 > /tmp/nav_rect_mission.log 2>&1 < /dev/null &
sleep 4
timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
timeout 8 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
sleep 1
setsid nohup python3 "$BRIDGE/record_state_odom.py" "$CSV" 240 > /tmp/nav_rect_rec.log 2>&1 < /dev/null &
sleep 1
timeout 8 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1

FINAL=""
for k in $(seq 1 200); do
  st=$(timeout 5 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
  case "$st" in COMPLETED|ABORTED|STOPPED|TIMEOUT|CHASSIS_FAULT|ESTOP) FINAL=$st; break;; esac
  sleep 1
done
sleep 2
pkill -f record_state_odom 2>/dev/null
cleanup
log "final=$FINAL csv=$CSV"
python3 - "$CSV" >> "$RES" 2>&1 <<'PY'
import csv, sys, math
rows = list(csv.DictReader(open(sys.argv[1])))
if len(rows) < 20:
    print("NO_DATA"); raise SystemExit(0)
xs = [float(r["x"]) for r in rows]; ys = [float(r["y"]) for r in rows]; yaws = [float(r["yaw_deg"]) for r in rows]
x0, y0, yaw0 = xs[0], ys[0], yaws[0]; x1, y1, yaw1 = xs[-1], ys[-1], yaws[-1]
print("odom_start=(%.3f,%.3f,%.2f) end=(%.3f,%.3f,%.2f)" % (x0, y0, yaw0, x1, y1, yaw1))
print("closure=%.3f m yaw_err=%.2f deg" % (math.hypot(x1 - x0, y1 - y0), ((yaw1 - yaw0 + 180.0) % 360.0) - 180.0))
PY
python3 "$BRIDGE/analyze_turn_sign.py" "$CSV" >> "$RES" 2>&1
log "=== done: $RES ==="
