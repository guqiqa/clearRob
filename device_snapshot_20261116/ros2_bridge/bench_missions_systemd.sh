#!/bin/bash
# systemd v4 常驻版台架任务：F/B 0.5 m 各 3 次 + 矩形 2.0 m 1 次
set +u
source /root/nav_ws/env.sh
systemctl is-active laifei-chassis-v4.service | grep -q active || { echo "v4 unit not active"; exit 1; }
PARAMS=/root/nav_ws/config/ground_demo_bench_stable.yaml
pkill -f demo_mission_manager 2>/dev/null
pkill -f navigation_hardware_dry_run.launch.py 2>/dev/null
pkill -f navigation_node 2>/dev/null
pkill -f safety_supervisor_node 2>/dev/null
pkill -f flat_ground_grid 2>/dev/null
sleep 1
setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/bm_nav.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 /root/ros2_bridge/flat_ground_grid.py > /tmp/bm_grid.log 2>&1 < /dev/null &
sleep 1
timeout 8 ros2 param set /navigation_node profile GROUND_DEMO_BENCH_STABLE >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1
run_task() {
  TASK=$1; shift
  pkill -f demo_mission_manager 2>/dev/null; sleep 1
  setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file "$PARAMS" -p task:="$TASK" "$@" > /tmp/bm_mission.log 2>&1 < /dev/null &
  sleep 4
  timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  timeout 8 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  sleep 1
  X0=$(timeout 6 ros2 topic echo /odom --once 2>/dev/null | grep -A2 "position:" | grep "x:" | head -1 | awk '{print $2}')
  timeout 8 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1
  FINAL=""; for k in $(seq 1 150); do
    st=$(timeout 5 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
    case "$st" in COMPLETED|ABORTED|STOPPED|TIMEOUT|CHASSIS_FAULT|ESTOP) FINAL=$st; break;; esac
    sleep 1
  done
  sleep 1
  X1=$(timeout 6 ros2 topic echo /odom --once 2>/dev/null | grep -A2 "position:" | grep "x:" | head -1 | awk '{print $2}')
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f demo_mission_manager 2>/dev/null
  echo "[task] $TASK final=$FINAL x0=$X0 x1=$X1"
}
for i in 1 2 3; do run_task FORWARD -p distance_m:=0.5; sleep 1; done
for i in 1 2 3; do run_task BACKWARD -p distance_m:=0.5; sleep 1; done
run_task RECTANGLE_LOOP -p side_m:=2.0 -p corner_radius_m:=0.6 -p waypoint_step_m:=0.4
pkill -f demo_mission_manager 2>/dev/null
pkill -f navigation_hardware_dry_run.launch.py 2>/dev/null
pkill -f navigation_node 2>/dev/null
pkill -f safety_supervisor_node 2>/dev/null
pkill -f flat_ground_grid 2>/dev/null
echo "[missions] done"
