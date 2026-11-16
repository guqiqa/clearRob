#!/bin/bash
# systemd v4 常驻版短稳定性：idle 5 min + 10 次 F/B 往复（轮子离地）
set +u
source /root/nav_ws/env.sh
R=/tmp/stability_systemd_result.txt
: > $R
log() { echo "$*" | tee -a $R; }
systemctl is-active laifei-chassis-v4.service | grep -q active || { echo "v4 unit not active"; exit 1; }
V4PID=$(pgrep -f "RemoteControl_WheelPID_V2 .*config_emu.xml" | head -1)
log "v4_pid=$V4PID"
for i in $(seq 1 5); do
  sleep 60
  RSS=$(ps -o rss= -p "$V4PID" 2>/dev/null | tr -d " ")
  ALIVE=$(kill -0 "$V4PID" 2>/dev/null && echo yes || echo no)
  ST=$(timeout 6 ros2 topic echo /chassis/state --once 2>/dev/null)
  FB=$(echo "$ST" | grep "feedback_valid:" | awk '{print $2}')
  CANH=$(echo "$ST" | grep "can_healthy:" | awk '{print $2}')
  log "[idle $i] alive=$ALIVE rss_kb=$RSS fb=$FB can=$CANH"
done
log "[idle] done"
PARAMS=/root/nav_ws/config/ground_demo_bench_stable.yaml
setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/st_nav.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 /root/ros2_bridge/flat_ground_grid.py > /tmp/st_grid.log 2>&1 < /dev/null &
sleep 1
timeout 8 ros2 param set /navigation_node profile GROUND_DEMO_BENCH_STABLE >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1
for i in $(seq 1 10); do
  if [ $((i % 2)) -eq 1 ]; then TASK=FORWARD; else TASK=BACKWARD; fi
  pkill -f demo_mission_manager 2>/dev/null; sleep 1
  setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file "$PARAMS" -p task:=$TASK -p distance_m:=0.5 > /tmp/st_mission.log 2>&1 < /dev/null &
  sleep 4
  timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  timeout 8 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  sleep 1
  timeout 8 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1
  FINAL=""; for k in $(seq 1 120); do
    st=$(timeout 5 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
    case "$st" in COMPLETED|ABORTED|STOPPED|TIMEOUT|CHASSIS_FAULT|ESTOP) FINAL=$st; break;; esac
    sleep 1
  done
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f demo_mission_manager 2>/dev/null
  RSS=$(ps -o rss= -p "$V4PID" 2>/dev/null | tr -d " ")
  log "[cycle $i] task=$TASK final=$FINAL rss_kb=$RSS"
  sleep 1
done
pkill -f navigation_hardware_dry_run.launch.py 2>/dev/null
pkill -f navigation_node 2>/dev/null
pkill -f safety_supervisor_node 2>/dev/null
pkill -f flat_ground_grid 2>/dev/null
log "[stability] done"
