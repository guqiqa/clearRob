#!/bin/bash
set +u
source /root/nav_ws/env.sh
TASK=$1; shift
WATCH=${WATCH:-180}
PARAMS=${PARAMS:-/root/nav_ws/config/ground_demo_bench_stable.yaml}
PROFILE=${PROFILE:-GROUND_DEMO_BENCH_STABLE}
SAFE_LIN=${SAFE_LIN:-0.12}
SAFE_ANG=${SAFE_ANG:-0.20}
pkill -f demo_mission_manager 2>/dev/null
pkill -f record_trace 2>/dev/null
bash /root/ros2_bridge/stop_nav.sh >/dev/null 2>&1
sleep 1
echo "=== v4 bench run: task=$TASK profile=$PROFILE ==="
timeout 120 bash /root/ros2_bridge/start_v4_bench.sh 2>&1 | tail -8
sleep 1
timeout 10 ros2 param set /navigation_node profile $PROFILE 2>&1 | tail -1
timeout 10 ros2 param set /safety_supervisor max_linear_x_mps $SAFE_LIN >/dev/null 2>&1
timeout 10 ros2 param set /safety_supervisor max_angular_z_radps $SAFE_ANG >/dev/null 2>&1
setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file $PARAMS -p task:=$TASK "$@" > /tmp/bench_mission.log 2>&1 < /dev/null &
sleep 5
echo "[mode] enabling chassis + safety autonomous mode"
timeout 10 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" 2>&1 | tail -1
timeout 10 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" 2>&1 | tail -1
sleep 1
timeout 8 ros2 topic echo /safety/status --once 2>&1 | tail -2
timeout 8 ros2 topic echo /chassis/state --once 2>&1 | grep -E "control_mode|motion_allowed|feedback_valid"
timeout 8 ros2 topic echo /odom --once > /tmp/odom_start.txt 2>&1
setsid nohup python3 /root/ros2_bridge/record_trace.py /tmp/trace_$TASK.csv > /tmp/recorder.log 2>&1 < /dev/null &
sleep 2
echo "=== start mission ==="
timeout 10 ros2 service call /demo_mission/start std_srvs/srv/Trigger 2>&1 | tail -2
start_ts=$(date +%s)
final=""
for i in $(seq 1 $WATCH); do
  st=$(timeout 6 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
  now=$(date +%s); el=$((now-start_ts))
  if [ -n "$st" ] && [ "$st" != "RUNNING" ] && [ "$st" != "IDLE" ]; then final="$st"; echo "[$el s] state=$st"; break; fi
  if [ $el -gt $WATCH ]; then
    timeout 5 ros2 service call /demo_mission/stop std_srvs/srv/Trigger >/dev/null 2>&1
    final="WATCHDOG_STOP"; break
  fi
  sleep 1
done
timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
sleep 2
timeout 8 ros2 topic echo /odom --once > /tmp/odom_end.txt 2>&1
timeout 8 ros2 topic echo /chassis/state --once > /tmp/chassis_end.txt 2>&1
pkill -f record_trace 2>/dev/null
pkill -f demo_mission_manager 2>/dev/null
echo "=== FINAL: $final ==="
echo "--- odom_end ---"; head -18 /tmp/odom_end.txt
echo "--- chassis_end ---"; head -24 /tmp/chassis_end.txt
echo "--- mission events ---"; grep -E "goal |state=" /tmp/bench_mission.log | head -10
echo "--- trace rows ---"; wc -l /tmp/trace_$TASK.csv 2>/dev/null
bash /root/ros2_bridge/stop_nav.sh 2>&1 | tail -1
pkill -f RemoteControl_WheelPID_V2 2>/dev/null
pkill -f sbus_emu 2>/dev/null
pkill -f flat_ground_grid 2>/dev/null
echo V4_BENCH_DONE
