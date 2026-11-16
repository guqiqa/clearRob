#!/bin/bash
set +u
source /root/nav_ws/env.sh
TASK=$1; shift
WATCH=${WATCH:-180}
PARAMS=${PARAMS:-/root/nav_ws/config/ground_demo.yaml}
PROFILE=${PROFILE:-GROUND_DEMO}
pkill -f demo_mission_manager 2>/dev/null
pkill -f record_trace 2>/dev/null
bash /root/ros2_bridge/stop_nav.sh >/dev/null 2>&1
sleep 1
echo "=== start bench chain (params=$PARAMS profile=$PROFILE) ==="
timeout 90 bash /root/ros2_bridge/start_nav_hardware_dry_run.sh --allow-cmd-vel 2>&1 | tail -4
sleep 1
timeout 10 ros2 param set /navigation_node profile $PROFILE 2>&1 | tail -1
timeout 10 ros2 param get /navigation_node profile 2>&1 | tail -1
# 安全层上限按本次验证工况设置（dry-run 之外的第二道限制）
SAFE_LIN=${SAFE_LIN:-0.50}
SAFE_ANG=${SAFE_ANG:-1.00}
timeout 10 ros2 param set /safety_supervisor max_linear_x_mps $SAFE_LIN >/dev/null 2>&1
timeout 10 ros2 param set /safety_supervisor max_angular_z_radps $SAFE_ANG >/dev/null 2>&1
echo "[bench] safety limits: max_linear=$SAFE_LIN max_angular=$SAFE_ANG"
setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file $PARAMS -p task:=$TASK "$@" > /tmp/bench_mission.log 2>&1 < /dev/null &
sleep 5
timeout 10 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" 2>&1 | tail -1
sleep 1
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
    echo "WATCHDOG: mission stop"
    timeout 5 ros2 service call /demo_mission/stop std_srvs/srv/Trigger >/dev/null 2>&1
    final="WATCHDOG_STOP"; break
  fi
  sleep 1
done
timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
sleep 2
timeout 8 ros2 topic echo /odom --once > /tmp/odom_end.txt 2>&1
pkill -f record_trace 2>/dev/null
pkill -f demo_mission_manager 2>/dev/null
echo "=== FINAL: $final ==="
echo "--- odom_start ---"; head -18 /tmp/odom_start.txt
echo "--- odom_end ---"; head -18 /tmp/odom_end.txt
echo "--- mission events ---"; grep -E "goal |state=" /tmp/bench_mission.log | head -10
echo "--- trace rows ---"; wc -l /tmp/trace_$TASK.csv 2>/dev/null
bash /root/ros2_bridge/stop_nav.sh 2>&1 | tail -1
pkill -f wheel_speed_node 2>/dev/null
echo BENCH_RUN_DONE
