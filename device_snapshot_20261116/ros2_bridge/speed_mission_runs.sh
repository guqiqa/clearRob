#!/bin/bash
# speed 模式低速任务验证：链启动一次，4 档速度 × 3 次 FORWARD 0.5 m
set +u
source /root/nav_ws/env.sh
PARAMS=/root/nav_ws/config/ground_demo_bench_stable.yaml
pkill -f demo_mission_manager 2>/dev/null
pkill -f record_trace 2>/dev/null
bash /root/ros2_bridge/stop_v4_bench.sh >/dev/null 2>&1
sleep 1
timeout 120 bash /root/ros2_bridge/start_v4_bench.sh > /tmp/sm_start.log 2>&1
sleep 1
for pair in "GROUND_BENCH_005 0.05" "GROUND_BENCH_008 0.08" "GROUND_BENCH_010 0.10" "GROUND_BENCH_012 0.12"; do
  set -- $pair; PROFILE=$1; V=$2
  timeout 10 ros2 param set /navigation_node profile $PROFILE >/dev/null 2>&1
  timeout 10 ros2 param set /safety_supervisor max_linear_x_mps $V >/dev/null 2>&1
  timeout 10 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1
  for i in 1 2 3; do
    pkill -f demo_mission_manager 2>/dev/null
    setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file $PARAMS -p task:=FORWARD -p distance_m:=0.5 > /tmp/sm_mission.log 2>&1 < /dev/null &
    sleep 4
    timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
    timeout 8 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
    sleep 1
    setsid nohup python3 /root/ros2_bridge/record_trace.py /tmp/sm_${V}_${i}.csv > /tmp/sm_rec.log 2>&1 < /dev/null &
    sleep 1
    timeout 10 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1
    final=""
    for k in $(seq 1 60); do
      st=$(timeout 5 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
      case "$st" in COMPLETED|ABORTED|STOPPED|TIMEOUT|CHASSIS_FAULT|ESTOP) final=$st; break;; esac
      sleep 1
    done
    sleep 1
    pkill -f record_trace 2>/dev/null
    timeout 5 ros2 topic echo /odom --once > /tmp/sm_odom_${V}_${i}.txt 2>&1
    echo "run profile=$PROFILE v=$V rep=$i final=$final"
    timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
    timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
    pkill -f demo_mission_manager 2>/dev/null
    sleep 1
  done
done
pkill -f record_trace 2>/dev/null
bash /root/ros2_bridge/stop_v4_bench.sh >/dev/null 2>&1
echo "[speedmissions] done"
