#!/bin/bash
# v4 速度扫描：持续发布 SafeMotionCommand（无间隙），速度由 /tmp/sweep_speed 控制
set +u
source /root/nav_ws/env.sh
TAG=${1:-current}
SPEEDS=${2:-"0.05 0.08 0.10 0.12"}
pkill -f demo_mission_manager 2>/dev/null
pkill -f record_trace 2>/dev/null
pkill -f sweep_driver 2>/dev/null
bash /root/ros2_bridge/stop_v4_bench.sh >/dev/null 2>&1
sleep 1
timeout 120 bash /root/ros2_bridge/start_v4_bench.sh > /tmp/sweep_start.log 2>&1
sleep 1
pkill -f safety_supervisor_node 2>/dev/null
pkill -f navigation_node 2>/dev/null
sleep 1
echo 0 > /tmp/sweep_speed
setsid nohup python3 /root/ros2_bridge/sweep_driver.py 60 > /tmp/sweep_driver.log 2>&1 < /dev/null &
sleep 1
timeout 10 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" 2>&1 | tail -1
sleep 1
for v in $SPEEDS; do
  tag=$(echo $v | tr "." "_")
  echo $v > /tmp/sweep_speed
  setsid nohup python3 /root/ros2_bridge/record_trace.py /tmp/sweep_${TAG}_${tag}.csv > /tmp/sweep_rec.log 2>&1 < /dev/null &
  sleep 9
  pkill -f record_trace 2>/dev/null
  echo "[sweep] speed=$v done"
  sleep 1
done
echo 0 > /tmp/sweep_speed
sleep 1
pkill -f sweep_driver 2>/dev/null
pkill -f record_trace 2>/dev/null
timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
bash /root/ros2_bridge/stop_v4_bench.sh >/dev/null 2>&1
echo "[sweep] done tag=$TAG"
