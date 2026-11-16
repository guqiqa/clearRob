#!/bin/bash
# systemd v4 常驻版速度扫描：不启停 v4，只停 safety/nav 后直接注入 SafeMotionCommand
set +u
source /root/nav_ws/env.sh
TAG=${1:-systemd}
SPEEDS=${2:-"0.05 0.08 0.10 0.12"}
systemctl is-active laifei-chassis-v4.service | grep -q active || { echo "v4 unit not active"; exit 1; }
pgrep -f "wheel_speed_node|hardware_adapter" >/dev/null && { echo "forbidden CAN node running"; exit 1; }
pkill -f safety_supervisor_node 2>/dev/null
pkill -f navigation_node 2>/dev/null
pkill -f demo_mission_manager 2>/dev/null
pkill -f sweep_driver 2>/dev/null
pkill -f record_trace 2>/dev/null
sleep 1
timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
echo 0 > /tmp/sweep_speed
setsid nohup python3 /root/ros2_bridge/sweep_driver.py 90 > /tmp/sweep_driver.log 2>&1 < /dev/null &
sleep 1
for v in $SPEEDS; do
  tag=$(echo "$v" | tr "." "_")
  echo "$v" > /tmp/sweep_speed
  setsid nohup python3 /root/ros2_bridge/record_trace.py "/tmp/sweep_${TAG}_${tag}.csv" > /tmp/sweep_rec.log 2>&1 < /dev/null &
  sleep 9
  pkill -f record_trace 2>/dev/null
  echo "[sweep] $v done"
  sleep 1
done
echo 0 > /tmp/sweep_speed
sleep 1
pkill -f sweep_driver 2>/dev/null
pkill -f record_trace 2>/dev/null
timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
echo "[sweep] done tag=$TAG"
