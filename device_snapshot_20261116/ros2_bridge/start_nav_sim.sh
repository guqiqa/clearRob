#!/bin/bash
# 启动导航仿真栈（chassis_adapter 仿真 + scenario 合成栅格/目标）
set +u
. /root/nav_ws/env.sh
pkill -f 'navigation_simulation.launch.py' 2>/dev/null
sleep 1
: > /tmp/nav_sim.log
setsid nohup ros2 launch navigation_ros2 navigation_simulation.launch.py > /tmp/nav_sim.log 2>&1 < /dev/null &
echo "[nav] launch started, waiting nodes..."
for i in $(seq 1 60); do
  if ros2 node list 2>/dev/null | grep -q navigation_node; then break; fi
  sleep 0.5
done
ros2 node list
echo "[nav] enable autonomous in simulation"
ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" 2>&1 | tail -2
