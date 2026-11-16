#!/bin/bash
# 导航链路安全停机：先通过安全层撤销自主模式，再按依赖顺序停止节点。
# 注意：hardware_adapter 与 wheel_speed_node 必须在这里停掉，
# 不能再依赖文档里额外的 pkill -f hardware_adapter / wheel_speed_node。
set +u
if [ -f /root/nav_ws/env.sh ]; then . /root/nav_ws/env.sh; fi

# 1) 通过安全层停机，避免直接向公共 /cmd_vel 发布绕过安全链路的指令。
if timeout 4 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool \
  "{data: false}" >/tmp/nav_stop_mode.log 2>&1; then
  cat /tmp/nav_stop_mode.log
  sleep 0.5
else
  echo "[nav] WARNING: could not confirm safe-layer stop; use physical E-stop if motion may continue"
fi

# 2) 停上层节点
pkill -f 'navigation_simulation.launch.py' 2>/dev/null
pkill -f 'navigation_hardware' 2>/dev/null
pkill -f 'navigation_node' 2>/dev/null
pkill -f 'safety_supervisor_node' 2>/dev/null
pkill -f 'chassis_adapter_node' 2>/dev/null
pkill -f 'navigation_scenario' 2>/dev/null

# 3) 停适配器与速度节点（速度节点退出时会释放电机力矩）
pkill -f 'hardware_adapter' 2>/dev/null
pkill -f 'wheel_speed_node' 2>/dev/null
pkill -f 'flat_ground_grid' 2>/dev/null
sleep 1

# 4) 确认适配器与速度节点确实已退出（比 DDS 发布者计数可靠）
if pgrep -af 'hardware_adapter|wheel_speed_node' >/dev/null 2>&1; then
  echo "[nav] stopped (WARNING: hardware_adapter/wheel_speed_node still running)"
else
  echo "[nav] stopped (adapter and wheel_speed_node are gone)"
fi
