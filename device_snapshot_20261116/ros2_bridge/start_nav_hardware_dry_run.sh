#!/bin/bash
# 启动“导航 + Python 底盘速度节点”链路（安全默认态：手动模式、零速、无模拟目标）
#
# 默认 dry-run：不启动会打开 can0 并发送 CAN 心跳的 wheel_speed_node，且
# hardware_adapter 的 publish_cmd_vel=false。只有轮子离地台架才允许加
# --allow-cmd-vel（此时会启动 CAN 速度节点并允许安全出口发布 /cmd_vel）。
# 本脚本不启动 navigation_scenario，也不会自动使能自主模式；目标必须由
# 明确的任务节点人工下发。
set +u
. /root/nav_ws/env.sh
BRIDGE=/root/ros2_bridge
ALLOW_CMD_VEL=0
for arg in "$@"; do
  if [ "$arg" = "--allow-cmd-vel" ]; then ALLOW_CMD_VEL=1; fi
done

bash "$BRIDGE/stop_nav.sh" >/dev/null 2>&1
pkill -f hardware_adapter 2>/dev/null
pkill -f wheel_speed_node 2>/dev/null
sleep 1
# 1) 只有显式台架许可时才启动 CAN 速度节点。
if [ "$ALLOW_CMD_VEL" = "1" ]; then
  # 设备重启后 can0 会回到 DOWN 且未配置比特率：先配置 500 kbit/s 再拉起
  ip link set can0 down 2>/dev/null
  ip link set can0 type can bitrate 500000 2>/dev/null
  ip link set can0 up 2>/dev/null
  ip link set can0 txqueuelen 1000 2>/dev/null
  echo "[start] can0: $(ip -br link show can0 2>/dev/null | awk '{print $2, $3}')"
  # 可选：用环境变量覆盖起步 kick 做峰值试验（默认沿用 config.xml）
  KICK_ARGS=""
  if [ -n "${START_CURRENT_MA:-}" ]; then KICK_ARGS="$KICK_ARGS --start-current-ma $START_CURRENT_MA"; fi
  if [ -n "${START_KICK_MS:-}" ]; then KICK_ARGS="$KICK_ARGS --start-kick-ms $START_KICK_MS"; fi
  setsid nohup python3 /root/wheel_speed_node.py $KICK_ARGS > /tmp/wheel_speed_node.log 2>&1 < /dev/null &
  sleep 2
  # 台架/地面演示没有真实感知时，用全空白局部栅格供导航节点规划（不发布任何目标）
  setsid nohup python3 /root/ros2_bridge/flat_ground_grid.py > /tmp/flat_grid.log 2>&1 < /dev/null &
  echo "[start] flat_ground_grid started (all-free local occupancy grid, no goals)"
else
  echo "[start] dry-run: wheel_speed_node not started; no CAN access"
fi

# 2) 导航 + 安全层（不带 scenario；with_scenario 默认 false）
setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/nav_hw_dry.log 2>&1 < /dev/null &
sleep 8

# 3) 安全出口适配器（默认干跑；台架时才允许发速度）
if [ "$ALLOW_CMD_VEL" = "1" ]; then
  PUB=true
  echo "[start] WARNING: publish_cmd_vel=true（只允许轮子离地的台架测试）"
else
  PUB=false
  echo "[start] dry-run: publish_cmd_vel=false（底盘不会收到速度指令）"
fi
setsid nohup python3 /root/nav_ws/scripts/hardware_adapter.py \
  --ros-args -p publish_cmd_vel:=$PUB > /tmp/hw_adapter.log 2>&1 < /dev/null &
sleep 2

timeout 10 ros2 node list
# 安全启动：保持手动/零速，绝不自动使能自主模式
timeout 10 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" 2>&1 | tail -1
echo "[start] manual mode + zero speed; publish a goal only from an explicit task node"
