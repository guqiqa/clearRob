#!/bin/bash
# 以 ROS 2 控制方式启动底盘控制器：
#   1) 停止旧控制器 + SBUS 仿真
#   2) 启动 SBUS 中位仿真（无接收机时必需）
#   3) 用 config.xml 生成 config_emu.xml 并启动控制器（带 ROS 2 环境）
#   4) 等待 /laifei_chassis_driver 出现，并自动开启 autonomous 模式，
#      之后 /cmd_vel 可直接控制底盘（300 ms 无指令自动停车）
set +u
BASE=${BASE:-/root/chassis_ros2_app_20260921}
BRIDGE=${BRIDGE:-/root/ros2_bridge}
LOG=${LOG:-/tmp/wheel_ctrl_ros2.log}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

echo "[ros2] 停止旧控制器"
pkill -INT -f '^\./RemoteControl_WheelPID_V2' 2>/dev/null || true
sleep 1
if [ -f /tmp/sbus_emu.pid ]; then
  cat /tmp/sbus_emu.pid | xargs -r kill 2>/dev/null || true
  sleep 0.3
fi
rm -f /tmp/sbus_emu_path /tmp/sbus_emu.pid

echo "[ros2] 启动 SBUS 中位仿真"
setsid nohup python3 "$BRIDGE/sbus_emu.py" > /tmp/sbus_emu.log 2>&1 < /dev/null &
for i in $(seq 1 30); do
  [ -s /tmp/sbus_emu_path ] && break
  sleep 0.1
done
PTY=$(cat /tmp/sbus_emu_path 2>/dev/null)
if [ -z "$PTY" ]; then echo "[ros2] SBUS 仿真启动失败"; exit 1; fi
echo "[ros2] SBUS PTY=$PTY"

echo "[ros2] 生成运行配置"
python3 "$BRIDGE/prep_config.py" "$BASE" "$PTY" || exit 1

echo "[ros2] 启动控制器"
cd "$BASE" || exit 1
: > "$LOG"
setsid nohup bash -lc "source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; exec ./RemoteControl_WheelPID_V2 ./config_emu.xml" > "$LOG" 2>&1 < /dev/null &

echo "[ros2] 等待节点 /laifei_chassis_driver"
OK=0
for i in $(seq 1 40); do
  if ros2 node list 2>/dev/null | grep -q laifei_chassis_driver; then OK=1; break; fi
  sleep 0.25
done
if [ "$OK" != "1" ]; then echo "[ros2] 节点未出现，检查 $LOG"; tail -20 "$LOG"; exit 1; fi

echo "[ros2] 开启 autonomous 模式"
ros2 service call /set_autonomous_mode std_srvs/srv/SetBool "{data: true}" 2>&1 | tail -2

echo "[ros2] 就绪"
ros2 node list
ros2 topic list
