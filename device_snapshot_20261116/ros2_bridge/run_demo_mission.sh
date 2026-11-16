#!/bin/bash
# 运行固定演示任务节点（设备未安装 colcon，直接从源码运行）
set +u
source /root/nav_ws/env.sh
export PYTHONPATH=/root/nav_ws/src/demo_mission_manager:$PYTHONPATH
exec python3 -m demo_mission_manager.node "$@"
