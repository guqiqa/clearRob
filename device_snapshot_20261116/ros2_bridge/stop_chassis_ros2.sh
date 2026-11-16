#!/bin/bash
set +u
echo "[ros2] 停止控制器"
pkill -INT -f '^\./RemoteControl_WheelPID_V2' 2>/dev/null || true
sleep 1
if [ -f /tmp/sbus_emu.pid ]; then
  cat /tmp/sbus_emu.pid | xargs -r kill 2>/dev/null || true
fi
rm -f /tmp/sbus_emu_path /tmp/sbus_emu.pid
echo "[ros2] 已停止"
