#!/bin/bash
# 由 laifei-chassis-v4.service 调用：严格生成 config_emu.xml 并 exec v4（失败即退出，不带默认值）
set +u
. /root/nav_ws/env.sh
set -eu
BASE=/root/chassis_ros2_nav_v4
# 真实遥控模式：/etc/laifei-chassis-rc-input 存在时直接读 config.xml 的 /dev/ttyS5
if [ -f /etc/laifei-chassis-rc-input ]; then
  echo "RC_INPUT_MODE: real SBUS via /dev/ttyS5"
  exec "$BASE/build/RemoteControl_WheelPID_V2" "$BASE/src/config.xml"
fi
PTY=$(cat /tmp/sbus_emu_path 2>/dev/null || true)
if [ -z "${PTY:-}" ] || [ ! -e "$PTY" ]; then
  echo "ERROR: /tmp/sbus_emu_path missing or invalid: ${PTY:-<empty>}" >&2
  exit 1
fi
sed "s#<SBus_Port>[^<]*</SBus_Port>#<SBus_Port>${PTY}</SBus_Port>#" "$BASE/src/config.xml" > "$BASE/src/config_emu.xml"
exec "$BASE/build/RemoteControl_WheelPID_V2" "$BASE/src/config_emu.xml"
