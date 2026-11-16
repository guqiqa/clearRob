#!/bin/bash
# 切换底盘输入模式: rc = 真实SBUS(/dev/ttyS5), emu = 中性帧PTY(导航/测试用)
set -e
case "$1" in
  on|rc)
    touch /etc/laifei-chassis-rc-input
    systemctl restart laifei-chassis-v4.service
    sleep 3
    echo "mode: RC (config.xml, /dev/ttyS5)"
    systemctl is-active laifei-chassis-v4.service
    journalctl -u laifei-chassis-v4.service -n 12 --no-pager | tail -12
    ;;
  off|emu)
    rm -f /etc/laifei-chassis-rc-input
    systemctl restart laifei-chassis-v4.service
    sleep 3
    echo "mode: EMU (PTY neutral)"
    systemctl is-active laifei-chassis-v4.service
    journalctl -u laifei-chassis-v4.service -n 12 --no-pager | tail -12
    ;;
  *)
    echo "usage: $0 on|off"
    exit 1
    ;;
esac
