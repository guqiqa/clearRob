#!/usr/bin/env python3
"""无遥控器接收机时，给底盘控制器提供 SBUS 中位帧（14 ms/帧，100 kbit/s 参数由控制器设置）。
启动后把 PTY 路径写到 /tmp/sbus_emu_path，PID 写到 /tmp/sbus_emu.pid。
说明：控制器开机会读取 SBUS；没有接收机时通道全 0 会被解码成“油门+1/转向-1”，
      触发安全层 motion_requested_without_fresh_feedback 锁死，因此需要中性帧。
"""
import os
import pty
import time

master, slave = pty.openpty()
path = os.ttyname(slave)
with open('/tmp/sbus_emu_path', 'w') as f:
    f.write(path)
with open('/tmp/sbus_emu.pid', 'w') as f:
    f.write(str(os.getpid()))

channels = [1024] * 16          # 中位
bits = 0
for i, v in enumerate(channels):
    bits |= (v & 0x7FF) << (11 * i)
data = bytearray(25)
data[0] = 0x0F                  # SBUS 帧头
for i in range(22):
    data[1 + i] = (bits >> (8 * i)) & 0xFF
data[23] = 0x00                 # 无 frame_lost / failsafe 标志
data[24] = 0x00
frame = bytes(data)
os.write(master, frame)
try:
    while True:
        os.write(master, frame)
        time.sleep(0.014)
except Exception:
    pass
