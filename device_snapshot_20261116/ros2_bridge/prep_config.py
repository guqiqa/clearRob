#!/usr/bin/env python3
"""根据 SBus_Port 生成控制器运行配置 config_emu.xml（不动原始 config.xml）。"""
import sys
import os

base = sys.argv[1] if len(sys.argv) > 1 else '/root/chassis_ros2_app_20260921'
pty = sys.argv[2]
src = os.path.join(base, 'config.xml')
dst = os.path.join(base, 'config_emu.xml')
text = open(src, encoding='utf-8').read()
old = '<SBus_Port>/dev/ttyS5</SBus_Port>'
new = '<SBus_Port>' + pty + '</SBus_Port>'
if old not in text:
    raise SystemExit('config.xml 中未找到 SBus_Port 标签')
open(dst, 'w', encoding='utf-8').write(text.replace(old, new, 1))
print('config_emu.xml: SBus_Port ->', pty)
