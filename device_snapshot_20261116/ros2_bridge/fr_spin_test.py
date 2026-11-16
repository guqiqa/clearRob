#!/usr/bin/env python3
# 单轮匀速转动测试：直接通过 CAN 驱动右前轮 (FR, ID=4) 以固定 erpm 转动，
# 由人工数圈数，用于反算实际 pp*gr。
# 前置：systemctl stop laifei-chassis-v4.service（避免与 v4 抢 CAN）；轮子离地；人持急停。
# 用法: python3 fr_spin_test.py [erpm] [duration_s]
import socket, struct, sys, time, select

AF_CAN = 29
CAN_RAW = 1
FR = 4
ALL = (2, 1, 4, 3)

erpm_cmd = int(sys.argv[1]) if len(sys.argv) > 1 else 1560
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

s = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
s.bind(("can0",))
s.setblocking(False)

def send(can_id, data):
    frame = struct.pack("<IB3x8s", can_id, len(data), bytes(data).ljust(8, b"\x00"))
    s.send(frame)

def speed_cmd(can_id, erpm):
    raw = struct.pack(">i", int(erpm))
    send(can_id, bytes([0x02]) + raw)

def query(can_id):
    send(can_id, bytes([0x0F, 0x01]))

def heartbeat(can_id):
    send(can_id, bytes([0x00]))

def set_max_current(can_id, value_10ma):
    send(can_id, bytes([0x14, (value_10ma >> 8) & 0xFF, value_10ma & 0xFF]))

set_max_current(FR, 100)  # 1.0 A 上限，保证低速安全
speed_cmd(FR, 0)
time.sleep(0.2)
print("SPIN START  erpm_cmd=%d duration=%.0fs  (开始数圈!)" % (erpm_cmd, duration), flush=True)

start = time.time()
last_fb_t = None
sum_erpm_s = 0.0
last_fb = 0
loop_i = 0
next_print = start + 1.0
while time.time() - start < duration:
    now = time.time()
    speed_cmd(FR, erpm_cmd)
    query(FR)
    loop_i += 1
    if loop_i % 5 == 0:
        for cid in ALL:
            heartbeat(cid)
    while True:
        r, _, _ = select.select([s], [], [], 0)
        if not r:
            break
        frame = s.recv(16)
        can_id = struct.unpack_from("<I", frame, 0)[0] & 0x1FFFFFFF
        dlc = frame[4]
        data = frame[8:8 + dlc]
        if can_id == FR and dlc >= 6 and data[0] == 0x0F and data[1] == 0x01:
            fb = struct.unpack(">i", data[2:6])[0]
            t = time.time()
            if last_fb_t is not None:
                sum_erpm_s += fb * (t - last_fb_t)
            last_fb_t = t
            last_fb = fb
    if time.time() >= next_print:
        print("  %5.1fs erpm=%8d" % (time.time() - start, last_fb), flush=True)
        next_print += 1.0
    time.sleep(0.05)

for _ in range(5):
    speed_cmd(FR, 0)
    time.sleep(0.05)
for _ in range(5):
    for cid in ALL:
        heartbeat(cid)
    time.sleep(0.05)
print("SPIN STOP", flush=True)
print("--- summary ---")
print("sum_erpm_s = %.1f" % sum_erpm_s)
try:
    revs = float(input("请输入刚才 FR 实际转过的圈数: "))
except Exception:
    revs = 0.0
if revs > 0:
    K = sum_erpm_s / (60.0 * revs)
    print("revs=%.2f  K(实际 pp*gr) = %.3f" % (revs, K))
    print("若 K≈52 -> 配置正确；若 K≈36.4 -> 轮径应约 0.08 m")
