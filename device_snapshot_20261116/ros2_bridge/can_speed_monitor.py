#!/usr/bin/env python3
# 只读监听 can0 的 OID 轮速回包（0x0F 0x01），积分得到 erpm·s，用于反算 pp*gr。
# 用法: python3 can_speed_monitor.py [duration_s] [revs]
#   旋转指定轮子正好 revs 圈后，脚本输出 K = sum(erpm*dt)/(60*revs) = 实际 pp*gr
import socket, struct, sys, time, select

CAN_RAW = 1
AF_CAN = 29
NAMES = {2: "FL", 1: "RL", 4: "FR", 3: "RR"}

duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
revs = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
s = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
s.bind(("can0",))
s.setblocking(False)
print("listening can0 %.0f s, revs=%.1f (0=只打印)" % (duration, revs), flush=True)
start = time.time()
last = {}
total = {}
count = {}
while time.time() - start < duration:
    r, _, _ = select.select([s], [], [], 0.1)
    if not r:
        continue
    now = time.time()
    frame = s.recv(16)
    can_id = struct.unpack_from("<I", frame, 0)[0] & 0x1FFFFFFF
    dlc = frame[4]
    data = frame[8:8 + dlc]
    if dlc >= 6 and data[0] == 0x0F and data[1] == 0x01:
        erpm = struct.unpack(">i", data[2:6])[0]
        name = NAMES.get(can_id, str(can_id))
        if name in last:
            total[name] = total.get(name, 0.0) + erpm * (now - last[name])
        last[name] = now
        count[name] = count.get(name, 0) + 1
        print("%6.2fs %-3s erpm=%8d" % (now - start, name, erpm), flush=True)
print("--- summary ---")
for name in ("FL", "RL", "FR", "RR"):
    if name in total:
        ssum = total[name]
        print("%-3s samples=%d sum_erpm_s=%.1f" % (name, count[name], ssum))
        if revs > 0:
            K = ssum / (60.0 * revs)
            print("    revs=%.1f => K(pp*gr) = %.3f" % (revs, K))
