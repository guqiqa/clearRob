#!/usr/bin/env python3
"""Real-time SBUS channel monitor — shows every frame change immediately."""
import os, time, struct, fcntl, sys

# Init port_bridge
fd = os.open('/dev/ttyS1', os.O_RDWR | os.O_NOCTTY | os.O_NDELAY)
TCGETS2, TCSETS2 = 0x802c542a, 0x402c542b
buf = bytearray(72); fcntl.ioctl(fd, TCGETS2, buf)
cflag = struct.unpack_from('I', buf, 8)[0]
cflag &= 0xffffffcf; cflag |= 0x30 | 0x100 | 0x40 | 0x880; cflag &= ~0x200
struct.pack_into('I', buf, 8, cflag)
buf[17+5] = 0; buf[17+6] = 0
cflag2 = struct.unpack_from('I', buf, 8)[0]
cflag2 &= 0xffffeff0; cflag2 |= 0x1000
struct.pack_into('I', buf, 8, cflag2)
for off in (36, 40, 40, 44): struct.pack_into('I', buf, off, 100000)
fcntl.ioctl(fd, TCSETS2, buf)
time.sleep(0.3)

SBUS_LEN = 25
prev = None
data = b''
t0 = time.time()

print("Real-time SBUS Monitor — press Ctrl+C to stop\n")
print(f"{'Time':>8s}  {'CH1':>5s} {'CH2':>5s} {'CH5':>5s} {'CH7':>5s}  Status")
print("-" * 55)

try:
    while time.time() - t0 < 120:  # 2 min timeout
        data += os.read(fd, 256)
        while len(data) >= SBUS_LEN:
            hdr = data.find(b'\x0f')
            if hdr < 0: data = b''; break
            if hdr > 0: data = data[hdr:]
            if len(data) < SBUS_LEN: break
            if data[SBUS_LEN-1] != 0x00: data = data[1:]; continue

            frame = data[:SBUS_LEN]; data = data[SBUS_LEN:]
            chs = [0]*16; bp = bi = 0
            for ch in range(16):
                v = bits = 0
                while bits < 11:
                    if bp >= 8: bi += 1; bp = 0
                    n = min(11-bits, 8-bp)
                    v |= ((frame[bi] >> bp) & ((1<<n)-1)) << bits
                    bp += n; bits += n
                chs[ch] = v

            ch1, ch2, ch5, ch7 = chs[0], chs[1], chs[4], chs[6]
            flags = frame[23]
            fs = bool(flags & 0x04)
            fl = bool(flags & 0x08)

            key = (ch1, ch2, ch5, ch7, fs, fl)
            if key != prev:
                elapsed = time.time() - t0
                bar1 = '█' * int(abs(ch1-992)/50) + ('R' if ch1<992 else 'L' if ch1>992 else '·')
                bar2 = '█' * int(abs(ch2-992)/50) + ('B' if ch2<992 else 'F' if ch2>992 else '·')
                status = f"FS={fs} FL={fl}"
                if fs or fl: status = f"\033[31m{status}\033[0m"
                print(f"{elapsed:7.1f}s  {ch1:5d} {ch2:5d} {ch5:5d} {ch7:5d}  {status}")
                prev = key

except KeyboardInterrupt:
    pass
finally:
    os.close(fd)
    print("\nMonitor stopped.")
