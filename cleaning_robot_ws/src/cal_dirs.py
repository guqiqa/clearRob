#!/usr/bin/env python3
"""
cal_dirs.py — INDEPENDENT calibration & config tool for test time.

Runs WITHOUT the chassis_driver node.  It opens can0 itself, brings each
motor into motion with a small OID current command, reads the RAW erpm
response to determine each motor's PHYSICAL direction, and (optionally) writes
the corrected motor_dirs AND base parameters into phase1_params.yaml.

Sub-commands:
  python3 cal_dirs.py                    # measure directions, print result only
  python3 cal_dirs.py --apply            # measure + WRITE motor_dirs to yaml
  python3 cal_dirs.py --basics           # ask base params, write them to yaml
  python3 cal_dirs.py --rollback         # restore yaml from the .bak backup
  python3 cal_dirs.py --show             # print current yaml base params

SAFETY (why --apply is explicit):
  - Direction is a safety-critical parameter.  The tool measures the MOTOR
    physical spin, but the chassis needs "wheel drives the car forward".
    The gap is bridged by assumptions (+current=CCW, right side reverse-mount).
    So by default the tool only PRINTS the result.
  - --apply backs up phase1_params.yaml → phase1_params.yaml.bak BEFORE writing,
    so --rollback restores it.

PREREQUISITES:
  - chassis_driver must be STOPPED (this tool owns can0 while it runs)
  - wheels jacked up / free-spinning (motors WILL spin briefly)
  - python-can installed

Typical flow:
  pkill -f chassis_node                  # release can0
  python3 cal_dirs.py                    # measure, review the output
  python3 cal_dirs.py --basics           # confirm base params (pole_pairs etc.)
  python3 cal_dirs.py --apply            # write motor_dirs + basics to yaml
  # restart chassis, then on-ground: python3 drivetest.py forward 1
  python3 cal_dirs.py --rollback         # only if a test shows it's wrong
"""
import os
import shutil
import struct
import sys
import time

import can

# ---------------------------------------------------------------------------
# YAML paths — only the INSTALL copy that launch actually loads.  The src copy
# is a stale flat-format file on the device and is managed by git locally.
# ---------------------------------------------------------------------------
WS = "/root/cleaning_robot_ws"
YAML_INSTALL = f"{WS}/install/cleaning_robot_bringup/share/cleaning_robot_bringup/config/phase1_params.yaml"
YAML_PATHS = [YAML_INSTALL]
BAK = ".bak"   # appended to each path for rollback

CAN_IDS = [2, 1, 4, 3]        # FL, RL, FR, RR
NAMES = {2: "FL", 1: "RL", 4: "FR", 3: "RR"}
RIGHT_IDS = {4, 3}            # FR, RR — reverse-mounted on this robot

# OID commands
CMD_HEARTBEAT = 0x00
CMD_CURRENT = 0x01
CMD_QUERY = 0x0F
QUERY_SPEED = 0x01

# ---------------------------------------------------------------------------
# CAN helpers
# ---------------------------------------------------------------------------


def send(bus, can_id, data, timeout=0.01):
    bus.send(can.Message(arbitration_id=can_id, data=data,
                         is_extended_id=False), timeout=timeout)


def heartbeat(bus, can_id):
    send(bus, can_id, bytes([CMD_HEARTBEAT]))


def current(bus, can_id, value_10ma):
    value_10ma = max(-32768, min(32767, value_10ma))
    raw = struct.pack(">h", value_10ma)
    send(bus, can_id, bytes([CMD_CURRENT]) + raw)


def query_speed(bus, can_id):
    send(bus, can_id, bytes([CMD_QUERY, QUERY_SPEED]))


def read_erpm(bus, timeout=0.1):
    end = time.time() + timeout
    while time.time() < end:
        msg = bus.recv(timeout=max(0.0, end - time.time()))
        if msg is None:
            return None
        if msg.arbitration_id in CAN_IDS and msg.dlc >= 6 and msg.data[0] == CMD_QUERY:
            erpm = struct.unpack(">i", msg.data[2:6])[0]
            return msg.arbitration_id, erpm
    return None


def spin_one(bus, can_id, current_10ma, duration=1.2):
    for _ in range(5):
        heartbeat(bus, can_id)
        time.sleep(0.02)
    current(bus, can_id, current_10ma)
    end = time.time() + duration
    best = 0
    while time.time() < end:
        query_speed(bus, can_id)
        r = read_erpm(bus)
        if r and r[0] == can_id and r[1] != 0:
            best = r[1]
        time.sleep(0.02)
    current(bus, can_id, 0)
    return best


# ---------------------------------------------------------------------------
# YAML helpers — minimal line-aware editor (preserves comments/formatting)
# ---------------------------------------------------------------------------


def _load_lines(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.readlines()


def _write_lines(path, lines):
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(lines)


def backup(path):
    shutil.copy2(path, path + BAK)
    return path + BAK


def rollback(path):
    bak = path + BAK
    if not os.path.exists(bak):
        print(f"  no backup at {bak} — nothing to roll back")
        return False
    shutil.copy2(bak, path)
    print(f"  restored {path} from {bak}")
    return True


def get_value(lines, key):
    """Return the current value text after `<key>:` at 6-space indent (inside chassis_core:)."""
    for ln in lines:
        if ln.startswith("      " + key + ":") and "chassis_core" not in ln:
            body = ln.split(":", 1)[1].strip()
            body = body.split("#")[0].strip()
            return body
    return None


def set_value(lines, key, new_text, comment=""):
    """Rewrite the line `<key>: <new_text>   # <comment>`, preserving position.
    If the key isn't found, append a placeholder line after the first `chassis_core:` block marker."""
    newline = f"      {key}: {new_text}"
    if comment:
        newline += f"      # {comment}"
    newline += "\n"
    for i, ln in enumerate(lines):
        if ln.startswith("      " + key + ":"):
            lines[i] = newline
            return True
    # not found — insert after the last known chassis_core base param (motor_ids)
    for i, ln in enumerate(lines):
        if ln.startswith("      motor_ids:"):
            lines.insert(i + 1, newline)
            return True
    return False


def apply_base_params(updates):
    """Write base params to both yaml files (src + install)."""
    for path in YAML_PATHS:
        if not os.path.exists(path):
            print(f"  skip (not found): {path}")
            continue
        lines = _load_lines(path)
        for key, (val, comment) in updates.items():
            if set_value(lines, key, str(val), comment):
                print(f"  {path.split('/')[-1]}: {key} -> {val}")
            else:
                print(f"  WARN: could not set {key} in {path}")
        _write_lines(path, lines)


# ---------------------------------------------------------------------------
# Measure directions
# ---------------------------------------------------------------------------


def measure_dirs(bus):
    print("\n=== measuring motor directions (wheels must be free-spinning) ===")
    raw = {}
    for cid in CAN_IDS:
        fwd = spin_one(bus, cid, 200)
        rev = spin_one(bus, cid, -200)
        raw[cid] = (fwd, rev)
        print(f"  {NAMES[cid]}(CAN{cid}): +cur->erpm={fwd:+d}  -cur->erpm={rev:+d}")
    # raw physical: +current direction (sign of erpm_fwd)
    dirs_raw = [1 if raw[c][0] > 0 else -1 for c in CAN_IDS]
    # sign convention: OID +current = CCW on this robot -> negate
    dirs_neg = [-d for d in dirs_raw]
    # right side reverse-mounted -> flip FR/RR
    dirs_final = []
    for i, cid in enumerate(CAN_IDS):
        d = dirs_neg[i]
        if cid in RIGHT_IDS:
            d = -d
        dirs_final.append(d)
    print(f"\n  raw physical       (FL,RL,FR,RR) = {dirs_raw}")
    print(f"  after sign+mount   (FL,RL,FR,RR) = {dirs_final}")
    return dirs_final


# ---------------------------------------------------------------------------
# Sub-commands
# ---------------------------------------------------------------------------


def cmd_show():
    for path in YAML_PATHS:
        if not os.path.exists(path):
            continue
        lines = _load_lines(path)
        print(f"\n{path}:")
        for key in ("pole_pairs", "gear_ratio", "wheel_radius", "track_width",
                    "motor_ids", "motor_dirs", "mower_id"):
            v = get_value(lines, key)
            print(f"  {key:14s} = {v}")


def cmd_basics():
    print("\n=== base parameter check ===")
    print("These are motor/hardware specs that differ between robot builds.")
    lines = None
    for path in YAML_PATHS:
        if os.path.exists(path):
            lines = _load_lines(path)
            break
    if lines is None:
        print("  no yaml found — abort")
        return

    cur = {k: get_value(lines, k) for k in
           ("pole_pairs", "gear_ratio", "wheel_radius", "track_width",
            "motor_ids", "motor_dirs", "mower_id")}

    print("Current values:")
    for k, v in cur.items():
        print(f"  {k:14s} = {v}")

    q = input("\nUpdate any of these? [y/N] ").strip().lower()
    if q not in ("y", "yes"):
        print("  skipped")
        return

    # (key, description, is_number)
    prompts = [
        ("pole_pairs", "motor pole pairs (e.g. 10)", True),
        ("gear_ratio", "reduction gear ratio (e.g. 5.2)", True),
        ("wheel_radius", "wheel radius meters (e.g. 0.10)", True),
        ("track_width", "track width meters (e.g. 0.45)", True),
        ("motor_ids", "motor CAN IDs [FL,RL,FR,RR] (e.g. [2, 1, 4, 3])", False),
        ("motor_dirs", "motor directions [FL,RL,FR,RR] (e.g. [-1, -1, 1, 1])", False),
        ("mower_id", "mower CAN ID (e.g. 5)", True),
    ]
    updates = {}
    for key, desc, is_num in prompts:
        raw = input(f"  {key} ({desc}) [{cur[key]}]: ").strip()
        if not raw:
            continue
        if is_num:
            try:
                val = float(raw)
            except ValueError:
                print(f"    invalid number — keeping {cur[key]}")
                continue
            if key in ("pole_pairs", "mower_id") or val == int(val):
                val = int(val)
        else:
            # list form: keep as string, must look like [a, b, c, d]
            if not raw.startswith("[") or not raw.endswith("]"):
                print(f"    invalid list — keeping {cur[key]}")
                continue
            val = raw
        updates[key] = (val, "set by cal_dirs --basics")

    if not updates:
        print("  nothing changed")
        return

    print("\nAbout to write:")
    for k, (v, c) in updates.items():
        print(f"  {k} = {v}")
    ok = input("Confirm? [y/N] ").strip().lower()
    if ok not in ("y", "yes"):
        print("  aborted")
        return

    for path in YAML_PATHS:
        backup(path)
    apply_base_params(updates)
    print("\nDone. Restart chassis for changes to take effect.")


def cmd_apply():
    print("\n=== measure directions and write motor_dirs ===")
    bus = can.interface.Bus(channel="can0", bustype="socketcan")
    dirs = measure_dirs(bus)
    bus.shutdown()

    for path in YAML_PATHS:
        backup(path)
    apply_base_params({"motor_dirs": (str(dirs),
                                      "auto-calibrated by cal_dirs --apply")})
    print("\nWrote motor_dirs. On-ground verification:")
    print("  restart chassis, then: python3 drivetest.py forward 1")
    print("  if direction is wrong: python3 cal_dirs.py --rollback")


def cmd_measure():
    print("\n=== measure directions (print only) ===")
    bus = can.interface.Bus(channel="can0", bustype="socketcan")
    measure_dirs(bus)
    bus.shutdown()
    print("\n(not written. Use --apply to write.)")


def cmd_rollback():
    for path in YAML_PATHS:
        rollback(path)


def main():
    args = sys.argv[1:]
    if "--show" in args:
        cmd_show()
        return
    if "--basics" in args:
        cmd_basics()
        return
    if "--rollback" in args:
        cmd_rollback()
        return
    if "--apply" in args:
        if "--yes" not in args:
            r = input("Motors will spin briefly. Continue? [y/N] ").strip().lower()
            if r not in ("y", "yes"):
                print("aborted")
                return
        cmd_apply()
        return
    # default: measure only
    if "--yes" not in args:
        r = input("Motors will spin briefly. Continue? [y/N] ").strip().lower()
        if r not in ("y", "yes"):
            print("aborted")
            return
    cmd_measure()


if __name__ == "__main__":
    main()
