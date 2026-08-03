#!/usr/bin/env python3
"""
wheel_loop_sim.py — Numerical simulation of the WheelPID V2 port.

Drives the EXACT WheelSpeedController shipped in cleaning_robot_common with a
simulated motor plant, to validate control behavior without hardware:

  1. speed  mode   — 0→0.3 m/s step: slew ramping, driver closed-loop follow
  2. current mode  — same step: startup current ramp → PI convergence
  3. current stall — wheel physically blocked: startup ramp → safety latch
  4. speed mode feedback loss — CAN feedback vanishes: wheels stop, recover
  5. current overspeed — external torque pushes wheel past 180 rpm: latch

Usage:  python3 sim/wheel_loop_sim.py [--plot]
"""
import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src",
                                "cleaning_robot_common"))

from cleaning_robot_common.config import WHEEL_SPEED_PARAMS  # noqa: E402
from cleaning_robot_common.wheel_speed_loop import WheelSpeedController  # noqa: E402

DT = 0.05  # control tick, matches control_interval_ms=50


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MotorModel:
    """4-wheel plant. Two drive modes mirroring the two controller modes.

    speed   : driver-internal closed loop — ERPM ramps to the requested
              target at the configured driver accel/decel, with a small
              first-order lag (tau) to represent finite torque authority.
    current : current (A) → torque; Coulomb + viscous drag; rpm integrates.
    """

    def __init__(self, ctrl, mode):
        self.ctrl = ctrl
        self.mode = mode
        self.cids = ctrl.cids
        self.erpm = {cid: 0 for cid in self.cids}
        self.rpm = {cid: 0.0 for cid in self.cids}
        # driver closed-loop params (erpm/s), same values sent via OID 0x0a/0x10
        self.driver_accel = ctrl.accel_rpm_s * ctrl.pole_pairs * ctrl.gear_ratio
        self.driver_decel = ctrl.decel_rpm_s * ctrl.pole_pairs * ctrl.gear_ratio
        self.tau = 0.08
        # current-mode plant — calibrated so 1.7 A (170 x10mA) startup current
        # lifts the wheel from 0 to ~20 rpm in about 1.2 s (5.2:1 gearbox
        # inertia). The WheelPID V2 PI gains are stable on the real hardware
        # with this magnitude of acceleration authority.
        self.coulomb_A = 0.30
        self.visc_A_per_rpm = 0.016
        self.k_acc = 12.0   # rpm/s per A of excess torque
        # scenario hooks
        self.blocked = set()       # cids mechanically blocked (stall)
        self.force_rpm = {}        # cid → externally driven rpm (overspeed)

    def apply(self, cmds, dt):
        for cid, kind, value in cmds:
            if cid in self.force_rpm:
                self.rpm[cid] = self.force_rpm[cid]
                self.erpm[cid] = self.ctrl.wheel_rpm_to_erpm(cid, self.rpm[cid])
            elif self.mode == "speed":
                self._speed(cid, value, dt)
            else:
                self._current(cid, value, dt)

    def _speed(self, cid, target_erpm, dt):
        if cid in self.blocked:
            self.rpm[cid] = 0.0
            self.erpm[cid] = 0
            return
        cur = self.erpm[cid]
        err = target_erpm - cur
        max_rate = self.driver_accel if err >= 0 else self.driver_decel
        rate = clamp(err / self.tau, -max_rate, max_rate)
        cur += rate * dt
        self.erpm[cid] = int(round(cur))
        self.rpm[cid] = self.ctrl.erpm_to_wheel_rpm(cid, self.erpm[cid])

    def _current(self, cid, value, dt):
        if cid in self.blocked:
            self.rpm[cid] = 0.0
            self.erpm[cid] = 0
            return
        I_amp = value / 100.0
        rpm = self.rpm[cid]
        if rpm == 0.0 and abs(I_amp) < self.coulomb_A:
            return  # static friction not overcome
        sgn = 1.0 if rpm >= 0 else -1.0
        net = I_amp - sgn * (self.coulomb_A + self.visc_A_per_rpm * abs(rpm))
        self.rpm[cid] += net * self.k_acc * dt
        self.erpm[cid] = self.ctrl.wheel_rpm_to_erpm(cid, self.rpm[cid])

    def measured_rpm(self):
        return {cid: self.rpm[cid] for cid in self.cids}


class Scenario:
    """Runs a controller + motor model loop, records a time series."""

    def __init__(self, name, mode, total_time, overrides=None, hooks=None):
        params = dict(WHEEL_SPEED_PARAMS)
        params["wheel_control_mode"] = mode
        if overrides:
            params.update(overrides)
        self.name = name
        self.ctrl = WheelSpeedController(params)
        self.motor = MotorModel(self.ctrl, mode)
        self.total = total_time
        self.hooks = hooks or {}
        self.t = 0.0
        self.rows = []
        self.cid0 = self.ctrl.cids[0]

    def target(self):
        fn = self.hooks.get("target")
        return fn(self.t) if fn else self._default_target(self.t)

    def _default_target(self, t):
        # default: 0.3 m/s forward from 0.5 s, stop at 4.0 s
        if t < 0.5 or t >= 4.0:
            return {cid: 0.0 for cid in self.ctrl.cids}
        return {cid: self.ctrl.mps_to_wheel_rpm(0.3) for cid in self.ctrl.cids}

    def fresh(self):
        fn = self.hooks.get("fresh")
        return fn(self.t) if fn else {cid: True for cid in self.ctrl.cids}

    def run(self):
        n = int(round(self.total / DT))
        for _ in range(n):
            if self.hooks.get("perturb"):
                self.hooks["perturb"](self.t, self.motor, self.ctrl)
            targets = self.target()
            measured = self.motor.measured_rpm()
            fr = self.fresh()
            cmds = self.ctrl.step(targets, measured, fr)
            self.motor.apply(cmds, DT)
            rpm = self.ctrl.erpm_to_wheel_rpm(
                self.cid0, self.motor.erpm[self.cid0])
            self.rows.append({
                "t": round(self.t, 3),
                "target_rpm": targets.get(self.cid0, 0.0),
                "actual_rpm": rpm,
                "cmd": cmds[0][2],
                "cmd_kind": cmds[0][1],
                "safety": self.ctrl.safety_reason if self.ctrl.safety_latched else "",
            })
            self.t += DT
        return self.rows

    def metrics(self):
        r = self.rows
        cid = self.cid0
        # truncate to before a safety latch so latch-stop rows don't skew metrics
        latch_i = next((i for i, row in enumerate(r) if row["safety"]), len(r))
        pre = r[:latch_i]
        moving = [row for row in pre if row["target_rpm"] != 0]
        settled_t = None
        overshoot = 0.0
        if moving:
            want = moving[-1]["target_rpm"]
            # first time |actual-want| <= 2 rpm stays within band for 1 s (20 ticks)
            ok_run = 0
            for row in moving:
                err = abs(abs(row["actual_rpm"]) - want)
                if err <= 2.0:
                    ok_run += 1
                    if ok_run >= 20:
                        settled_t = row["t"]
                        break
                else:
                    ok_run = 0
            overshoot = max((abs(row["actual_rpm"]) - want) / want * 100.0
                            for row in moving)
        # safety latch time
        latch_t = next((row["t"] for row in r if row["safety"]), None)
        final_err = 0.0
        if moving and not latch_t:
            want = moving[-1]["target_rpm"]
            final_err = abs(r[-1]["actual_rpm"]) - want
        return {
            "settle_s": settled_t,
            "overshoot_pct": overshoot,
            "safety_latch_s": latch_t,
            "safety_reason": next((row["safety"] for row in r if row["safety"]), ""),
            "final_err_rpm": final_err,
        }


SCENARIOS = [
    Scenario("1.speed_step", "speed", total_time=6.0),
    Scenario("2.current_step", "current", total_time=6.0),
]

# 3. current stall: wheel blocked from the start, command a small forward target
def _stall(target):
    return {cid: 7.0 for cid in [2, 1, 4, 3]}  # small target, stays in startup ramp

def _stall_block(t, motor, ctrl):
    motor.blocked.update(ctrl.cids)

scen3 = Scenario("3.current_stall", "current", total_time=8.0,
                 hooks={"target": _stall, "perturb": _stall_block})

# 4. speed feedback loss 1.0–3.0 s, keep commanding until 5.5 s
def _fb_target(t):
    if t < 0.5 or t >= 5.5:
        return {cid: 0.0 for cid in [2, 1, 4, 3]}
    return {cid: 35.81 for cid in [2, 1, 4, 3]}

def _fb_loss(t):
    return {cid: (t < 1.0 or t >= 3.0) for cid in [2, 1, 4, 3]}

scen4 = Scenario("4.speed_fb_loss", "speed", total_time=7.0,
                 hooks={"target": _fb_target, "fresh": _fb_loss})

# 5. current overspeed: externally push wheel past 180 rpm at t=1.0
def _ovs_perturb(t, motor, ctrl):
    if t >= 1.0:
        motor.force_rpm[2] = 210.0

scen5 = Scenario("5.current_overspeed", "current", total_time=5.0,
                 hooks={"perturb": _ovs_perturb})

ALL = [scen1 := SCENARIOS[0], scen2 := SCENARIOS[1], scen3, scen4, scen5]


def write_csv(scen):
    out = os.path.join(os.path.dirname(__file__), f"sim_{scen.name}.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t", "target_rpm", "actual_rpm", "cmd", "cmd_kind", "safety"])
        w.writeheader()
        for row in scen.rows:
            w.writerow(row)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    print("=" * 78)
    print("WheelPID V2 port — numerical simulation")
    print(f"motor: 10 pole pairs, 5.2 gear, 0.08 m wheel, 0.53 m track")
    print(f"mode default = {WHEEL_SPEED_PARAMS['wheel_control_mode']}, "
          f"KP={WHEEL_SPEED_PARAMS['wheel_speed_kp']}, "
          f"KI={WHEEL_SPEED_PARAMS['wheel_speed_ki']}, "
          f"FF={WHEEL_SPEED_PARAMS['wheel_feedforward_current']}x10mA")
    print("=" * 78)

    results = {}
    for scen in ALL:
        scen.run()
        m = scen.metrics()
        results[scen.name] = m
        print(f"\n[{scen.name}]  mode={scen.ctrl.mode}")
        print(f"  settle to ±2rpm @ {m['settle_s']} s"
              f"  overshoot {m['overshoot_pct']:.1f}%"
              f"  safety_latch {m['safety_latch_s']} s"
              f"  ({m['safety_reason']})")
        # print a compact table every 0.5 s
        last_t = -1
        for row in scen.rows:
            ti = round(row["t"], 1)
            if ti != last_t and round(ti * 10) % 5 == 0:
                last_t = ti
                print(f"    t={ti:4.1f}  target={row['target_rpm']:6.2f}  "
                      f"actual={row['actual_rpm']:7.2f}  cmd={row['cmd']:6d} "
                      f"{row['cmd_kind']:7s} {row['safety']}")
        csv_path = write_csv(scen)
        print(f"  -> {csv_path}")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(len(ALL), 1, figsize=(10, 14), sharex=False)
            for ax, scen in zip(axes, ALL):
                ts = [r["t"] for r in scen.rows]
                tr = [r["target_rpm"] for r in scen.rows]
                ar = [r["actual_rpm"] for r in scen.rows]
                cm = [r["cmd"] for r in scen.rows]
                ax.plot(ts, tr, "--", color="gray", label="target")
                ax.plot(ts, ar, label="actual rpm")
                ax.set_title(scen.name + f" (mode={scen.ctrl.mode})")
                ax.set_ylabel("rpm")
                ax.legend(loc="upper right", fontsize=8)
                ax.grid(alpha=0.3)
                ax2 = ax.twinx()
                ax2.plot(ts, cm, alpha=0.4, label="cmd")
                ax2.set_ylabel("cmd (10mA or erpm)")
            plt.tight_layout()
            png = os.path.join(os.path.dirname(__file__), "sim_plot.png")
            plt.savefig(png, dpi=110)
            print(f"\nPlot saved: {png}")
        except Exception as e:
            print(f"\n(plot unavailable: {e})")

    print("\n" + "=" * 78)
    print("SUMMARY")
    print("=" * 78)
    for name, m in results.items():
        print(f"  {name:22s} settle={m['settle_s']}s  overshoot={m['overshoot_pct']:5.1f}%"
              f"  latch={m['safety_latch_s']}s")


if __name__ == "__main__":
    main()
