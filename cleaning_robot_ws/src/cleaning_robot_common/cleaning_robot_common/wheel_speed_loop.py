#!/usr/bin/env python3
"""
cleaning_robot_common/wheel_speed_loop.py — Pure wheel speed-loop controller.

This is the WheelPID V2 control scheme, extracted into a dependency-free
module so the exact code shipped on the robot can be unit- and
simulation-tested without ROS2. The chassis_driver node instantiates this
class and feeds it measured wheel RPM / fresh flags; it returns CAN commands.

Two modes:
  "speed"   — driver internal closed-loop (OID cmd 0x02). Software only does
              slew-rate ramping of the target wheel RPM.
  "current" — software PI + feed-forward on top of OID current control (0x01),
              with a startup current ramp and stall re-arm.

All RPM values are PHYSICAL wheel RPM (post gearbox).
ERPM = wheel_rpm * pole_pairs * gear_ratio * motor_dir.
"""

import math
from typing import Dict, List, Optional, Tuple


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


# Motor layout order (FL, RL, FR, RR) — matches WheelPID V2.
MOTOR_ORDER = [2, 1, 4, 3]


class WheelSpeedController:
    """Per-wheel speed controller. One instance holds all 4 wheels' state.

    The caller drives the loop at `control_interval_ms`. Each call produces
    one CAN command per wheel: ("current", mA_10) or ("speed", erpm).
    """

    def __init__(self, params: dict):
        # Required params (see config.py WHEEL_SPEED_PARAMS for defaults)
        self.pole_pairs = int(params["pole_pairs"])
        self.gear_ratio = float(params["gear_ratio"])
        self.wheel_radius_m = float(params["wheel_radius"])
        self.mode = params.get("wheel_control_mode", "speed")
        self.motor_dirs = dict(params.get("motor_dirs", {}))
        # per-motor speed gain (target-side scaling), keyed by cid. Applied so
        # all wheels converge to the same physical speed (straight-line drift).
        self.motor_gains = {cid: float(g) for cid, g in
                            params.get("motor_gains", {}).items()}

        self.accel_rpm_s = int(params.get("speed_accel_rpm_s", 120))
        self.decel_rpm_s = int(params.get("speed_decel_rpm_s", 140))
        self.wheel_max_current = int(params.get("wheel_max_current", 500))
        self.kp = float(params.get("wheel_speed_kp", 4.5))
        self.ki = float(params.get("wheel_speed_ki", 0.35))
        self.ff_current = int(params.get("wheel_feedforward_current", 90))
        self.intg_max_current = int(params.get("wheel_integral_max_current", 120))
        self.start_init_current = int(params.get("wheel_start_initial_current", 170))
        self.start_step_current = int(params.get("wheel_start_step_current", 20))
        self.start_step_ms = int(params.get("wheel_start_step_ms", 50))
        self.start_max_current = int(params.get("wheel_start_max_current", 500))
        self.start_threshold_rpm = int(params.get("wheel_start_threshold_rpm", 20))
        self.start_confirm_samples = int(params.get("wheel_start_confirm_samples", 4))
        self.stall_threshold_rpm = int(params.get("wheel_stall_threshold_rpm", 5))
        self.stall_confirm_samples = int(params.get("wheel_stall_confirm_samples", 2))
        self.start_timeout_s = float(params.get("wheel_start_timeout_ms", 7000)) / 1000.0
        self.overspeed_rpm = int(params.get("wheel_overspeed_rpm", 180))
        self.feedback_timeout_s = float(params.get("speed_feedback_timeout_ms", 1000)) / 1000.0
        self.control_interval_s = float(params.get("control_interval_ms", 50)) / 1000.0

        self.cids: List[int] = list(self.motor_dirs.keys()) if self.motor_dirs else list(MOTOR_ORDER)
        self.reset()

    # ---- state ----

    def reset(self) -> None:
        self.ramped = {cid: 0.0 for cid in self.cids}
        self.wheel_integral = {cid: 0.0 for cid in self.cids}
        self.started = {cid: False for cid in self.cids}
        self.start_elapsed_s = {cid: 0.0 for cid in self.cids}
        self.start_confirm = {cid: 0 for cid in self.cids}
        self.stall_count = {cid: 0 for cid in self.cids}
        self.safety_latched = False
        self.safety_reason = ""
        self.safety_reported = False

    # ---- conversions ----

    def mps_to_wheel_rpm(self, v: float) -> float:
        return v * 60.0 / (2.0 * math.pi * self.wheel_radius_m)

    def wheel_rpm_to_erpm(self, cid: int, rpm: float) -> int:
        return int(round(rpm * self.pole_pairs * self.gear_ratio * self.motor_dirs.get(cid, 1)))

    def erpm_to_wheel_rpm(self, cid: int, erpm: int) -> float:
        return erpm * self.motor_dirs.get(cid, 1) / (self.pole_pairs * self.gear_ratio)

    def wheel_rpm_to_mps(self, rpm: float) -> float:
        return rpm * 2.0 * math.pi * self.wheel_radius_m / 60.0

    # ---- main entry ----

    def step(self, targets: Dict[int, float], measured: Dict[int, float],
             fresh: Dict[int, bool]) -> List[Tuple[int, str, int]]:
        """One control tick.

        targets : {cid: desired wheel RPM}
        measured : {cid: measured wheel RPM}
        fresh   : {cid: feedback received within timeout}

        Returns list of (cid, kind, value):
          kind = "speed"    → value is target ERPM (driver closed-loop)
          kind = "current"  → value is current in 10 mA units
          kind = "none"     → no command this tick
        """
        if self.mode == "speed":
            return self._speed_step(targets, measured, fresh)
        return self._current_step(targets, measured, fresh)

    def _apply_gain(self, targets: Dict[int, float]) -> Dict[int, float]:
        """Scale each wheel's target RPM by its per-motor gain. Slow wheels get
        a higher command so all four physically converge."""
        if not self.motor_gains:
            return targets
        out = {}
        for cid, rpm in targets.items():
            g = self.motor_gains.get(cid, 1.0)
            out[cid] = rpm * g
        return out

    # ---- speed mode ----

    def _speed_step(self, targets, measured, fresh):
        cmds = []
        targets = self._apply_gain(targets)
        self._check_overspeed(measured)
        if self.safety_latched:
            cmds = [(cid, "speed", 0) for cid in self.cids]
            return cmds
        for cid in self.cids:
            target = targets.get(cid, 0.0)
            ramp = self.ramped[cid]
            slew = self._slew_step(target, ramp)
            new_ramp = ramp + _clamp(target - ramp, -slew, slew)
            if not fresh.get(cid, False):
                self.ramped[cid] = 0.0
                cmds.append((cid, "speed", 0))
                continue
            self.ramped[cid] = new_ramp
            cmds.append((cid, "speed", self.wheel_rpm_to_erpm(cid, new_ramp)))
        return cmds

    # ---- current mode ----

    def _current_step(self, targets, measured, fresh):
        cmds = []
        targets = self._apply_gain(targets)
        self._check_overspeed(measured)
        if self.safety_latched:
            cmds = [(cid, "current", 0) for cid in self.cids]
            return cmds
        stopped = all(abs(targets.get(cid, 0.0)) < 0.5 for cid in self.cids)
        if stopped:
            self.safety_latched = False
            self.safety_reason = ""
            for cid in self.cids:
                self._reset_wheel(cid)
                cmds.append((cid, "current", 0))
            return cmds
        for cid in self.cids:
            target = targets.get(cid, 0.0)
            ramp = self.ramped[cid]
            slew = self._slew_step(target, ramp)
            new_ramp = ramp + _clamp(target - ramp, -slew, slew)
            if not fresh.get(cid, False):
                self._reset_wheel(cid)
                cmds.append((cid, "current", 0))
                continue
            if self.kp <= 0.0:
                # Legacy open-loop: proportional current, no closed loop
                cur = self._legacy_current(target)
                cmds.append((cid, "current", cur))
                continue
            self.ramped[cid] = new_ramp
            cur = self._current_wheel(cid, target, new_ramp, measured.get(cid, 0.0))
            cmds.append((cid, "current", cur))
        return cmds

    def _slew_step(self, target: float, ramp: float) -> float:
        accel = self.accel_rpm_s if abs(target) >= abs(ramp) else self.decel_rpm_s
        return accel * self.control_interval_s

    def _legacy_current(self, target: float) -> int:
        # Proportional to target speed relative to max velocity. Kept for the
        # no-PI fallback path; not used by the default tuned profile.
        max_rpm = self.mps_to_wheel_rpm(0.5)
        ratio = target / max(max_rpm, 0.01)
        cur = int(ratio * self.wheel_max_current)
        return 0 if abs(cur) < 10 else cur

    def _reset_wheel(self, cid: int) -> None:
        self.ramped[cid] = 0.0
        self.wheel_integral[cid] = 0.0
        self.started[cid] = False
        self.start_elapsed_s[cid] = 0.0
        self.start_confirm[cid] = 0
        self.stall_count[cid] = 0

    def _current_wheel(self, cid: int, target_rpm: float, ramp_rpm: float,
                       measured_rpm: float) -> int:
        if not self.started[cid]:
            start_cmd = _clamp(abs(target_rpm) * 0.67, 2.0, 10.0)
            if abs(target_rpm) < start_cmd:
                self.start_elapsed_s[cid] = 0.0
                self.start_confirm[cid] = 0
                return 0
            self.start_elapsed_s[cid] += self.control_interval_s
            if abs(measured_rpm) >= self.start_threshold_rpm:
                self.start_confirm[cid] += 1
                if self.start_confirm[cid] >= self.start_confirm_samples:
                    self.started[cid] = True
                    self.wheel_integral[cid] = 0.0
                    self.stall_count[cid] = 0
            else:
                self.start_confirm[cid] = 0
            steps = int(self.start_elapsed_s[cid] * 1000.0 / self.start_step_ms)
            cur = min(self.start_max_current,
                      self.start_init_current + steps * self.start_step_current)
            if self.start_elapsed_s[cid] > self.start_timeout_s:
                self.safety_latched = True
                self.safety_reason = f"startup timeout cid={cid}"
                return 0
            return cur * self.motor_dirs.get(cid, 1)
        # Stall re-arm
        if abs(measured_rpm) < self.stall_threshold_rpm:
            self.stall_count[cid] += 1
            if self.stall_count[cid] >= self.stall_confirm_samples:
                self.started[cid] = False
                self.start_elapsed_s[cid] = 0.0
                self.start_confirm[cid] = 0
                self.stall_count[cid] = 0
                self.wheel_integral[cid] = 0.0
        else:
            self.stall_count[cid] = 0
        # PI + feed-forward
        error = ramp_rpm - measured_rpm
        dt = self.control_interval_s
        next_integral = self.wheel_integral[cid] + error * dt
        if self.ki > 0.0:
            lim = self.intg_max_current / self.ki
            next_integral = _clamp(next_integral, -lim, lim)
        else:
            next_integral = 0.0
        ff_scale = _clamp(abs(ramp_rpm) / max(1.0, abs(target_rpm)), 0.0, 1.0)
        feedforward = self.ff_current * ff_scale
        ff = feedforward if ramp_rpm >= 0 else -feedforward
        raw = ff + self.kp * error + self.ki * next_integral
        limited = _clamp(raw, -self.wheel_max_current, self.wheel_max_current)
        if abs(raw) <= self.wheel_max_current:
            self.wheel_integral[cid] = next_integral
        return int(round(limited)) * self.motor_dirs.get(cid, 1)

    # ---- safety ----

    def _check_overspeed(self, measured: Dict[int, float]) -> None:
        for cid in self.cids:
            if abs(measured.get(cid, 0.0)) > self.overspeed_rpm:
                self.safety_latched = True
                self.safety_reason = f"wheel overspeed cid={cid} rpm={measured[cid]:.0f}"

    def stop_all(self) -> List[Tuple[int, str, int]]:
        self.ramped = {cid: 0.0 for cid in self.cids}
        self.wheel_integral = {cid: 0.0 for cid in self.cids}
        kind = "speed" if self.mode == "speed" else "current"
        return [(cid, kind, 0) for cid in self.cids]
