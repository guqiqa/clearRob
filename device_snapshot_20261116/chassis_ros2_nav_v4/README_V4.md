# Chassis ROS 2 v4 (SafeMotionCommand safety exit)

独立于 v3（`/root/chassis_ros2_nav_v3`）：v3 二进制、源码、配置全部保留，v4 不设默认服务。

## Changes vs v3

- Subscribe `/safety/safe_motion_command` (`navigation_interfaces/msg/SafeMotionCommand`) and validate it in-process:
  monotonic `sequence`, `ttl_s` via `std::chrono::steady_clock`, `enable`/`stop_required`/`emergency_stop`,
  autonomous mode, CAN/feedback health, linear/angular limits, reject `linear_x=0 && angular_z!=0`.
- Publish `/chassis/state` (`navigation_interfaces/msg/ChassisState`) with control_mode/motion_allowed/
  emergency_stop/can_healthy/feedback_valid/linear_velocity/angular_velocity/wheel_rpm/safety_state/
  fault_reason/command_age_s.
- Services `/chassis/set_autonomous_mode` plus legacy alias `/set_autonomous_mode`.
- Legacy `/cmd_vel` subscription kept but disabled by default (`ROS_Legacy_CmdVel_Enable=0`).
- Fault latches: safe-command timeout, estop, CAN fault, feedback loss; cleared by disabling/enabling autonomous mode.
- `--dry-run` mode: no CAN/SBUS/IMU access, ROS interfaces + validation only (no-motor checks).

## Build

```bash
source /root/nav_ws/env.sh
export CMAKE_PREFIX_PATH=/root/nav_ws/install/navigation_interfaces:/opt/ros/humble:$CMAKE_PREFIX_PATH
cd /root/chassis_ros2_nav_v4/build && cmake ../src -DCMAKE_BUILD_TYPE=Release && make -j4
```

## Run (bench only)

```bash
bash /root/ros2_bridge/start_v4_bench.sh
bash /root/ros2_bridge/run_mission_bench_v4.sh FORWARD -p distance_m:=0.5
bash /root/ros2_bridge/stop_v4_bench.sh
```

禁止与 Python `wheel_speed_node`/`hardware_adapter` 同时运行（CAN 只能有一个控制者）。

## Test-only fault injection (bench only)

```bash
# keep CAN open, force speed feedback invalid for 4 s starting 8 s after launch
./RemoteControl_WheelPID_V2 config_emu.xml \
  --inject-feedback-loss-after-s 8 --inject-feedback-loss-duration-s 4
```

- Runtime-only arguments; never add them to the systemd/default startup path.
- Expected: `LATCHED_FAULT / CHASSIS_FEEDBACK_LOST` within one control cycle;
  latch cleared only by disabling and re-enabling autonomous mode; motion resumes afterwards.

## ChassisState sign convention

`linear_velocity`/`angular_velocity` follow ROS convention (forward +x, left turn +yaw),
matching `/odom` twist; wheel RPM keep the chassis-internal physical sign.

## Effective calibration (2026-10-24)

- Config uses `Pole_Pairs=10`, `Gear_Ratio=4.5`, `Wheel_Radius_m=0.0762` (6-inch wheel).
- `K = pp x gr = 45` is an **effective** value derived from single-wheel CAN spin test
  (`erpm=1560` -> 35.0 rpm -> K~44.5-45); it is not the nameplate gear ratio.
- Old config (5.2 / 0.10) over-read odometry and made the robot physically slow;
  ground tape showed ~1.12 m for a 1.0 m odom command (scale error + ~0.25 m coasting).
- Ground two-point calibration (BACKWARD 1.0/2.0 m with tape) is the final validation;
  `Track_Width_m=0.45` remains uncalibrated until turning tests.
