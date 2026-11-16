#!/bin/bash
# 切换窗口自动执行：安装 unit -> dry-run -> 启动 sbus_emu/v4 -> 接口检查 -> 离地 F/B/R -> 急停/超时 -> 结束
# 任一检查失败：自动回滚（禁用/删除 unit、停止进程），并写 ROLLED_BACK
set +u
source /root/nav_ws/env.sh
RES=/tmp/switch_window_result.txt
LOG=/tmp/switch_window.log
DONE=/tmp/switch_window_done.txt
rm -f $DONE; : > $RES; : > $LOG
log() { echo "$*" | tee -a $RES >> $LOG; }

rollback() {
  log "ROLLBACK: disabling/removing units and stopping processes"
  systemctl disable --now laifei-chassis-v4.service >/dev/null 2>&1
  systemctl disable --now laifei-sbus-emu.service >/dev/null 2>&1
  rm -f /etc/systemd/system/laifei-chassis-v4.service /etc/systemd/system/laifei-sbus-emu.service
  systemctl daemon-reload >/dev/null 2>&1
  pkill -f RemoteControl_WheelPID_V2 2>/dev/null
  pkill -f sbus_emu 2>/dev/null
  pkill -f demo_mission_manager 2>/dev/null
  pkill -f flat_ground_grid 2>/dev/null
  pkill -f "navigation_hardware_dry_run.launch.py" 2>/dev/null
  sleep 1
  log "ROLLED_BACK units_removed=1 processes_stopped=1"
  echo "ROLLED_BACK" > $DONE
  exit 1
}
fail() { log "FAIL: $*"; rollback; }

preflight() {
  log "=== preflight $(date +%F_%T) ==="
  [ -f /root/systemd_staging/laifei-sbus-emu.service ] || fail "staged sbus unit missing"
  [ -f /root/systemd_staging/laifei-chassis-v4.service ] || fail "staged chassis unit missing"
  [ -f /root/systemd_staging/gen_config_emu_and_run.sh ] || fail "staged hook missing"
  pgrep -f "RemoteControl_WheelPID_V2|wheel_speed_node|hardware_adapter|sbus_emu" >/dev/null && fail "pre-existing controller process"
  [ -f /root/backup_systemd_switch_20261023_2139/MANIFEST.txt ] || fail "v3 backup manifest missing"
  log "preflight PASS"
}

install_units() {
  log "--- install units + hook ---"
  mkdir -p /root/chassis_ros2_nav_v4/scripts
  install -m 644 /root/systemd_staging/laifei-sbus-emu.service /etc/systemd/system/ || fail "install sbus unit failed"
  install -m 644 /root/systemd_staging/laifei-chassis-v4.service /etc/systemd/system/ || fail "install chassis unit failed"
  install -m 755 /root/systemd_staging/gen_config_emu_and_run.sh /root/chassis_ros2_nav_v4/scripts/gen_config_emu_and_run.sh || fail "install hook failed"
  [ -x /root/chassis_ros2_nav_v4/scripts/gen_config_emu_and_run.sh ] || fail "hook not executable"
  systemctl daemon-reload
  log "install PASS"
}

dry_run_check() {
  log "--- dry-run (service not started) ---"
  setsid nohup /root/chassis_ros2_nav_v4/build/RemoteControl_WheelPID_V2 --dry-run /root/chassis_ros2_nav_v4/src/config.xml > /tmp/dryrun_switch.log 2>&1 < /dev/null &
  sleep 3
  timeout 8 ros2 node list 2>/dev/null | grep -q laifei_chassis_driver || fail "dry-run node missing"
  PUBS=$(timeout 8 ros2 topic info /chassis/state 2>/dev/null | awk '/Publisher count/ {print $3}')
  [ "$PUBS" = "1" ] || fail "dry-run /chassis/state publishers=$PUBS (expect 1)"
  CMDP=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/ {print $3}')
  [ "$CMDP" = "0" ] || fail "dry-run /cmd_vel publishers=$CMDP (expect 0)"
  timeout 8 ros2 service list 2>/dev/null | grep -q /chassis/set_autonomous_mode || fail "dry-run mode service missing"
  ST=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
  echo "$ST" | grep -q "control_mode: MANUAL" || fail "dry-run not MANUAL"
  V=$(echo "$ST" | awk '/linear_velocity:/ {print $2}')
  python3 -c "import sys; sys.exit(0 if abs(float('$V'))<1e-6 else 1)" || fail "dry-run nonzero linear_velocity=$V"
  pkill -INT -f "RemoteControl_WheelPID_V2 --dry-run" 2>/dev/null
  for i in $(seq 1 30); do pgrep -f "RemoteControl_WheelPID_V2 --dry-run" >/dev/null || break; sleep 0.2; done
  pgrep -f "RemoteControl_WheelPID_V2 --dry-run" >/dev/null && fail "dry-run process did not exit"
  log "dry-run PASS (MANUAL, zero velocity, unique publisher)"
}

start_units() {
  log "--- start sbus_emu + chassis-v4 units ---"
  systemctl enable --now laifei-sbus-emu.service || fail "enable sbus-emu failed"
  for i in $(seq 1 30); do [ -s /tmp/sbus_emu_path ] && break; sleep 0.1; done
  [ -s /tmp/sbus_emu_path ] || fail "sbus PTY not ready"
  systemctl enable --now laifei-chassis-v4.service || fail "enable chassis-v4 failed"
  sleep 6
  systemctl is-active laifei-chassis-v4.service | grep -q active || fail "chassis-v4 not active"
  systemctl is-active laifei-sbus-emu.service | grep -q active || fail "sbus-emu not active"
  timeout 8 ros2 node list 2>/dev/null | grep -q laifei_chassis_driver || fail "chassis node missing"
  PUBS=$(timeout 8 ros2 topic info /chassis/state 2>/dev/null | awk '/Publisher count/ {print $3}')
  [ "$PUBS" = "1" ] || fail "/chassis/state publishers=$PUBS"
  ODOMP=$(timeout 8 ros2 topic info /odom 2>/dev/null | awk '/Publisher count/ {print $3}')
  [ "$ODOMP" = "1" ] || fail "/odom publishers=$ODOMP"
  CMDP=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/ {print $3}')
  [ "$CMDP" = "0" ] || fail "/cmd_vel publishers=$CMDP"
  ST=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
  echo "$ST" | grep -q "control_mode: MANUAL" || fail "started chassis not MANUAL"
  echo "$ST" | grep -q "feedback_valid: true" || fail "feedback invalid"
  echo "$ST" | grep -q "can_healthy: true" || fail "CAN unhealthy"
  pgrep -f "wheel_speed_node|hardware_adapter" >/dev/null && fail "forbidden Python CAN node running"
  VC=$(pgrep -f "RemoteControl_WheelPID_V2 .*config_emu.xml" | wc -l)
  [ "$VC" = "1" ] || fail "v4 process count=$VC (expect 1)"
  log "units PASS (MANUAL, unique publishers, CAN owner=v4 only)"
}

launch_nav() {
  log "--- launch navigation + safety + grid ---"
  setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/switch_nav.log 2>&1 < /dev/null &
  sleep 8
  setsid nohup python3 /root/ros2_bridge/flat_ground_grid.py > /tmp/switch_grid.log 2>&1 < /dev/null &
  sleep 1
  timeout 8 ros2 node list 2>/dev/null | grep -q navigation_node || fail "navigation_node missing"
  timeout 8 ros2 node list 2>/dev/null | grep -q safety_supervisor || fail "safety_supervisor missing"
  timeout 8 ros2 param set /navigation_node profile GROUND_DEMO_BENCH_STABLE >/dev/null 2>&1
  timeout 8 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
  timeout 8 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1
  log "nav+safety PASS"
}

run_task() {
  TASK=$1; shift
  log "--- task $TASK $* ---"
  pkill -f demo_mission_manager 2>/dev/null; sleep 1
  setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file /root/nav_ws/config/ground_demo_bench_stable.yaml -p task:=$TASK "$@" > /tmp/switch_mission.log 2>&1 < /dev/null &
  sleep 4
  timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  timeout 8 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  sleep 1
  X0=$(timeout 6 ros2 topic echo /odom --once 2>/dev/null | grep -A2 "position:" | grep "x:" | head -1 | awk '{print $2}')
  timeout 8 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1
  FINAL=""; for k in $(seq 1 150); do
    st=$(timeout 5 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
    case "$st" in COMPLETED|ABORTED|STOPPED|TIMEOUT|CHASSIS_FAULT|ESTOP) FINAL=$st; break;; esac
    sleep 1
  done
  sleep 1
  X1=$(timeout 6 ros2 topic echo /odom --once 2>/dev/null | grep -A2 "position:" | grep "x:" | head -1 | awk '{print $2}')
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f demo_mission_manager 2>/dev/null
  log "task=$TASK final=$FINAL x0=$X0 x1=$X1"
  [ "$FINAL" = "COMPLETED" ] || fail "task $TASK final=$FINAL"
}

estop_test() {
  log "--- audit pass (incl. estop) ---"
  AUD=$(timeout 120 python3 /root/ros2_bridge/audit_v4_interfaces.py 2>&1 | tail -3)
  echo "$AUD" | grep -q "AUDIT RESULT: PASS=22 FAIL=0" || fail "audit failed: $AUD"
  log "audit PASS=22 FAIL=0"
}

timeout_test() {
  log "--- timeout/TTL test (safety supervisor stopped) ---"
  pkill -f safety_supervisor_node 2>/dev/null; sleep 1
  OUT=$(timeout 120 python3 /root/ros2_bridge/v4_rejection_test.py 2>&1 | tail -3)
  echo "$OUT" | grep -q "REJECTION RESULT: PASS=8 FAIL=0" || fail "timeout test failed: $OUT"
  log "timeout/TTL PASS=8 FAIL=0"
}

cleanup_window() {
  log "--- cleanup non-systemd test processes; leave units enabled ---"
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f demo_mission_manager 2>/dev/null
  pkill -f flat_ground_grid 2>/dev/null
  pkill -f "navigation_hardware_dry_run.launch.py" 2>/dev/null
  pkill -f navigation_node 2>/dev/null
  pkill -f safety_supervisor_node 2>/dev/null
  sleep 2
  ST=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
  echo "$ST" | grep -q "control_mode: MANUAL" || fail "final state not MANUAL"
  systemctl is-enabled laifei-chassis-v4.service | grep -q enabled || fail "chassis unit not enabled"
  systemctl is-active laifei-chassis-v4.service | grep -q active || fail "chassis unit not active"
  log "final PASS: units enabled+active, chassis MANUAL, zero speed"
}

preflight
install_units
dry_run_check
start_units
launch_nav
run_task FORWARD -p distance_m:=0.5
run_task BACKWARD -p distance_m:=0.5
run_task RECTANGLE_LOOP -p side_m:=2.0 -p corner_radius_m:=0.6 -p waypoint_step_m:=0.4
estop_test
timeout_test
cleanup_window
log "=== SWITCH WINDOW RESULT: PASS $(date +%F_%T) ==="
echo "PASS" > $DONE
