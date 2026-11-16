#!/bin/bash
# v4 长时间空载 + 往复稳定性自动测试（轮子离地，全自动，无人工轮询）
# 阶段：空载 idle 10 min（每 60 s 采样）-> 20 次 FORWARD/BACKWARD 0.5 m 往复
set +u
source /root/nav_ws/env.sh
R=/tmp/stability_report.txt
DONE=/tmp/stability_done.txt
LOG=/tmp/stability_run.log
rm -f $DONE; : > $R
log() { echo "$*" | tee -a $R; }

run_chain() {
  bash /root/ros2_bridge/stop_v4_bench.sh >/dev/null 2>&1
  timeout 120 bash /root/ros2_bridge/start_v4_bench.sh > /tmp/stab_start.log 2>&1
  sleep 1
}

stop_chain() {
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f demo_mission_manager 2>/dev/null
  bash /root/ros2_bridge/stop_v4_bench.sh >/dev/null 2>&1
}

v4_pid() { pgrep -f "RemoteControl_WheelPID_V2 .*config_emu.xml" | head -1; }

odom_x() {
  timeout 6 ros2 topic echo /odom --once 2>/dev/null | grep -A2 "position:" | grep "x:" | head -1 | awk '{print $2}'
}

can_errors() {
  ip -details -statistics link show can0 2>/dev/null | grep -E "bus-error|error-warn|error-pass|restarts|bus-off" | tr "\n" ";"
}

log "=== v4 stability test start $(date +%F_%T) ==="
run_chain
V4PID=$(v4_pid)
log "v4_pid=$V4PID"
if [ -z "$V4PID" ]; then log "FAIL: v4 not running"; echo done > $DONE; exit 1; fi

# ---------- idle phase ----------
log "--- idle phase: 10 min ---"
IDLE_START=$(date +%s)
IDLE_FAIL=0
for i in $(seq 1 10); do
  sleep 60
  RSS=$(ps -o rss= -p "$V4PID" 2>/dev/null | tr -d " ")
  ALIVE=$(kill -0 "$V4PID" 2>/dev/null && echo yes || echo no)
  X=$(odom_x)
  ST=$(timeout 6 ros2 topic echo /chassis/state --once 2>/dev/null)
  FB=$(echo "$ST" | grep "feedback_valid:" | awk '{print $2}')
  CANH=$(echo "$ST" | grep "can_healthy:" | awk '{print $2}')
  FAULT=$(echo "$ST" | grep "fault_reason:" | cut -d" " -f2-)
  CE=$(can_errors)
  log "[idle ${i}] alive=$ALIVE rss_kb=$RSS x=$X feedback=$FB can=$CANH fault=$FAULT can_errs=$CE"
  if [ "$ALIVE" != "yes" ] || [ "$FB" != "true" ] || [ "$CANH" != "true" ]; then IDLE_FAIL=1; fi
done
log "idle_phase: $([ $IDLE_FAIL -eq 0 ] && echo PASS || echo FAIL)"

# ---------- reciprocating phase ----------
log "--- reciprocating phase: 20 cycles ---"
CYC_FAIL=0; CYC_DONE=0
for i in $(seq 1 20); do
  if [ $((i % 2)) -eq 1 ]; then TASK=FORWARD; else TASK=BACKWARD; fi
  pkill -f demo_mission_manager 2>/dev/null
  setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file /root/nav_ws/config/ground_demo_bench_stable.yaml -p task:=$TASK -p distance_m:=0.5 > /tmp/stab_mission.log 2>&1 < /dev/null &
  sleep 4
  timeout 8 ros2 param set /navigation_node profile GROUND_BENCH_012 >/dev/null 2>&1
  timeout 8 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
  timeout 8 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1
  timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  timeout 8 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  sleep 1
  X0=$(odom_x)
  timeout 8 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1
  FINAL=""
  for k in $(seq 1 40); do
    st=$(timeout 5 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
    case "$st" in COMPLETED|ABORTED|STOPPED|TIMEOUT|CHASSIS_FAULT|ESTOP) FINAL=$st; break;; esac
    sleep 1
  done
  sleep 1
  X1=$(odom_x)
  REL=$(python3 -c "print(abs(float(\"$X1\")-float(\"$X0\")))" 2>/dev/null)
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f demo_mission_manager 2>/dev/null
  sleep 1
  ALIVE=$(kill -0 "$V4PID" 2>/dev/null && echo yes || echo no)
  log "[cycle ${i}] task=$TASK final=$FINAL travel=${REL}m alive=$ALIVE x=$X1"
  if [ "$FINAL" = "COMPLETED" ] && [ "$ALIVE" = "yes" ]; then CYC_DONE=$((CYC_DONE+1)); else CYC_FAIL=$((CYC_FAIL+1)); fi
  if [ $((i % 5)) -eq 0 ]; then
    RSS=$(ps -o rss= -p "$V4PID" 2>/dev/null | tr -d " ")
    CE=$(can_errors)
    DMESG=$(dmesg | grep -icE "can0|can: " 2>/dev/null)
    log "[checkpoint ${i}] rss_kb=$RSS can_errs=$CE dmesg_can_lines=$DMESG"
  fi
done

# ---------- final ----------
FINAL=$(pgrep -f "RemoteControl_WheelPID_V2 .*config_emu.xml" >/dev/null && echo running || echo gone)
RSS_END=$(ps -o rss= -p "$V4PID" 2>/dev/null | tr -d " ")
CE_END=$(can_errors)
STOP_FAIL=""; stop_chain
if [ $((IDLE_FAIL + CYC_FAIL)) -eq 0 ]; then RESULT=PASS; else RESULT=FAIL; fi
log "=== summary ==="
log "result=$RESULT idle_fail=$IDLE_FAIL cycles_ok=$CYC_DONE/20 v4_final=$FINAL rss_end_kb=$RSS_END"
log "can_errs_end=$CE_END"
log "=== v4 stability test end $(date +%F_%T) ==="
echo done > $DONE
