#!/bin/bash
# G1 地面测试：直线前进 0.3 m ×3（0.12 m/s，轮子着地，交互确认后执行）
# 用法：bash /root/ros2_bridge/ground_g1_forward.sh
set +u
source /root/nav_ws/env.sh
RES=/tmp/g1_forward_result.txt
LOG=/tmp/g1_forward.log
: > $RES; : > $LOG
log() { echo "$*" | tee -a $RES >> $LOG; }
cleanup() {
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f demo_mission_manager 2>/dev/null
  pkill -f navigation_hardware_dry_run.launch.py 2>/dev/null
  pkill -f navigation_node 2>/dev/null
  pkill -f safety_supervisor_node 2>/dev/null
  pkill -f flat_ground_grid 2>/dev/null
  sleep 1
}
fail() { log "FAIL: $*"; cleanup; exit 1; }
odomx() { timeout 6 ros2 topic echo /odom --once 2>/dev/null | grep -A2 "position:" | grep "x:" | head -1 | awk '{print $2}'; }

log "=== G1 forward ground test $(date +%F_%T) ==="

# 前置检查
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "chassis unit not active"
pgrep -f "wheel_speed_node|hardware_adapter" >/dev/null && fail "forbidden CAN node running"
ST=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
echo "$ST" | grep -q "control_mode: MANUAL" || fail "chassis not MANUAL before test"
CMDP=$(timeout 8 ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/ {print $3}')
[ "$CMDP" = "0" ] || fail "/cmd_vel publishers=$CMDP (expect 0)"
echo "前检查 OK: MANUAL / 零速 / cmd_vel 0 发布者"
echo "安全确认：场地已清空、人员持急停、后退方向无人。"
read -r -p "输入 GO 开始 G1（其他任意键中止）: " ANS
[ "$ANS" = "GO" ] || { echo "已中止，未执行运动"; exit 0; }

# 启动 nav + safety + grid
setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/g1_nav.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 /root/ros2_bridge/flat_ground_grid.py > /tmp/g1_grid.log 2>&1 < /dev/null &
sleep 1
timeout 8 ros2 param set /navigation_node profile GROUND_DEMO_BENCH_STABLE >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1

for i in 1 2 3; do
  log "--- run $i/3: FORWARD 0.3 m @0.12 ---"
  pkill -f demo_mission_manager 2>/dev/null; sleep 1
  setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file /root/nav_ws/config/ground_demo_bench_stable.yaml -p task:=FORWARD -p distance_m:=0.3 > /tmp/g1_mission.log 2>&1 < /dev/null &
  sleep 4
  timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  timeout 8 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
  sleep 1
  X0=$(odomx); log "x0=$X0"
  timeout 8 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1
  FINAL=""; for k in $(seq 1 90); do
    st=$(timeout 5 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
    case "$st" in COMPLETED|ABORTED|STOPPED|TIMEOUT|CHASSIS_FAULT|ESTOP) FINAL=$st; break;; esac
    sleep 1
  done
  sleep 1
  X1=$(odomx)
  timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  pkill -f demo_mission_manager 2>/dev/null
  DIST=$(python3 -c "print(abs(float(\"$X1\")-float(\"$X0\")))" 2>/dev/null)
  log "run=$i final=$FINAL x0=$X0 x1=$X1 distance=$DIST"
  [ "$FINAL" = "COMPLETED" ] || fail "run $i final=$FINAL"
  sleep 2
done

cleanup
ST=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
echo "$ST" | grep -q "control_mode: MANUAL" || fail "final state not MANUAL"
log "=== G1 RESULT: PASS ==="
log "结果文件: /tmp/g1_forward_result.txt"
