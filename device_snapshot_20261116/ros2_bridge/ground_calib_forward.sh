#!/bin/bash
# 地面里程计标定：单次 FORWARD 0.5 m（odom 闭环），人工卷尺量实际距离并输入，计算比例
set +u
source /root/nav_ws/env.sh
RES=/tmp/ground_calib_result.txt
: > $RES
log() { echo "$*" | tee -a $RES; }
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

log "=== ground odom calibration $(date +%F_%T) ==="
systemctl is-active laifei-chassis-v4.service | grep -q active || fail "chassis unit not active"
ST=$(timeout 8 ros2 topic echo /chassis/state --once 2>/dev/null)
echo "$ST" | grep -q "control_mode: MANUAL" || fail "not MANUAL"
echo "准备：地面贴 0 m 与 0.5 m 标记；车头对准直线；前后各留 1 m 清空；一人持急停。"
read -r -p "确认标记完成，输入 GO 开始（其他键中止）: " ANS
[ "$ANS" = "GO" ] || { echo "已中止"; exit 0; }

setsid nohup ros2 launch navigation_ros2 navigation_hardware_dry_run.launch.py > /tmp/calib_nav.log 2>&1 < /dev/null &
sleep 8
setsid nohup python3 /root/ros2_bridge/flat_ground_grid.py > /tmp/calib_grid.log 2>&1 < /dev/null &
sleep 1
timeout 8 ros2 param set /navigation_node profile GROUND_DEMO_BENCH_STABLE >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
timeout 8 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1

setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file /root/nav_ws/config/ground_demo_bench_stable.yaml -p task:=FORWARD -p distance_m:=0.5 > /tmp/calib_mission.log 2>&1 < /dev/null &
sleep 4
timeout 8 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
timeout 8 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
sleep 1
X0=$(odomx); log "odom_start_x=$X0"
timeout 8 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1
FINAL=""; for k in $(seq 1 90); do
  st=$(timeout 5 ros2 topic echo /demo_mission/status --once 2>/dev/null | grep -o "state=[A-Z_]*" | head -1 | cut -d= -f2)
  case "$st" in COMPLETED|ABORTED|STOPPED|TIMEOUT|CHASSIS_FAULT|ESTOP) FINAL=$st; break;; esac
  sleep 1
done
sleep 1
X1=$(odomx); log "odom_end_x=$X1"
ODOM=$(awk "BEGIN{printf \"%.4f\", ($X1)-($X0); if ($X1-$X0<0) printf \"\"}" 2>/dev/null)
ODOM=$(python3 -c "print(round(abs(float(\"$X1\")-float(\"$X0\")),4))" 2>/dev/null)
log "mission=$FINAL odom_distance=$ODOM m"
timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
pkill -f demo_mission_manager 2>/dev/null
cleanup

echo "请用卷尺测量：起点标记 -> 机器人最终位置（沿运动方向）的实际距离（米）"
read -r -p "实测物理距离(m): " PHYS
SCALE=$(awk "BEGIN{printf \"%.4f\", ($PHYS)/($ODOM)}" 2>/dev/null)
log "physical=$PHYS m odom=$ODOM m scale=physical/odom=$SCALE"
echo "建议:" | tee -a $RES
echo "  若 scale 接近 1.0: odom 标定基本正确，之前的偏差可能来自手测/打滑" | tee -a $RES
echo "  若 scale 明显 < 1: 配置轮径偏大，需要把 Wheel_Radius_m 调小" | tee -a $RES
echo "  scale=$SCALE -> 参考新轮径 = 0.10 * scale = $(awk "BEGIN{printf \"%.4f\", 0.10*($SCALE)}" 2>/dev/null) m" | tee -a $RES
log "结果文件: /tmp/ground_calib_result.txt"
