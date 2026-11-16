#!/bin/bash
# v4 故障测试：mission 运行中触发故障，验证零速/锁存
set +u
source /root/nav_ws/env.sh
MODE=$1
PARAMS=/root/nav_ws/config/ground_demo_bench_stable.yaml
PROFILE=GROUND_DEMO_BENCH_STABLE
pkill -f demo_mission_manager 2>/dev/null
pkill -f record_trace 2>/dev/null
bash /root/ros2_bridge/stop_v4_bench.sh >/dev/null 2>&1
sleep 1
echo "=== FAULT TEST: $MODE ==="
timeout 120 bash /root/ros2_bridge/start_v4_bench.sh > /tmp/ft_start.log 2>&1
sleep 1
timeout 10 ros2 param set /navigation_node profile $PROFILE >/dev/null 2>&1
timeout 10 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
timeout 10 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1
setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file $PARAMS -p task:=FORWARD -p distance_m:=1.5 > /tmp/ft_mission.log 2>&1 < /dev/null &
sleep 5
timeout 10 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
timeout 10 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
sleep 1
timeout 8 ros2 topic echo /odom --once > /tmp/ft_odom0.txt 2>&1
timeout 10 ros2 service call /demo_mission/start std_srvs/srv/Trigger 2>&1 | tail -1
sleep 2.5
timeout 8 ros2 topic echo /odom --once > /tmp/ft_odom_pre.txt 2>&1
echo "[fault] triggering $MODE"
case "$MODE" in
  estop) timeout 10 ros2 topic pub --once /safety/emergency_stop std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 ;;
  mode_off) timeout 10 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 ;;
  kill_nav) pkill -f navigation_node 2>/dev/null ;;
  kill_safety) pkill -f safety_supervisor_node 2>/dev/null ;;
  can_down) ip link set can0 down 2>/dev/null ;;
  *) echo "unknown mode"; exit 2 ;;
esac
sleep 2
timeout 8 ros2 topic echo /odom --once > /tmp/ft_odom_post1.txt 2>&1
timeout 8 ros2 topic echo /chassis/state --once > /tmp/ft_state.txt 2>&1
timeout 8 ros2 topic echo /safety/status --once > /tmp/ft_safety.txt 2>&1
timeout 8 ros2 topic echo /demo_mission/status --once > /tmp/ft_mission_status.txt 2>&1
sleep 2
timeout 8 ros2 topic echo /odom --once > /tmp/ft_odom_post2.txt 2>&1
echo "--- v4 log tail ---"; tail -6 /tmp/v4_chassis.log
echo "--- cleanup ---"
timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
if [ "$MODE" = "can_down" ]; then ip link set can0 up 2>/dev/null; fi
pkill -f demo_mission_manager 2>/dev/null
bash /root/ros2_bridge/stop_v4_bench.sh >/dev/null 2>&1
echo "[$MODE] cleanup done"
