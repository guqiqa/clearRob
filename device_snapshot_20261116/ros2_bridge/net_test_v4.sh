#!/bin/bash
# 网络断开测试：本机节点间 ROS 通信是否会中断（eth0 down/up 4s）
set +u
source /root/nav_ws/env.sh
OUT=/tmp/net_test_result.txt
: > $OUT
pkill -f demo_mission_manager 2>/dev/null; pkill -f record_trace 2>/dev/null
bash /root/ros2_bridge/stop_v4_bench.sh >/dev/null 2>&1
sleep 1
echo "=== net test start $(date +%T) ===" | tee -a $OUT
timeout 120 bash /root/ros2_bridge/start_v4_bench.sh >> $OUT 2>&1
sleep 1
timeout 10 ros2 param set /navigation_node profile GROUND_DEMO_BENCH_STABLE >/dev/null 2>&1
timeout 10 ros2 param set /safety_supervisor max_linear_x_mps 0.12 >/dev/null 2>&1
timeout 10 ros2 param set /safety_supervisor max_angular_z_radps 0.20 >/dev/null 2>&1
setsid nohup bash /root/ros2_bridge/run_demo_mission.sh --ros-args --params-file /root/nav_ws/config/ground_demo_bench_stable.yaml -p task:=FORWARD -p distance_m:=3.0 > /tmp/net_mission.log 2>&1 < /dev/null &
sleep 5
timeout 10 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
timeout 10 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
sleep 1
timeout 10 ros2 service call /demo_mission/start std_srvs/srv/Trigger >/dev/null 2>&1
sleep 2.5
timeout 8 ros2 topic echo /odom --once > /tmp/net_odom_pre.txt 2>&1
echo "[net] dropping eth0 for 4 s" | tee -a $OUT
setsid nohup bash -c "sleep 1; ip link set eth0 down; sleep 4; ip link set eth0 up" > /tmp/net_link.log 2>&1 < /dev/null &
sleep 12
timeout 8 ros2 topic echo /odom --once > /tmp/net_odom_post.txt 2>&1
timeout 8 ros2 topic echo /chassis/state --once > /tmp/net_state.txt 2>&1
timeout 8 ros2 topic echo /demo_mission/status --once > /tmp/net_mission_status.txt 2>&1
echo "--- odom pre/post ---" | tee -a $OUT
grep -A2 "position:" /tmp/net_odom_pre.txt | head -4 | tee -a $OUT
grep -A2 "position:" /tmp/net_odom_post.txt | head -4 | tee -a $OUT
echo "--- chassis state ---" | tee -a $OUT
grep -E "control_mode|motion_allowed|can_healthy|feedback_valid|fault_reason|command_age" /tmp/net_state.txt | tee -a $OUT
echo "--- mission ---" | tee -a $OUT
cat /tmp/net_mission_status.txt | tail -2 | tee -a $OUT
echo "--- v4 latch lines ---" | tee -a $OUT
grep -E "safety latch|SAFETY_STATE|ROS_VELOCITY" /tmp/v4_chassis.log | tail -6 | tee -a $OUT
echo "--- eth0 ---" | tee -a $OUT
ip -br link show eth0 | tee -a $OUT
echo "=== net test end $(date +%T) ===" | tee -a $OUT
timeout 6 ros2 service call /control/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
pkill -f demo_mission_manager 2>/dev/null
bash /root/ros2_bridge/stop_v4_bench.sh >/dev/null 2>&1
