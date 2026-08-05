#!/bin/bash
# cleaning_robot — start / stop / restart script for the full Phase 1 + 2 stack.
# Usage:
#   ./start.sh              # launch (with autopilot)
#   ./start.sh --no-auto    # launch (without autonomous nodes)
#   ./stop.sh               # kill ALL related processes
#   ./restart.sh            # stop + start

set -e

NODES="imu_node|chassis_node|remote_node|rtk_node|bridge_node|fusion_node|path_planner|ros2 launch"
LOG=/tmp/cleaning_robot.log
WS=/root/cleaning_robot_ws

source $WS/install/setup.bash 2>/dev/null || true
source /opt/ros/humble/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=0

# ----- stop -----
do_stop() {
    echo "=== stopping cleaning_robot ==="
    for pid in $(ps -ef | grep -E "$NODES" | grep -v grep | awk '{print $2}'); do
        kill -9 "$pid" 2>/dev/null || true
    done
    sleep 2
    remaining=$(ps -ef | grep -E "$NODES" | grep -v grep | wc -l)
    if [ "$remaining" -gt 0 ]; then
        echo "WARNING: $remaining process(es) still alive, retrying..."
        for pid in $(ps -ef | grep -E "$NODES" | grep -v grep | awk '{print $2}'); do
            kill -9 "$pid" 2>/dev/null || true
        done
        sleep 1
    fi
    echo "all cleaning_robot processes stopped"
}

# ----- start -----
do_start() {
    AUTOPILOT="autopilot:=true"
    if [ "$1" = "--no-auto" ]; then
        AUTOPILOT="autopilot:=false"
    fi
    echo "=== starting cleaning_robot (autopilot=$AUTOPILOT) ==="
    nohup ros2 launch cleaning_robot_bringup phase1_chassis.launch.py \
        simulate:=false $AUTOPILOT > $LOG 2>&1 &
    launch_pid=$!
    echo "launch PID: $launch_pid"
    sleep 10
    count=$(ps -ef | grep -E "$NODES" | grep -v grep | wc -l)
    echo "started $count processes"
    echo "tail -f $LOG  to follow logs"
}

# ----- restart -----
do_restart() {
    do_stop
    do_start "$@"
}

# ----- status -----
do_status() {
    echo "=== cleaning_robot status ==="
    running=$(ps -ef | grep -E "$NODES" | grep -v grep | wc -l)
    echo "processes: $running"
    for n in imu_node chassis_node remote_node rtk_node bridge_node fusion_node path_planner; do
        c=$(ps -ef | grep "$n" | grep -v grep | wc -l)
        if [ "$c" -ne 1 ]; then
            echo "  WARNING: $n: $c instances (expected 1)"
        else
            echo "  $n: $c"
        fi
    done
}

case "${1:-start}" in
    start)   do_start "$2" ;;
    stop)    do_stop ;;
    restart) do_restart "$2" ;;
    status)  do_status ;;
    *)       echo "Usage: $0 {start|stop|restart|status} [--no-auto]" ;;
esac
