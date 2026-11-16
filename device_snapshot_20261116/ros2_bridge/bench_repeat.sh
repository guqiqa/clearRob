#!/bin/bash
set +u
TASK=$1
COUNT=${2:-5}
source /root/nav_ws/env.sh
for i in $(seq 1 $COUNT); do
  echo "===== RUN $i/$COUNT $TASK ====="
  WATCH=90 timeout 180 bash /root/ros2_bridge/run_mission_bench.sh "$TASK" -p distance_m:=0.5 > /tmp/bench_${TASK}_${i}.log 2>&1
  cp /tmp/trace_$TASK.csv /tmp/bench_${TASK}_${i}.csv 2>/dev/null
  grep -E "FINAL:" /tmp/bench_${TASK}_${i}.log | tail -1
  pkill -f wheel_speed_node 2>/dev/null
  sleep 2
done
echo "===== $TASK x$COUNT DONE ====="
