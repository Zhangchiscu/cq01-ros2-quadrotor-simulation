#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/drone_task/your_drone_sim_ws"
OUT="$WS/results/cq01_requirement_closure"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$OUT/requirement_closure_v12_audit_$STAMP.txt"

mkdir -p "$OUT"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

{
  echo "======================================================================"
  echo "CQ01 REQUIREMENT CLOSURE V1.2 AUDIT"
  echo "time=$STAMP"
  echo "======================================================================"
  echo

  echo "[0] Running nodes"
  ros2 node list \
    | grep -E "sensor_noise_bridge|local_perception_filter" \
    || true
  echo

  echo "[1] Core input topic"
  ros2 topic type /map/obstacles 2>/dev/null || true
  echo

  echo "[2] Sensor-noise status"
  timeout 4 ros2 topic echo \
    /cq01_bonus/sensor_noise/status \
    --once 2>/dev/null || true
  echo

  echo "[3] Local-perception status"
  timeout 4 ros2 topic echo \
    /cq01_bonus/local_perception/status \
    --once 2>/dev/null || true
  echo

  echo "[4] Local point-cloud type"
  ros2 topic type \
    /map/local_obstacles_cloud \
    2>/dev/null || true
  echo

  echo "[5] Local point-cloud width"
  timeout 4 ros2 topic echo \
    /map/local_obstacles_cloud \
    --once \
    --field width \
    2>/dev/null || true
  echo

  echo "[6] Perception-range marker type"
  ros2 topic type \
    /cq01_bonus/perception_range \
    2>/dev/null || true
  echo

  echo "======================================================================"
  echo "V1.2 AUDIT COMPLETE"
  echo "======================================================================"
} | tee "$REPORT"

echo
echo "Saved: $REPORT"
