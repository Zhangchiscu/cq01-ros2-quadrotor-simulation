#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/drone_task/your_drone_sim_ws"
OUT="$WS/results/cq01_requirement_closure"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$OUT/requirement_closure_audit_$STAMP.txt"

mkdir -p "$OUT"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

{
  echo "======================================================================"
  echo "CQ01 REQUIREMENT CLOSURE AUDIT"
  echo "time=$STAMP"
  echo "======================================================================"
  echo

  echo "[1] Core topic types"
  for topic in /drone/odom /drone/imu /map/obstacles; do
    echo "--- $topic"
    ros2 topic type "$topic" 2>/dev/null || true
  done
  echo

  echo "[2] Add-on topic types"
  for topic in \
    /drone/odom_noisy \
    /drone/imu_noisy \
    /map/local_obstacles \
    /cq01_bonus/perception_range \
    /cq01_bonus/sensor_noise/status \
    /cq01_bonus/local_perception/status
  do
    echo "--- $topic"
    ros2 topic type "$topic" 2>/dev/null || true
  done
  echo

  echo "[3] Sensor-noise status"
  timeout 4 ros2 topic echo \
    /cq01_bonus/sensor_noise/status \
    --once 2>/dev/null || true
  echo

  echo "[4] Local-perception status"
  timeout 4 ros2 topic echo \
    /cq01_bonus/local_perception/status \
    --once 2>/dev/null || true
  echo

  echo "[5] Noisy odom sample"
  timeout 4 ros2 topic echo \
    /drone/odom_noisy \
    --once 2>/dev/null | head -n 45 || true
  echo

  echo "[6] Local obstacle count/sample"
  timeout 4 ros2 topic echo \
    /map/local_obstacles \
    --once 2>/dev/null | head -n 55 || true
  echo

  echo "======================================================================"
  echo "AUDIT COMPLETE"
  echo "======================================================================"
} | tee "$REPORT"

echo
echo "Saved: $REPORT"
