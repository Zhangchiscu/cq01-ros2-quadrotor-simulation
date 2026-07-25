#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/drone_task/your_drone_sim_ws"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

echo "===== Phase 1 节点 ====="
ros2 node list | grep -E 'cq01_v9_bonus_dashboard|cq01_v9_bonus_recorder' || true

echo
echo "===== Dashboard 控制话题审计（正常应无输出） ====="
ros2 node info /cq01_v9_bonus_dashboard 2>/dev/null \
  | grep -E '/drone/goal|external_wrench|control_mode|wrench_cmd' || true

echo
echo "===== Recorder 状态 ====="
ros2 topic echo /cq01_v9/bonus_recorder/status --once || true

echo
echo "===== 最新结果目录 ====="
find "$WS/results/cq01_bonus_suite" -mindepth 1 -maxdepth 1 \
  -type d -printf '%T@ %p\n' 2>/dev/null \
  | sort -nr | head -n 3 | cut -d' ' -f2- || true
