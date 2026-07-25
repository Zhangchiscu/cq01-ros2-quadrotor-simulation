#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/drone_task/your_drone_sim_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/mnt/hgfs/VMShare/CQ01_TEACHER_INTERFACE_PASS_${STAMP}"
LOG="$OUT/launch.log"
mkdir -p "$OUT"

LAUNCH_PID=""

cleanup() {
  set +e
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 3
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
  fi
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

LIVE="$(timeout 5 ros2 node list 2>/dev/null || true)"
if printf '%s\n' "$LIVE" | grep -Eq \
  '^/quadrotor_dynamics$|^/quadrotor_controller$|^/astar_3d_planner$'; then
  echo "[ERROR] 检测到任务仍在运行，请先关闭。"
  exit 2
fi

# 清除已经退出的旧 DDS 图缓存，避免旧多无人机 topic 污染审计。
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true
sleep 2

CFG="$WS/install/cq01_v9_mc/share/cq01_v9_mc/config/vehicle_nominal.yaml"
if [ ! -f "$CFG" ]; then
  CFG="$WS/src/cq01_v9_mc/config/vehicle_nominal.yaml"
fi

ros2 launch cq01_v9_mc cq01_v9_mc.launch.py \
  use_rviz:=false \
  vehicle_profile:=nominal \
  vehicle_profile_config:="$CFG" \
  sensor_profile:=truth \
  control_state_topic:=/drone/odom \
  use_sensor_chain_guard:=false \
  >"$LOG" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 35); do
  NODES="$(timeout 5 ros2 node list 2>/dev/null || true)"
  if printf '%s\n' "$NODES" | grep -q '^/quadrotor_dynamics$' \
    && printf '%s\n' "$NODES" | grep -q '^/astar_3d_planner$' \
    && printf '%s\n' "$NODES" | grep -q '^/cq01_v9_teacher_interface_bridge$'; then
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[FAIL] launch 提前退出"
    tail -n 120 "$LOG" || true
    exit 3
  fi
  sleep 1
done

sleep 8

ros2 node list >"$OUT/node_list.txt"
ros2 topic list -t >"$OUT/topic_list_types.txt"

TOPICS=(
  /drone/motor_rpm_cmd
  /drone/odom
  /drone/imu
  /drone/path
  /drone/goal
  /map/obstacles
  /drone/reference
  /drone/waypoints
  /drone/safe_goal
)

for topic in "${TOPICS[@]}"; do
  safe="$(printf '%s' "$topic" | tr '/:' '__')"
  timeout 8 ros2 topic info -v "$topic" \
    >"$OUT/topic_info_${safe}.txt" 2>&1 || true
  timeout 6 ros2 topic echo "$topic" --once \
    >"$OUT/topic_sample_${safe}.txt" 2>&1 || true
done

timeout 10 ros2 run tf2_ros tf2_echo map base_link \
  >"$OUT/tf_map_base_link.txt" 2>&1 || true

python3 - "$OUT" <<'PY'
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

root = Path(sys.argv[1])
lines = (
    root / "topic_list_types.txt"
).read_text(
    encoding="utf-8",
    errors="replace",
).splitlines()

topics = {}
for line in lines:
    match = re.match(r"^(\S+)\s+\[(.+)\]\s*$", line.strip())
    if match:
        topics[match.group(1)] = match.group(2)

expected = {
    "/drone/motor_rpm_cmd":
        "std_msgs/msg/Float32MultiArray",
    "/drone/odom":
        "nav_msgs/msg/Odometry",
    "/drone/imu":
        "sensor_msgs/msg/Imu",
    "/drone/path":
        "nav_msgs/msg/Path",
    "/drone/goal":
        "geometry_msgs/msg/PoseStamped",
    "/map/obstacles":
        "sensor_msgs/msg/PointCloud2",
    "/drone/reference":
        "nav_msgs/msg/Path",
    "/drone/waypoints":
        "nav_msgs/msg/Path",
    "/drone/safe_goal":
        "geometry_msgs/msg/PoseStamped",
}

checks = {
    f"topic_{name}":
        topics.get(name) == message_type
    for name, message_type in expected.items()
}

tf_text = (
    root / "tf_map_base_link.txt"
).read_text(
    encoding="utf-8",
    errors="replace",
)
# tf2_echo may print an initial waiting message before the frame appears.
checks["dynamic_tf_map_base_link"] = (
    "Translation:" in tf_text
    and "Rotation:" in tf_text
    and "At time" in tf_text
)

node_text = (
    root / "node_list.txt"
).read_text(
    encoding="utf-8",
    errors="replace",
)
node_names = [
    line.strip()
    for line in node_text.splitlines()
    if line.startswith("/")
]
checks["no_duplicate_node_names"] = (
    len(node_names) == len(set(node_names))
)
checks["no_active_multi_uav_nodes"] = not any(
    "multi_uav" in name
    or name.startswith("/uav1/")
    or name.startswith("/uav2/")
    for name in node_names
)
checks["teacher_bridge_present"] = (
    "/cq01_v9_teacher_interface_bridge"
    in node_names
)

failed = [
    name
    for name, passed in checks.items()
    if passed is not True
]

payload = {
    "pass": not failed,
    "failed_criteria": failed,
    "checks": checks,
    "topic_types": topics,
    "tf_map_base_link_detected": (
        checks["dynamic_tf_map_base_link"]
    ),
}

(root / "teacher_interface_summary.json").write_text(
    json.dumps(
        payload,
        ensure_ascii=False,
        indent=2,
    ),
    encoding="utf-8",
)

print("=" * 72)
print("CQ01 TEACHER INTERFACE STRICT AUDIT")
for name, passed in checks.items():
    print(f'[{"PASS" if passed else "FAIL"}] {name}')
print()
print(
    "TEACHER INTERFACE RESULT: "
    f"pass={not failed}; failed={failed}"
)
print("Output:", root)
print("=" * 72)

raise SystemExit(0 if not failed else 1)
PY

tar -czf "${OUT}.tar.gz" \
  -C "$(dirname "$OUT")" \
  "$(basename "$OUT")"

echo "[OK] 压缩包：${OUT}.tar.gz"
