#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/drone_task/your_drone_sim_ws"
RESULT_ROOT="$WS/results/cq01_bonus_suite"
LOG_ROOT="$RESULT_ROOT/suite_logs_$(date +%Y%m%d_%H%M%S)"
MODE="${1:---run-all}"
MAX_WAIT_S="${CQ01_SUITE_MAX_WAIT_S:-900}"

mkdir -p "$LOG_ROOT"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

if [ "$MODE" = "--aggregate-only" ]; then
  exec "$WS/scripts/generate_cq01_final_bonus_package.py"
fi

if [ "$MODE" != "--run-all" ]; then
  echo "用法："
  echo "  $0 --run-all"
  echo "  $0 --aggregate-only"
  exit 2
fi

if timeout 5 ros2 node list 2>/dev/null \
  | grep -Eq '^/quadrotor_dynamics$|^/quadrotor_controller$'; then
  echo "[ERROR] 检测到 CQ01/四旋翼节点仍在运行。"
  echo "[ERROR] 请先 Ctrl+C 关闭现有任务。"
  exit 3
fi

run_case() {
  local case_name="$1"
  local prefix="$2"
  local vehicle_profile="$3"
  local sensor_profile="$4"
  local control_topic="$5"
  local use_sensor_guard="$6"

  local config="$WS/install/cq01_v9_mc/share/cq01_v9_mc/config/vehicle_${vehicle_profile}.yaml"
  if [ ! -f "$config" ]; then
    config="$WS/src/cq01_v9_mc/config/vehicle_${vehicle_profile}.yaml"
  fi

  if [ ! -f "$config" ]; then
    echo "[ERROR] 找不到参数配置：$config"
    return 10
  fi

  local log="$LOG_ROOT/${case_name}.log"
  local launch_pid=""

  cleanup_case() {
    set +e
    if [ -n "$launch_pid" ] && kill -0 "$launch_pid" 2>/dev/null; then
      kill -INT "$launch_pid" 2>/dev/null || true
      sleep 3
      kill -TERM "$launch_pid" 2>/dev/null || true
    fi
    wait "$launch_pid" 2>/dev/null || true
    set -e
  }

  echo
  echo "============================================================"
  echo "RUN CASE: $case_name"
  echo "vehicle_profile: $vehicle_profile"
  echo "sensor_profile:  $sensor_profile"
  echo "control_state:   $control_topic"
  echo "log:             $log"
  echo "============================================================"

  local start_epoch
  start_epoch="$(date +%s)"

  ros2 launch cq01_v9_mc cq01_v9_mc.launch.py \
    use_rviz:=false \
    vehicle_profile:="$vehicle_profile" \
    vehicle_profile_config:="$config" \
    sensor_profile:="$sensor_profile" \
    control_state_topic:="$control_topic" \
    use_sensor_chain_guard:="$use_sensor_guard" \
    >"$log" 2>&1 &
  launch_pid=$!

  local result_dir=""
  local summary=""
  local elapsed=0

  while [ "$elapsed" -lt "$MAX_WAIT_S" ]; do
    result_dir="$(
      find "$RESULT_ROOT" \
        -maxdepth 1 \
        -mindepth 1 \
        -type d \
        -name "${prefix}*" \
        -newermt "@${start_epoch}" \
        -printf '%T@ %p\n' 2>/dev/null \
      | sort -nr \
      | head -n 1 \
      | cut -d' ' -f2-
    )"

    if [ -n "$result_dir" ]; then
      summary="$result_dir/bonus_summary.json"
      if [ -s "$summary" ]; then
        break
      fi
    fi

    if ! kill -0 "$launch_pid" 2>/dev/null; then
      echo "[FAIL] launch 在结果生成前退出：$case_name"
      tail -n 120 "$log" || true
      cleanup_case
      return 11
    fi

    sleep 5
    elapsed=$((elapsed + 5))
  done

  if [ ! -s "$summary" ]; then
    echo "[FAIL] 超时，未生成结果：$case_name"
    cleanup_case
    return 12
  fi

  sleep 5
  cleanup_case

  python3 - "$summary" "$case_name" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
case_name = sys.argv[2]
payload = json.loads(path.read_text(encoding="utf-8"))

print(f"[INFO] {case_name} result:", path)
print("[INFO] pass =", payload.get("pass"))
print("[INFO] failed_criteria =", payload.get("failed_criteria"))

if payload.get("pass") is not True:
    raise SystemExit(20)
PY

  echo "[PASS] $case_name"
}

run_case \
  "nominal_truth" \
  "nominal_full_" \
  "nominal" \
  "truth" \
  "/drone/odom" \
  "false"

run_case \
  "heavy_truth" \
  "heavy_full_" \
  "heavy" \
  "truth" \
  "/drone/odom" \
  "false"

run_case \
  "sensor_nominal_closed_loop" \
  "nominal_nominal_closed_loop_full_" \
  "nominal" \
  "nominal_closed_loop" \
  "/cq01_v9/sensors/nominal/odom" \
  "true"

"$WS/scripts/evaluate_cq01_parameter_profiles.sh"
"$WS/scripts/generate_cq01_final_bonus_package.py"

echo
echo "============================================================"
echo "CQ01 BONUS SUITE COMPLETED"
echo "Logs: $LOG_ROOT"
echo "============================================================"
