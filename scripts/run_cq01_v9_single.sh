#!/usr/bin/env bash
set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export CQ01_WS="$WS"
MAIN_PID=""
RVIZ_PID=""

PROFILE="${CQ01_VEHICLE_PROFILE:-nominal}"
SENSOR_PROFILE="${CQ01_SENSOR_PROFILE:-truth}"
case "$PROFILE" in
  nominal|heavy) ;;
  *)
    echo "[ERROR] 不支持的 vehicle profile：$PROFILE"
    echo "[ERROR] 仅支持 nominal 或 heavy"
    exit 2
    ;;
esac

case "$SENSOR_PROFILE" in
  truth)
    CONTROL_STATE_TOPIC="/drone/odom"
    USE_SENSOR_CHAIN_GUARD=false
    ;;
  nominal_closed_loop)
    CONTROL_STATE_TOPIC="/cq01_v9/sensors/nominal/odom"
    USE_SENSOR_CHAIN_GUARD=true
    ;;
  *)
    echo "[ERROR] 不支持的 sensor profile：$SENSOR_PROFILE"
    echo "[ERROR] 仅支持 truth 或 nominal_closed_loop"
    exit 2
    ;;
esac

cleanup() {
  trap - INT TERM EXIT

  if [ -n "${RVIZ_PID:-}" ] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill -INT "$RVIZ_PID" 2>/dev/null || true
  fi

  if [ -n "${MAIN_PID:-}" ] && kill -0 "$MAIN_PID" 2>/dev/null; then
    kill -INT "$MAIN_PID" 2>/dev/null || true
  fi

  sleep 1

  if [ -n "${RVIZ_PID:-}" ] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill -TERM "$RVIZ_PID" 2>/dev/null || true
  fi

  if [ -n "${MAIN_PID:-}" ] && kill -0 "$MAIN_PID" 2>/dev/null; then
    kill -TERM "$MAIN_PID" 2>/dev/null || true
  fi
}

trap cleanup INT TERM EXIT

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

RVIZ_CONFIG="$WS/install/cq01_v9_mc/share/cq01_v9_mc/rviz/cq01_v9_single_manual.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
  RVIZ_CONFIG="$WS/src/cq01_v9_mc/rviz/cq01_v9_single_manual.rviz"
fi

if [ ! -f "$RVIZ_CONFIG" ]; then
  echo "[ERROR] 找不到单机 RViz 配置：cq01_v9_single_manual.rviz"
  exit 1
fi

echo "============================================================"
echo "CQ01 SINGLE-UAV V9"
echo "仅启动原 V9 单机综合任务"
echo "不会启动 cq01_multi_uav、UAV2 或双路线评价器"
echo "============================================================"

PROFILE_CONFIG="$WS/install/cq01_v9_mc/share/cq01_v9_mc/config/vehicle_${PROFILE}.yaml"
if [ ! -f "$PROFILE_CONFIG" ]; then
  PROFILE_CONFIG="$WS/src/cq01_v9_mc/config/vehicle_${PROFILE}.yaml"
fi

if [ ! -f "$PROFILE_CONFIG" ]; then
  echo "[ERROR] 找不到参数配置：$PROFILE_CONFIG"
  exit 3
fi

echo "[INFO] Vehicle profile: $PROFILE"
echo "[INFO] Profile config: $PROFILE_CONFIG"
echo "[INFO] Sensor profile: $SENSOR_PROFILE"
echo "[INFO] Control state source: $CONTROL_STATE_TOPIC"

ros2 launch cq01_v9_mc \
  cq01_v9_mc.launch.py \
  use_rviz:=false \
  vehicle_profile:="$PROFILE" \
  vehicle_profile_config:="$PROFILE_CONFIG" \
  sensor_profile:="$SENSOR_PROFILE" \
  control_state_topic:="$CONTROL_STATE_TOPIC" \
  use_sensor_chain_guard:="$USE_SENSOR_CHAIN_GUARD" &
MAIN_PID=$!

for SECOND in 1 2 3 4 5 6 7 8; do
  if ! kill -0 "$MAIN_PID" 2>/dev/null; then
    wait "$MAIN_PID"
    STATUS=$?
    echo "[ERROR] V9 launch 提前退出，状态码：$STATUS"
    exit "$STATUS"
  fi
  echo "[INFO] RViz 启动倒计时：$((9 - SECOND)) s"
  sleep 1
done

QT_QPA_PLATFORM=xcb rviz2 \
  -d "$RVIZ_CONFIG" \
  --ros-args -r __node:=cq01_v9_rviz &
RVIZ_PID=$!

sleep 3
if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
  wait "$RVIZ_PID"
  STATUS=$?
  echo "[ERROR] RViz 启动失败，状态码：$STATUS"
  exit "$STATUS"
fi

echo "[OK] 单机 RViz 已启动，PID=$RVIZ_PID"
echo "[INFO] 按 Ctrl+C 可统一关闭 V9 和 RViz。"

wait "$MAIN_PID"
STATUS=$?
exit "$STATUS"
