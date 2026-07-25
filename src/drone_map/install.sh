#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${HOME}/drone_task/your_drone_sim_ws"
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="${WORKSPACE}/src/drone_map"
BACKUP_ROOT="${HOME}/drone_task/package_backups"

if [ ! -d "${WORKSPACE}/src" ]; then
  echo "[错误] 未找到工作空间：${WORKSPACE}"
  exit 1
fi

mkdir -p "${BACKUP_ROOT}"

if [ -d "${TARGET_DIR}" ]; then
  BACKUP_DIR="${BACKUP_ROOT}/drone_map.$(date +%Y%m%d_%H%M%S)"
  cp -a "${TARGET_DIR}" "${BACKUP_DIR}"
  echo "[信息] 已备份旧包到：${BACKUP_DIR}"
fi

rm -rf "${TARGET_DIR}"
mkdir -p "${TARGET_DIR}"
cp -a "${SOURCE_DIR}/." "${TARGET_DIR}/"

chmod +x \
  "${TARGET_DIR}/drone_map/static_obstacle_map.py"

source /opt/ros/humble/setup.bash

cd "${WORKSPACE}"

rm -rf build/drone_map
rm -rf install/drone_map

colcon build \
  --symlink-install \
  --packages-select drone_map

source "${WORKSPACE}/install/setup.bash"

echo
echo "===== 可执行文件检查 ====="
ros2 pkg executables drone_map
echo
echo "[完成] drone_map 已安装并编译。"
echo "运行："
echo "ros2 launch drone_map static_map.launch.py"
