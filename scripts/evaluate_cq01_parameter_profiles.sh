#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/drone_task/your_drone_sim_ws"

set +u
source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && \
  source "$WS/install/setup.bash"
set -u

exec python3 \
  "$WS/scripts/evaluate_cq01_parameter_profiles.py"
