#!/usr/bin/env bash
set -e
export CQ01_VEHICLE_PROFILE=heavy
exec "$HOME/drone_task/your_drone_sim_ws/scripts/run_cq01_v9_single.sh"
