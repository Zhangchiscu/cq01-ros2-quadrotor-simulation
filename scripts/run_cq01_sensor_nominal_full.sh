#!/usr/bin/env bash
set -e
export CQ01_VEHICLE_PROFILE=nominal
export CQ01_SENSOR_PROFILE=nominal_closed_loop
exec "$HOME/drone_task/your_drone_sim_ws/scripts/run_cq01_v9_single.sh"
