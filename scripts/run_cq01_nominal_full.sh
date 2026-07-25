#!/usr/bin/env bash
set -e
export CQ01_VEHICLE_PROFILE=nominal
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_cq01_v9_single.sh"
