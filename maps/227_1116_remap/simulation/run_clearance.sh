#!/usr/bin/env bash
# Validated original-speed setting: command history, local seed, inward velocity guard.
set -euo pipefail
SCENE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-54}"
export IGN_PARTITION="${IGN_PARTITION:-srm_clearance54}"
export SIM_GUI="${SIM_GUI:-False}"
unset NAV_PARAMS_FILE SPEED_LIMITS_FILE
# Regenerate absolute resource paths for the current checkout before launching.
python3 "$SCENE_DIR/build_scene.py"
exec "$SCENE_DIR/run.sh"
