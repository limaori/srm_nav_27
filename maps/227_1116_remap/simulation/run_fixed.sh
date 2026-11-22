#!/usr/bin/env bash
# Isolate repeated simulation runs from stale DDS discovery; use original speed limits.
set -euo pipefail
SCENE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-37}"
export IGN_PARTITION="${IGN_PARTITION:-srm_227_1116_remap_fix}"
unset NAV_PARAMS_FILE SPEED_LIMITS_FILE
exec "$SCENE_DIR/run.sh"
