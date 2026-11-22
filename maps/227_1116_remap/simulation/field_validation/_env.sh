#!/usr/bin/env bash
# Shared environment for the map-specific validation entry points.
VALIDATION_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SRM27_WS_DIR:-$(cd "$VALIDATION_DIR/../../../.." && pwd)}"
SCENE_DIR="$WORKSPACE_DIR/maps/227_1116_remap/simulation"
OUTPUT_DIR="${SRM_VALIDATION_OUTPUT:-$WORKSPACE_DIR/log_diag/tdt_field_validation}"
WORKSPACE_SETUP="${SRM27_SETUP_FILE:-$WORKSPACE_DIR/install/setup.bash}"
if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "Missing workspace setup: $WORKSPACE_SETUP; build the Humble workspace first." >&2
  return 1
fi
source "${SRM27_ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "$WORKSPACE_SETUP"
source "$WORKSPACE_DIR/script/lib/srm_lib_path_guard.sh"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-54}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
mkdir -p "$OUTPUT_DIR"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$OUTPUT_DIR/ros}"
