#!/usr/bin/env bash
# Demonstrated setting: 0.8 m/s, 1.5 m/s^2, terminal-stop radius 0.35 m.
set -euo pipefail
SCENE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export NAV_PARAMS_FILE="$SCENE_DIR/nav2_params_srm_tdt_fakeyaw_slow.yaml"
export SPEED_LIMITS_FILE="$SCENE_DIR/speed_limits_slow.yaml"
exec "$SCENE_DIR/run.sh"
