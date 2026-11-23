#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
exec python3 "$WORKSPACE_DIR/script/waypoint_mission.py" \
  --file "$VALIDATION_DIR/out_and_back.yaml" --namespace red_standard_robot1 \
  --loop --pause 2 --retry 0 --on-failure abort --timeout 60 --send-settle 2 --no-markers
