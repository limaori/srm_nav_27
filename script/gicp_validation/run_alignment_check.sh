#!/usr/bin/env bash
set -u
WS=/home/srm/srm_nav_27
export GICP_DOMAIN_ID=177
set +u
source "$WS/tmp/gicp_validation/env.sh"
set -u
cd "$WS"
bash "$WS/script/gicp_validation/start_validation_stack.sh" > /dev/null 2>&1 &
sleep 75
python3 "$WS/src/srm27_navigation/srm27_gicp_validation/tools/check_scan_map_alignment.py" \
  --pcd "$WS/tmp/gicp_validation/promap.pcd" --map-leaf 0.5 --range 25 2>&1 | tail -40
pkill -INT -f "srm_sim.launch.py" 2>/dev/null
pkill -INT -f "validation_stack.launch.py" 2>/dev/null
pkill -INT -f "ign gazebo" 2>/dev/null
