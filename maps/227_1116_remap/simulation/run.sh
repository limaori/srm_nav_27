#!/usr/bin/env bash
set -euo pipefail
SCENE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCENE_DIR/../../.." && pwd)"
cd "$WS_DIR"
source script/lib/srm_lib_path_guard.sh
set +u
source install/setup.bash
set -u
exec ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
 namespace:=red_standard_robot1 world:=srm_empty \
 map:="$SCENE_DIR/../227_1116_remap.yaml" \
 params_file:="${NAV_PARAMS_FILE:-$SCENE_DIR/nav2_params_srm_tdt_fakeyaw.yaml}" \
 speed_limits_file:="${SPEED_LIMITS_FILE:-$WS_DIR/src/srm27_navigation/srm27_nav_bringup/config/simulation/speed_limits.yaml}" \
 start_simulation:=True sim_config_file:="$SCENE_DIR/sim.yaml" \
 map_to_odom_x:=-1.905 map_to_odom_y:=1.235 map_to_odom_yaw:=0.0 \
 gui:="${SIM_GUI:-True}" run_immediately:=True use_rviz:=True use_composition:=False \
 use_sim_time:=True use_lio_odometry:=False use_pcd_localization:=False \
 odometry_frontend:=point_lio gimbal_spin_rate:=0.0
