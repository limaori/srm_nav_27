#!/usr/bin/env bash
set -eo pipefail
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SOURCE_DIR/../../.." && pwd)"
SCENE_DIR="$WS_DIR/log_diag/sim_tdt_smoother_20261117"
cd "$WS_DIR"
if [ ! -f "$SCENE_DIR/sim.yaml" ]; then
  OPENBLAS_NUM_THREADS=1 python3 "$SOURCE_DIR/build_scene.py"
fi
source script/lib/srm_lib_path_guard.sh
source install/setup.bash
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
export IGN_PARTITION=tdt_smoother_227_1113pro_87
exec ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
 namespace:=red_standard_robot1 world:=srm_empty \
 map:="$WS_DIR/maps/227_1113pro/227_1113pro.yaml" \
 params_file:="$SOURCE_DIR/nav2_params_fixed_fakeyaw.yaml" \
 speed_limits_file:="$SOURCE_DIR/speed_limits.yaml" \
 start_simulation:=True sim_config_file:="$SCENE_DIR/sim.yaml" \
 map_to_odom_x:=0.236 map_to_odom_y:=-0.110 map_to_odom_yaw:=0.0 \
 gui:=False run_immediately:=True use_rviz:=False use_composition:=False \
 use_sim_time:=True use_lio_odometry:=False use_pcd_localization:=False \
 odometry_frontend:=point_lio gimbal_spin_rate:=0.0
