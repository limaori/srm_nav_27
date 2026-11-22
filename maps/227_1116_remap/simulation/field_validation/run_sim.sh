#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
export IGN_PARTITION="${IGN_PARTITION:-tdt_field_validation_${ROS_DOMAIN_ID}}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
if [[ -z "${DISPLAY:-}" ]]; then
  command -v Xvfb >/dev/null || { echo 'A display or Xvfb is required for the lidar renderer.' >&2; exit 1; }
  export DISPLAY="${SRM_VALIDATION_DISPLAY:-:111}"
  Xvfb "$DISPLAY" -screen 0 1280x1024x24 >"$OUTPUT_DIR/xvfb.log" 2>&1 &
  XVFB_PID=$!
  trap 'kill "$XVFB_PID" 2>/dev/null || true' EXIT
  sleep 1
  kill -0 "$XVFB_PID" || { echo "Xvfb failed; see $OUTPUT_DIR/xvfb.log" >&2; exit 1; }
fi
cd "$WORKSPACE_DIR"
python3 "$SCENE_DIR/build_scene.py" >"$OUTPUT_DIR/scene_manifest.log"
ros2 launch srm27_nav_bringup nav_srm_simulation_launch.py \
  namespace:=red_standard_robot1 world:=srm_empty \
  map:="$SCENE_DIR/../227_1116_remap.yaml" \
  params_file:="${NAV_PARAMS_FILE:-$SCENE_DIR/nav2_params_srm_tdt_fakeyaw.yaml}" \
  speed_limits_file:="$WORKSPACE_DIR/src/srm27_navigation/srm27_nav_bringup/config/simulation/speed_limits.yaml" \
  start_simulation:=True sim_config_file:="$SCENE_DIR/sim.yaml" \
  map_to_odom_x:=-1.905 map_to_odom_y:=1.235 map_to_odom_yaw:=0.0 \
  gui:="${SIM_GUI:-False}" run_immediately:=True use_rviz:="${USE_RVIZ:-False}" \
  use_composition:=False use_sim_time:=True use_lio_odometry:=False \
  use_pcd_localization:=False odometry_frontend:=point_lio gimbal_spin_rate:=0.0
