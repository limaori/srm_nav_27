#!/usr/bin/env bash
# 端到端验证: 只起 雷达 + 车体TF + 导航栈 (不开 RViz / 底盘串口), 看 Nav2 能不能激活
set -u
cd /home/srm/srm_nav_27
export ROS_LOG_DIR=/home/srm/srm_nav_27/.dsh_run/roslog_e2e
mkdir -p "$ROS_LOG_DIR"
set +u; source install/setup.bash; set -u
PKG=install/srm27_nav_bringup/share/srm27_nav_bringup
PARAMS=$PKG/config/real/nav2_params_srm_minco_fakeyaw.yaml
MAP=/home/srm/srm_nav_27/maps/227_1113pro/227_1113pro.yaml
PCD=/home/srm/srm_nav_27/maps/pcd/227_1113pro_mapframe.pcd

ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 --params-file "$PARAMS" \
  -p user_config_path:=$PKG/config/real/mid360_user_config.json > .dsh_run/e2e_driver.log 2>&1 &
DRV=$!
ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py use_sim_time:=False \
  params_file:="$PARAMS" lidar_config:=$PKG/config/real/mid360_user_config.json \
  gimbal_joint_type:=revolute gimbal_is_root:=true > .dsh_run/e2e_rsp.log 2>&1 &
RSP=$!
sleep 6
ros2 launch srm27_nav_bringup nav_srm_real_launch.py slam:=False use_pcd_localization:=False \
  use_lio_odometry:=True use_zlocalization_backend:=False odometry_frontend:=zlio \
  zlio_config:=$PKG/config/real/zlio_mid360.yaml map:=$MAP prior_pcd_file:=$PCD \
  params_file:=$PARAMS speed_limits_file:=$PKG/config/real/speed_limits.yaml \
  use_sim_time:=False autostart:=true use_composition:=True use_respawn:=false \
  use_fake_vel_transform:=True command_frame:=gimbal_yaw_fake use_chassis_mux:=True \
  mux_nav_topic:=cmd_vel_nav_body mux_output_topic:=cmd_vel_chassis \
  gimbal_joint_topic:=gimbal_joint_angle use_velocity_smoother:=True \
  cmd_vel_nav_topic:=cmd_vel_nav_fake yaw_mode:=xy_only \
  map_to_odom_x:=0.0 map_to_odom_y:=0.0 map_to_odom_yaw:=0.0 log_level:=info \
  use_rviz:=False > .dsh_run/e2e_nav.log 2>&1 &
NAV=$!

echo "等待 45 s ..."
sleep 45
echo "=== lifecycle 状态 ==="
for n in map_server controller_server planner_server bt_navigator behavior_server velocity_smoother waypoint_follower smoother_server; do
  printf '%-20s %s\n' "$n" "$(timeout 3 ros2 lifecycle get /$n 2>&1 | head -1)"
done
echo "=== TF map -> gimbal_yaw_fake ==="
timeout 6 ros2 run tf2_ros tf2_echo map gimbal_yaw_fake 2>&1 | head -8
echo "=== TF map -> base_link ==="
timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -8
echo "=== /odometry 频率 ==="
timeout 6 ros2 topic hz /odometry 2>&1 | tail -2
echo "=== nav 日志里的错误 (zlio/tf/plugin) ==="
grep -nE "ERROR|zlio|Failed|died" .dsh_run/e2e_nav.log | head -20

kill -INT $NAV $RSP $DRV 2>/dev/null
sleep 6
pkill -INT -f "nav_srm_real_launch|real_robot_state_publisher_launch|zlio_node|fake_vel_transform" 2>/dev/null
sleep 4
pkill -9 -f "nav_srm_real_launch|real_robot_state_publisher_launch|zlio_node|livox_ros_driver2|component_container" 2>/dev/null
echo "done"
