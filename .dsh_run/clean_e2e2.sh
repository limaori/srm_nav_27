#!/usr/bin/env bash
# 干净域验证: 雷达 + 车体TF + 导航栈, ROS_DOMAIN_ID=7 (避开用户 13:30 那套残留)
set -u
export ROS_DOMAIN_ID=7
cd /home/srm/srm_nav_27
export ROS_LOG_DIR=/home/srm/srm_nav_27/.dsh_run/roslog_clean
mkdir -p "$ROS_LOG_DIR"
set +u; source install/setup.bash; set -u
PKG=install/srm27_nav_bringup/share/srm27_nav_bringup
PARAMS=$PKG/config/real/nav2_params_srm_minco_fakeyaw.yaml
MAP=/home/srm/srm_nav_27/maps/227_1113pro/227_1113pro.yaml
PCD=/home/srm/srm_nav_27/maps/pcd/227_1113pro_mapframe.pcd

ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_ros_driver2 --params-file "$PARAMS" \
  -p user_config_path:=$PKG/config/real/mid360_user_config.json > .dsh_run/c_driver.log 2>&1 &
DRV=$!
ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py use_sim_time:=False \
  params_file:="$PARAMS" lidar_config:=$PKG/config/real/mid360_user_config.json \
  gimbal_joint_type:=revolute gimbal_is_root:=true > .dsh_run/c_rsp.log 2>&1 &
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
  use_rviz:=False > .dsh_run/c_nav.log 2>&1 &
NAV=$!
# 底盘串口没插 -> 用 0 度假关节角代替 srm27_nav_protocol 的 0x0010 回传
ros2 topic pub -r 50 /gimbal_joint_angle std_msgs/msg/Float32 "{data: 0.0}" > .dsh_run/c_joint.log 2>&1 &
JNT=$!

echo "等待 40 s ..."
sleep 40
echo "=== 节点数 (应各 1 个) ==="
timeout 10 ros2 node list 2>/dev/null | grep -vE "transform_listener" | sort | uniq -c | sort -k2 | head -25
echo "=== /odometry 频率 ==="
timeout 8 ros2 topic hz /odometry 2>&1 | tail -2
echo "=== zlio 是否有输入中断 ==="
grep -cE "IMU timestamp moved backwards|input interrupted" .dsh_run/c_nav.log || true
echo "=== lifecycle ==="
for n in map_server controller_server planner_server bt_navigator behavior_server velocity_smoother; do
  for i in 1 2 3; do
    s=$(timeout 4 ros2 lifecycle get /$n 2>&1 | head -1)
    case "$s" in *active*) break ;; esac
    sleep 2
  done
  printf '%-18s %s\n' "$n" "$s"
done
echo "=== TF ==="
timeout 5 ros2 run tf2_ros tf2_echo map gimbal_yaw_fake 2>&1 | grep -A3 "At time" | head -6
echo "=== 代价地图是否 active ==="
timeout 4 ros2 lifecycle get /local_costmap/local_costmap 2>&1 | head -1
timeout 4 ros2 lifecycle get /global_costmap/global_costmap 2>&1 | head -1

kill -INT $NAV $RSP $DRV $JNT 2>/dev/null
sleep 8
pkill -INT -f "nav_srm_real_launch|real_robot_state_publisher_launch|zlio_node|livox_ros_driver2|gimbal_joint_angle" 2>/dev/null
sleep 4
pkill -9 -f "nav_srm_real_launch|real_robot_state_publisher_launch|zlio_node|livox_ros_driver2|component_container|bt_navigator|costmap|planner_server|controller_server" 2>/dev/null
echo done
