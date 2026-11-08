#!/usr/bin/env bash
# 隔离验证 v2: RSP 静态 TF + relay constant 模式
set +u
cd /home/srm/srm_nav_27
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77
export ROS_LOG_DIR=/home/srm/srm_nav_27/.mapcheck/ros_log
export ROS_HOME=/home/srm/srm_nav_27/.mapcheck/ros_home
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME" .mapcheck/log

ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py \
  use_sim_time:=False \
  params_file:=/home/srm/srm_nav_27/install/srm27_nav_bringup/share/srm27_nav_bringup/config/real/nav2_params_srm_minco_fakeyaw.yaml \
  gimbal_joint_type:=revolute gimbal_is_root:=true \
  > .mapcheck/log/rsp_test.log 2>&1 &
RSP=$!
sleep 10
echo "== 节点 =="; timeout 6 ros2 node list 2>/dev/null
echo "== /tf_static 帧对 =="
timeout 6 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id" | sed 's/^ *//'

echo "== 起 relay (constant) =="
ros2 run srm27_nav_bringup gimbal_joint_state_relay.py --ros-args \
  -p input_type:=constant -p constant_angle:=0.0 -p publish_rate:=100.0 \
  -p output_topic:=joint_states -p joint_name:=gimbal_yaw_joint \
  > .mapcheck/log/relay_test.log 2>&1 &
RL=$!
sleep 6
echo "== relay log =="; head -3 .mapcheck/log/relay_test.log
echo "== /joint_states =="; timeout 5 ros2 topic echo /joint_states --once 2>&1 | head -6
echo "== /tf =="; timeout 5 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child_frame_id" | sed 's/^ *//' | head -6
echo "== tf2_echo gimbal_yaw -> base_link =="
timeout 8 ros2 run tf2_ros tf2_echo gimbal_yaw base_link 2>&1 | grep -E "Translation|Rotation|Invalid|Could not" | head -4

kill $RL 2>/dev/null; kill $RSP 2>/dev/null
sleep 1
pkill -f srm27_robot 2>/dev/null
pkill -f robot_state_publisher 2>/dev/null
pkill -f gimbal_joint_state_relay 2>/dev/null
echo "== done =="
