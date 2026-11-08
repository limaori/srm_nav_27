#!/usr/bin/env bash
# 隔离验证: revolute + gimbal_is_root + relay(constant) 能不能把 gimbal_yaw->base_link 接上
set +u
cd /home/srm/srm_nav_27
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77
export ROS_LOG_DIR=/home/srm/srm_nav_27/.mapcheck/ros_log
export ROS_HOME=/home/srm/srm_nav_27/.mapcheck/ros_home
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME" .mapcheck/log

echo "== 起 RSP (revolute, gimbal_is_root:=true) =="
ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py \
  use_sim_time:=False \
  params_file:=/home/srm/srm_nav_27/install/srm27_nav_bringup/share/srm27_nav_bringup/config/real/nav2_params_srm_minco_fakeyaw.yaml \
  gimbal_joint_type:=revolute gimbal_is_root:=true \
  > .mapcheck/log/rsp_test.log 2>&1 &
RSP=$!
sleep 6
echo "-- 先看: 没有 joint_states 时 --"
timeout 6 ros2 run tf2_ros tf2_echo gimbal_yaw base_link 2>&1 | head -6

echo "== 起 relay constant 0 度 =="
ros2 run srm27_nav_bringup gimbal_joint_state_relay.py --ros-args \
  -p input_topic:="" -p input_type:=constant -p constant_angle:=0.0 -p publish_rate:=100.0 \
  > .mapcheck/log/relay_test.log 2>&1 &
RL=$!
sleep 5
echo "-- relay 之后 --"
timeout 6 ros2 run tf2_ros tf2_echo gimbal_yaw base_link 2>&1 | head -8
echo "-- 顺带看 base_link 到 livox_imu 通不通 --"
timeout 6 ros2 run tf2_ros tf2_echo base_link livox_imu 2>&1 | grep -E "Translation|At time" | head -4

kill $RL 2>/dev/null; kill $RSP 2>/dev/null
sleep 1
pkill -f real_robot_state_publisher 2>/dev/null
pkill -f robot_state_publisher 2>/dev/null
pkill -f gimbal_joint_state_relay 2>/dev/null
echo "== done =="
