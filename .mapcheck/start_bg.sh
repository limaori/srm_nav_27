#!/usr/bin/env bash
set +u
cd /home/srm/srm_nav_27
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/home/srm/srm_nav_27/.mapcheck/ros_log
export ROS_HOME=/home/srm/srm_nav_27/.mapcheck/ros_home
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME" .mapcheck/log
nohup ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/srm/srm_nav_27/maps/227_1108/227_1108.yaml \
  -p topic_name:=map -p frame_id:=map -p use_sim_time:=false > .mapcheck/log/map_server.log 2>&1 &
sleep 4
ros2 lifecycle set /map_server configure >/dev/null 2>&1
ros2 lifecycle set /map_server activate  >/dev/null 2>&1
sleep 2
echo "map_server: $(ros2 lifecycle get /map_server 2>&1 | tail -1)"
setsid nohup ros2 run rviz2 rviz2 -d src/srm27_navigation/srm27_nav_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=false > .mapcheck/log/rviz.log 2>&1 < /dev/null &
disown
echo started
