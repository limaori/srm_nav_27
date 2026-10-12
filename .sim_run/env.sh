# 仿真测试专用环境：把所有会写 $HOME / /tmp 的东西都指到工作区内的可写目录
source /opt/ros/humble/setup.bash
source /home/srm/srm_nav_27/install/setup.bash
export TMPDIR=/home/srm/srm_nav_27/.sim_run/tmp
export ROS_LOG_DIR=/home/srm/srm_nav_27/.sim_run/roslog
export ROS_HOME=/home/srm/srm_nav_27/.sim_run/ros
export XDG_DATA_HOME=/home/srm/srm_nav_27/.sim_run/xdg_data
export XDG_CACHE_HOME=/home/srm/srm_nav_27/.sim_run/xdg_cache
export XDG_CONFIG_HOME=/home/srm/srm_nav_27/.sim_run/xdg_config
