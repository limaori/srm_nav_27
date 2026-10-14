# 沙盒专用环境（仅 AI 会话 source；用户终端不需要）：
# 1) /run 与 ~/.ros 只读 → XDG_RUNTIME_DIR / ROS_LOG_DIR / ROS_HOME 指进工作区
# 2) 继承环境可能带其它 workspace 前缀 → 先清空再 source 本工作空间
export XDG_RUNTIME_DIR="/home/srm/srm_nav_27/.sim_run/xdg"
export ROS_LOG_DIR="/home/srm/srm_nav_27/.sim_run/ros_log"
export ROS_HOME="/home/srm/srm_nav_27/.sim_run/ros_home"
mkdir -p "$XDG_RUNTIME_DIR" "$ROS_LOG_DIR" "$ROS_HOME"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH
source /home/srm/srm_nav_27/install/setup.bash
