#!/usr/bin/env bash
# 启动 SRM Gazebo 仿真；世界与 GUI 默认值见 config/srm_sim.yaml。
# 只跑场地可用 ros2 launch srm27_gazebo_simulator gazebo.launch.py。
set -e
WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$WS_DIR/install/setup.bash"
ros2 launch srm27_gazebo_simulator srm_sim.launch.py "$@"
