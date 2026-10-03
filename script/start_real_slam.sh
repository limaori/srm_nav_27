#!/usr/bin/env bash
set -eo pipefail

SLAM_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SLAM_WS_DIR="${SRM27_WS_DIR:-$SLAM_WS_DIR}"

if [[ ! -f "$SLAM_WS_DIR/install/local_setup.bash" ]]; then
  echo "未找到 $SLAM_WS_DIR/install/local_setup.bash，请先编译工作空间。" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$SLAM_WS_DIR/install/local_setup.bash"
cd "$SLAM_WS_DIR"
exec ros2 launch srm27_nav_bringup real_mapping_launch.py "$@"
