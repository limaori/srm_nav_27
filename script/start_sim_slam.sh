#!/usr/bin/env bash
# =============================================================
# 仿真 SLAM 建图一键启动（Gazebo 世界 → Point-LIO 建 3D 点云 + slam_toolbox 建 2D 栅格）
#
# 链路（详见 srm27_nav_bringup/launch/sim_slam_launch.py 的注释）:
#   Gazebo gpu_lidar ─bridge─► livox/lidar ─ign_sim_pointcloud_tool─► velodyne_points
#   livox/imu ─────────────────────────────────────────────────────────┘
#                              ▼
#                          Point-LIO ──► odom->base_link（经 loam_interface + sensor_scan_generation）
#                              └────────► 3D 点云地图: PCD/scans.pcd（退出时落盘）
#                          slam_toolbox ──► 2D 栅格 map（另存 pgm/yaml）
#
# 用法:
#   ./script/start_sim_slam.sh                        # 默认 rmuc_2025 世界
#   ./script/start_sim_slam.sh -w rmuc_2025           # 指定世界
#   ./script/start_sim_slam.sh -w rmuc_2025 --no-gui  # 关掉 Gazebo GUI，省一大半 GPU
#   ./script/start_sim_slam.sh --no-clean             # 不清理上次残留
#   DRY_RUN=1 ./script/start_sim_slam.sh              # 只打印将执行的命令
#
#   建完图之后（关掉 slam 标签页，Point-LIO 收到 Ctrl-C 才落盘）:
#   ./script/start_sim_slam.sh --collect              # 把 scans.pcd 收回 maps/pcd/
#   ./script/start_sim_slam.sh --collect --name rmuc_2025
#   ./script/start_sim_slam.sh --view maps/pcd/rmuc_2025.pcd   # 在 RViz 里看这个 PCD
#
# 参数:
#   -w, --world <name>     rmuc_2025 / rmuc_2024 / rmul_2024 / rmul_2025 / srm_empty
#   --gui / --no-gui       Gazebo 是否带 GUI (默认 no-gui，建图只看 RViz 更流畅)
#   --rviz / --no-rviz     是否启动 RViz (默认启动)
#   --params <路径>        用别的参数文件 (默认 config/simulation/nav2_params_srm.yaml)
#   --no-clean             跳过启动前的 kill_gzb/kill_nav/kill_rviz
#   -o, --out <目录>       --collect 的输出目录 (默认 <工作空间>/maps/pcd)
#   -n, --name <名字>      --collect 的输出文件名 (默认 sim_<世界>_<时间戳>)
#   --collect              把 point_lio/PCD/scans.pcd 拷到输出目录并报告
#   --view <文件.pcd>      用 pcl_ros 把 PCD 发成 PointCloud2，配合 RViz 查看
#
# 环境变量:
#   --min-free-gb <数值>   启动前要求的最小可用内存（GB），默认 3；0 = 不检查
#   PREKILL=0              等价于 --no-clean
#   DRY_RUN=1              只打印命令
#   TERMINAL               终端程序 (默认 gnome-terminal)
# =============================================================

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

WORLD="${WORLD:-rmuc_2025}"
USE_GUI="${USE_GUI:-false}"
USE_RVIZ="${USE_RVIZ:-True}"
PARAMS_FILE="${PARAMS_FILE:-}"
ROBOT_NS="${ROBOT_NS:-red_standard_robot1}"
TERMINAL="${TERMINAL:-gnome-terminal}"
DRY_RUN="${DRY_RUN:-0}"
PREKILL="${PREKILL:-1}"
OUT_DIR="${OUT_DIR:-$WS_DIR/maps/pcd}"
CLIP_NAME="${CLIP_NAME:-}"
ACTION="start"
MIN_FREE_GB="${MIN_FREE_GB:-3}"

# Point-LIO 的落盘路径是编译期常量 ROOT_DIR（见 point_lio/CMakeLists.txt）
PCD_SRC="$WS_DIR/src/srm27_navigation/point_lio/PCD/scans.pcd"

log() { printf '%s\n' "$*"; }
die() { printf '[错误] %s\n' "$*" >&2; exit 1; }

usage() { sed -n '2,45p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    -w|--world)   WORLD="${2:?--world 需要世界名}"; shift 2 ;;
    --gui)        USE_GUI="true"; shift ;;
    --no-gui)     USE_GUI="false"; shift ;;
    --rviz)       USE_RVIZ="True"; shift ;;
    --no-rviz)    USE_RVIZ="False"; shift ;;
    --params)     PARAMS_FILE="${2:?--params 需要绝对路径}"; shift 2 ;;
    --no-clean)   PREKILL="0"; shift ;;
    -o|--out)     OUT_DIR="${2:?--out 需要目录}"; shift 2 ;;
    -n|--name)    CLIP_NAME="${2:?--name 需要名字}"; shift 2 ;;
    --min-free-gb) MIN_FREE_GB="${2:?--min-free-gb 需要数值(GB)}"; shift 2 ;;
    --collect)    ACTION="collect"; shift ;;
    --view)       ACTION="view"; VIEW_FILE="${2:?--view 需要 .pcd 路径}"; shift 2 ;;
    -h|--help)    usage; exit 0 ;;
    *)            die "未知参数: $1（--help 看用法）" ;;
  esac
done

# ---------------------------------------------------------------- --collect
if [ "$ACTION" = "collect" ]; then
  [ -f "$PCD_SRC" ] || die "找不到点云文件: $PCD_SRC
  它只在 Point-LIO **正常退出** 时才会生成 —— 建图结束后要 Ctrl-C 掉 slam 那个标签页，
  看到 'current scan saved to ...' 之类的日志再回来收。"
  mkdir -p "$OUT_DIR" || die "无法创建输出目录: $OUT_DIR"
  if [ -z "$CLIP_NAME" ]; then
    CLIP_NAME="sim_${WORLD}_$(date +%Y%m%d_%H%M%S)"
  fi
  DEST="$OUT_DIR/${CLIP_NAME}.pcd"
  cp -f "$PCD_SRC" "$DEST" || die "拷贝失败: $PCD_SRC -> $DEST"
  SIZE="$(du -h "$DEST" | cut -f1)"
  log "已收集 3D 点云: $DEST  ($SIZE)"
  log ""
  log "在 RViz 里查看它:"
  log "  ./script/start_sim_slam.sh --view \"$DEST\""
  log "（另开一个 RViz，加 PointCloud2 显示，Topic 选 /cloud_pcd）"
  exit 0
fi

# ---------------------------------------------------------------- --view
if [ "$ACTION" = "view" ]; then
  [ -f "$VIEW_FILE" ] || die "找不到 PCD: $VIEW_FILE"
  command -v ros2 >/dev/null 2>&1 || die "没找到 ros2，先 source /opt/ros/humble/setup.bash"
  log "把 $VIEW_FILE 发成 PointCloud2（话题 /cloud_pcd，frame map）"
  log "另开 RViz: Fixed Frame 设 map，加 PointCloud2 显示，Topic 选 /cloud_pcd"
  log "Ctrl-C 结束发布。"
  # 注意: pcl_ros 的 pcd_to_pointcloud 用 tf_frame 指定 frame_id，
  # 写成 frame_id:= 会被忽略并退化成 /base_link。
  exec ros2 run pcl_ros pcd_to_pointcloud --ros-args \
    -p file_name:="$VIEW_FILE" -p tf_frame:=map -p rate:=1.0
fi

# ---------------------------------------------------------------- 启动
[ -n "$PARAMS_FILE" ] || PARAMS_FILE="$WS_DIR/src/srm27_navigation/srm27_nav_bringup/config/simulation/nav2_params_srm.yaml"
[ -f "$PARAMS_FILE" ] || die "参数文件不存在: $PARAMS_FILE"

if [ "$DRY_RUN" != "1" ] && ! command -v "$TERMINAL" >/dev/null 2>&1; then
  die "未找到终端程序: $TERMINAL（可用 TERMINAL=konsole 等指定）"
fi

open_term() {
  local title="$1" cmd="$2"
  local full_cmd="cd '$WS_DIR' && source install/setup.bash && ${cmd}; exec bash"
  if [ "$DRY_RUN" = "1" ]; then
    log "--- [dry-run] 标签页: $title"
    log "$full_cmd"
    return 0
  fi
  "$TERMINAL" --tab --title="$title" -- bash -c "$full_cmd"
}

prekill_leftovers() {
  if [ "$DRY_RUN" = "1" ]; then
    log "[dry-run] 跳过启动前清理。"; return 0
  fi
  if [ "$PREKILL" = "0" ]; then
    log "[提示] 跳过启动前清理。"; return 0
  fi
  log "[清理] 结束残留的 Gazebo / 导航栈 / RViz..."
  local name
  for name in kill_nav.sh kill_gzb.sh kill_rviz.sh; do
    [ -x "$SCRIPT_DIR/$name" ] || { log "[警告] 缺少 $name，跳过" >&2; continue; }
    "$SCRIPT_DIR/$name" || log "[警告] $name 退出码非 0，继续启动。" >&2
  done
}

prekill_leftovers

# ---------- 内存预检 ----------
# 这台机器实测被 OOM killer 杀过 nav2 容器(3.3 GB) 和 slam_toolbox(3.8 GB)：
# 两个一起崩 -> 地图停止更新、odom->base_link 消失，RViz 里就表现为"地图被拉长/漂飞"。
# 建图链路常驻内存大约需要：Gazebo 1.5~2 GB + Point-LIO 0.5 GB + nav2 容器 2~3 GB
# + RViz 0.5~1 GB，所以启动前先看可用内存，不够就明确拦下来而不是让它中途被 OOM 杀掉。
check_memory() {
  [ "$MIN_FREE_GB" = "0" ] && return 0
  local avail_gb
  avail_gb="$(awk '/MemAvailable/ {printf "%.1f", $2/1048576}' /proc/meminfo)"
  local swap_used_pct
  swap_used_pct="$(awk '/SwapTotal/ {t=$2} /SwapFree/ {f=$2} END {if (t>0) printf "%d", (t-f)*100/t; else print 0}' /proc/meminfo)"
  log "[检查] 可用内存 ${avail_gb} GB (要求 ≥ ${MIN_FREE_GB} GB)，swap 使用率 ${swap_used_pct}%"
  if awk -v a="$avail_gb" -v m="$MIN_FREE_GB" 'BEGIN {exit !(a < m)}'; then
    log "[警告] 可用内存不足 ${MIN_FREE_GB} GB，建图很可能被 OOM killer 杀掉。" >&2
    log "        建议：关掉 Gazebo GUI(--no-gui)、关浏览器/IDE、或先扩 swap。" >&2
    log "        确认强行启动：--min-free-gb 0" >&2
    [ "$DRY_RUN" = "1" ] && return 0
    exit 1
  fi
}
check_memory

# 上次的 scans.pcd 先挪走，免得这次没落盘时 --collect 收错文件
if [ "$DRY_RUN" != "1" ] && [ -f "$PCD_SRC" ]; then
  BAK="${PCD_SRC%.pcd}.prev_$(date +%H%M%S).pcd"
  mv -f "$PCD_SRC" "$BAK" 2>/dev/null && log "[提示] 旧点云已备份为: $BAK"
fi

log ""
log "========== SRM27 仿真 SLAM 建图 (世界: $WORLD) =========="
log "工作空间  : $WS_DIR"
log "参数文件  : $PARAMS_FILE"
log "Gazebo GUI: $USE_GUI      RViz: $USE_RVIZ"
log "点云落盘  : $PCD_SRC"
log ""

GAZEBO_CMD="ros2 launch srm27_gazebo_simulator srm_sim.launch.py world:=${WORLD} gui:=${USE_GUI} run_immediately:=true"
RVIZ_ARG=""
[ "$USE_RVIZ" = "False" ] && RVIZ_ARG=" rviz:=false"
SLAM_CMD="ros2 launch srm27_nav_bringup sim_slam_launch.py namespace:=${ROBOT_NS} world:=${WORLD} params_file:=\"${PARAMS_FILE}\"${RVIZ_ARG}"

log "[1/3] Gazebo 世界 + 车 + 传感器桥..."
open_term "SRM27 建图-Gazebo" "$GAZEBO_CMD"

if [ "$DRY_RUN" != "1" ]; then
  log "[提示] 等 Gazebo 起来（约 8 s）再拉建图栈..."
  sleep 8
fi

log "[2/3] 建图栈: Point-LIO + loam_interface + sensor_scan_generation + slam_toolbox + RViz..."
open_term "SRM27 建图-SLAM" "$SLAM_CMD"

log "[3/3] 操作指引"
log ""
cat <<'TIPS'
驱动车走一圈（建图要靠运动把扫描环补成面）：
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -r cmd_vel:=/red_standard_robot1/cmd_vel_sim

确认里程计和地图在动：
  ros2 topic hz   /red_standard_robot1/odometry
  ros2 topic echo /red_standard_robot1/odometry --once
  ros2 topic echo /red_standard_robot1/map --once --field info

建图结束（关键：必须让 Point-LIO 正常退出，它才会把点云写盘）：
  1) 在 "SRM27 建图-SLAM" 标签页按 Ctrl-C，等它打印保存日志；
  2) 回到这里执行：
       ./script/start_sim_slam.sh --collect --name rmuc_2025
     或直接看它在哪：
       ls -lh src/srm27_navigation/point_lio/PCD/scans.pcd

保存 2D 栅格地图（可选，slam_toolbox 的 map_saver）：
  ros2 service call /red_standard_robot1/map_saver/save_map nav2_msgs/srv/SaveMap \
    "{map_topic: map, map_url: /home/srm/srm_nav_27/maps/simulation/rmuc_2025_slam, \
      image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"

清理残留：
  ./script/kill_gzb.sh && ./script/kill_rviz.sh
TIPS
log ""
log "点云地图怎么在 RViz 里看:"
log "  ./script/start_sim_slam.sh --view maps/pcd/rmuc_2025.pcd"
log "  再开一个 RViz，Fixed Frame 设 map，加 PointCloud2，Topic 选 /cloud_pcd"
