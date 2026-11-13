#!/usr/bin/env bash
# =============================================================
# SRM 实车 SLAM 建图 (一键启动 + 一键存图)
#
# 建图链路 (real_mapping_launch.py 一个 launch 起全部):
#   步骤 1: Livox Mid-360 驱动 (start_lidar:=True)
#   步骤 2: 车体 TF robot_state_publisher (base_link / livox_frame / livox_imu / livox_scan)
#   步骤 3: Point-LIO 激光惯性里程计 (只出里程计, 不发布 map->odom)
#   步骤 4: loam_interface (aft_mapped_to_init -> odom) + sensor_scan_generation (发布 odom->base_link)
#   步骤 5: pointcloud_to_laserscan (registered_scan -> /scan)
#   步骤 6: slam_toolbox (mode: mapping, 唯一发布 map->odom 的节点)
#   步骤 7: RViz (mapping.rviz)
#   另外单独起 (不在这条 launch 里):
#   步骤 8: 底盘串口 srm27_nav_protocol (cmd_vel_chassis 的唯一消费者, 建图时必须能开车)
#   步骤 9: 手柄遥控 (可选, --joy; 默认关闭)
#
# 存图 (--save <名字>, 需要建图链路正在运行; 可与启动分开、另开终端执行):
#   slam_toolbox 两个服务各写一半, 合起来才是完整地图:
#     /slam_toolbox/save_map      -> maps/<名字>/<名字>.pgm + <名字>.yaml
#                                    (Nav2 用的二维栅格图)
#     /slam_toolbox/serialize_map -> maps/<名字>/<名字>.posegraph + <名字>.data
#                                    (位姿图, 续建图 / 后续定位用)
#   与工作区现有 maps/xjl0914、maps/227_0916 的目录布局一致。
#   save_map 依赖参数 use_map_saver: true (config/real/mapping_params.yaml 已配)。
#   --save 只是调这两个在线服务, 不自己起链路: 必须先有正在运行的建图链路,
#   否则脚本会明确告诉你是"没跑"还是"跑了但没开 use_map_saver"。
#
# ⚠ 安全须知 (务必先读):
#   1) srm27_nav_protocol 带看门狗 (协议 §6.3): cmd_timeout_sec (默认 0.5 s) 内没收到新的
#      cmd_vel 就把控制量归零, 所以"发速度的节点退出后车会一直跑"的旧说法已经不成立。
#      但看门狗只是兜底: 停任何节点前仍然先发零速, 物理急停按钮始终是第一手段。
#      本脚本的 --stop 会先发零速再结束进程。
#   2) map->odom 只能有一个发布者: 建图时不要同时跑导航栈
#      (start_real_nav.sh / nav2_stack_launch.py / nav2_slam_launch.py),
#      否则两个节点抢同一条 TF, RViz 里地图会来回跳。本脚本启动前会检查并拒绝。
#   3) 建图前先在 RViz 用 base_link 或 odom 作固定坐标系确认点云不重影;
#      重影 = 雷达外参/时间戳不对, 这时建出来的图必然错。
#
# 用法:
#   ./script/start_real_slam.sh                      # 起建图链路 (默认不含手柄)
#   ./script/start_real_slam.sh --joy                 # 顺带起手柄 (推杆走车, 边走边建图)
#   ./script/start_real_slam.sh --no-chassis          # 不起底盘串口 (车不打算动/已有别处起)
#   ./script/start_real_slam.sh --no-rviz             # 无图形界面 (无头 NUC)
#   ./script/start_real_slam.sh --save 227_1006       # 另开终端: 存图到 maps/227_1006/
#   ./script/start_real_slam.sh --save site01 --force # 同名地图已存在时覆盖
#   ./script/start_real_slam.sh --record-bag          # 建图同时录原始点云/IMU (事后可重建 PCD)
#   ./script/start_real_slam.sh --record-bag --bag-name 227_1010 --bag-derived
#   ./script/start_real_slam.sh -n                    # 只解析并打印将执行的命令, 不启动
#   ./script/start_real_slam.sh --list-maps           # 列出工作区现有地图
#   ./script/start_real_slam.sh --stop                # 先发零速, 再结束建图链路节点
#
# 参数:
#       --save <名字>           保存当前地图到 maps/<名字>/ (要求建图链路正在跑)
#       --out-dir <绝对路径>    存图输出目录, 默认 <工作空间>/maps/<名字>
#       --wait <秒>             存图前最多等 slam_toolbox 服务出现的秒数 (默认 8; 0 = 不等)
#       --force                 存图时允许覆盖已存在的同名地图文件
#       --record-bag            建图同时录包 (另起标签页跑 script/record_mapping_bag.sh)
#       --bag-name <名字>       录包目录名带上地图名, 建议与稍后 --save 的名字一致
#       --bag-derived           录包时追加 /cloud_registered /aft_mapped_to_init /scan /map
#       --bag-out-dir <绝对路径> 录包输出目录, 默认 <工作空间>/log_diag/mapping_bag_<名字|时间戳>
#   -p, --params <绝对路径>     建图参数, 默认 config/real/mapping_params.yaml
#       --lidar-config <路径>   雷达驱动网络配置, 默认 config/real/mid360_user_config.json
#       --lidar-xyz "X Y Z"     临时覆盖雷达安装位置, 米 (默认空 = 用 URDF 模型里的值)
#       --lidar-rpy "R P Y"     临时覆盖雷达安装角, 弧度 (默认空 = 用 URDF 模型里的值)
#       --no-lidar              不起雷达驱动 (回放 rosbag 时用, 配合 --use-sim-time)
#       --use-sim-time          使用 /clock (回放 bag 时用, 默认 False)
#       --lio-frontend <point_lio|zlio>  里程计前端（**默认 zlio**）；
#                               zlio = frontend/zlio_node + zlio_bridge（与导航链一致）
#       --scan-cloud <raw|registered>  2D 栅格用哪路点云（默认 **raw**）
#                               raw = /livox/lidar/pointcloud（~20k 点/帧，720 个方位格覆盖 ~600，
#                                     矮台阶/薄栏杆能进图；代价：未做运动去畸变）
#                               registered = registered_scan（已去畸变，但只有 ~1.7k 点/帧、
#                                     覆盖 ~264 格，薄/矮障碍常被相邻射线当空闲刷掉）
#       --save-pcd              退出时落盘累积点云（point_lio 自己写，zlio 由 zlio_bridge 写）
#       --save-pcd-path <路径>  自定义落盘路径，默认 src/srm27_navigation/point_lio/PCD/scans.pcd
#       --rviz / --no-rviz      是否启动 RViz (默认启动)
#       --chassis / --no-chassis    是否启动底盘串口 (默认启动)
#       --joy / --no-joy        是否启动手柄遥控 (默认关闭)
#       --joy-dev <N>           手柄设备号 (默认 0)
#   -n, --check                 只解析并打印将执行的命令 (等同 DRY_RUN=1)
#       --list-maps             列出工作区现有地图后退出
#       --stop                  发零速后结束建图链路进程 (不起新节点)
#   -h, --help                  显示本帮助
#
# 环境变量 (同名参数均可由环境变量给默认值):
#   SAVE_MAP_NAME MAP_OUT_DIR FORCE_SAVE WAIT_FOR_SLAM PARAMS_FILE LIDAR_CONFIG_FILE
#   RECORD_BAG BAG_NAME BAG_DERIVED BAG_OUT_DIR BAG_TOPICS BAG_MIN_FREE_GB BAG_FINALIZE_WAIT
#   LIDAR_XYZ LIDAR_RPY START_LIDAR USE_SIM_TIME SAVE_PCD SCAN_CLOUD
#   USE_RVIZ USE_CHASSIS USE_JOY JOY_DEV
#   MAPPING_EXTRA_ARGS  额外透传给 real_mapping_launch.py 的参数, 例如
#                       "scan_cloud_topic:=/livox/lidar/pointcloud" (2D 扫描改用原始点云)
#   OPEN_MODE(tab|window) TERMINAL DRY_RUN SRM27_WS_DIR PREKILL
#
# 说明:
#   - 启动前会自动先跑一遍 script/kill_gzb.sh 和 script/kill_rviz.sh, 清掉上次
#     遗留的 Gazebo / RViz (不清会抢 /clock、抢 RViz 窗口)。清理失败只告警不阻断;
#     想完全跳过这一步用 PREKILL=0。--save / --stop 分支不会触发这个清理。
#   - 雷达驱动、车体 TF、Point-LIO、slam_toolbox 全在 real_mapping_launch.py 里,
#     本脚本只负责"参数校验 + 补齐底盘/手柄 + 存图/停止", 不再重复起这些节点。
#   - 雷达安装位姿的**唯一来源是 URDF** (srm27_robot_description/urdf/srm27_sentry.urdf.xacro):
#     脚本默认**不传** lidar_xyz/lidar_rpy, 链路直接用模型里的值, 所以换 URDF 不用改这里。
#     启动摘要里会把模型算出来的 xyz/rpy/gravity 打出来, 换完模型第一眼就能核对。
#     --lidar-xyz / --lidar-rpy 只在现场临时改安装角(模型来不及改)时用, 会覆盖模型。
#   - 雷达驱动外参必须为零, 否则点云被驱动和 URDF 各转一次, 建图必然重影;
#     launch 会直接拒绝, 本脚本提前报错。
#   - 所有路径一律传绝对路径; 每个标签页都会先 source 工作空间的 install/setup.bash。
#   - 累积点云 (--save-pcd) 落在 src/srm27_navigation/point_lio/PCD/scans.pcd
#     （--save-pcd-path 可改）。两个前端都支持：
#       · point_lio: 由 point_lio 自己在正常退出时写（PCD/scans.pcd 是它的编译期路径）；
#       · zlio     : 由 zlio_bridge 累积 registered_scan 写（体素 0.10 m 降采样）。
#     两者都在**建图起点系**（Point-LIO camera_init / ZLIO 的 odom），
#     **不是**地图坐标系下的先验点云，直接拿给 --reloc 用会错位；
#     需要先 python3 script/pcd_to_mapframe.py ... 转到地图坐标系。
#   - --record-bag 另起一个标签页跑 script/record_mapping_bag.sh, 录
#     /livox/lidar(CustomMsg, 含逐点 offset_time) + /livox/imu + /tf + /tf_static,
#     约 240 MB/分钟 (Mid-360 满速)。这是唯一能事后重建三维 PCD 的原始数据:
#     slam_toolbox 的 .posegraph/.data 只存二维位姿图, 里面没有点云, 事后转不出来。
#     结束录包必须走 Ctrl+C 或 --stop (rosbag2 靠 SIGINT 才会写 metadata.yaml,
#     被 kill -9 的包无法回放)。
# =============================================================

set -euo pipefail

# ---------- 路径 ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"

# ---------- 环境消毒 ----------
# 海康 MVS 客户端会把 /opt/MVS/lib/64 前置进 LD_LIBRARY_PATH，那里那份老 libusb-1.0
# 缺 libusb_set_option()，任何链接 PCL 的程序（建图用的 point_lio 最典型）一加载就以
# exit 127 静默退出 —— 现场表现是"建图节点没起来、没有点云也没有里程计"。
# 只改本脚本及其子进程（含它开的标签页），不碰用户当前终端。
# 关掉: SRM_KEEP_MVS_LIB_PATH=1；额外剔除: SRM_LIB_PATH_STRIP_EXTRA=/p1,/p2
. "$SCRIPT_DIR/lib/srm_lib_path_guard.sh"

PKG_SRC="$WS_DIR/src/srm27_navigation/srm27_nav_bringup"
PKG_INSTALL="$WS_DIR/install/srm27_nav_bringup/share/srm27_nav_bringup"
# 优先用安装后的 share 目录 (launch 运行时 get_package_share_directory 返回的就是它),
# 找不到再退回源码目录 (未编译时也能做 --check / --list-maps)。
pick_dir() {
  local d
  for d in "$@"; do
    if [ -d "$d" ]; then
      printf '%s' "$d"
      return 0
    fi
  done
  printf '%s' "$1"
}
PKG_DIR="$(pick_dir "$PKG_INSTALL" "$PKG_SRC")"
MAP_PARAMS_DEFAULT="$PKG_DIR/config/real/mapping_params.yaml"
LIDAR_CONFIG_DEFAULT="$PKG_DIR/config/real/mid360_user_config.json"
# 手柄按键映射在 nav2 参数文件里, 不在 mapping_params.yaml 里: joy_teleop_launch.py
# 按 root_key 取 srm27_teleop_twist_joy_node 段, mapping_params.yaml 里没有这一段,
# 指过去节点会拿不到按键映射 (推杆无反应), 所以这里固定用实车 nav2 参数文件。
JOY_PARAMS_DEFAULT="$PKG_DIR/config/real/nav2_params_srm.yaml"
PCD_SAVE_PATH="$WS_DIR/src/srm27_navigation/point_lio/PCD/scans.pcd"

# ---------- 可配置项 ----------
OPEN_MODE="${OPEN_MODE:-tab}"                # tab: 同一窗口多标签页; window: 每个命令独立窗口
TERMINAL="${TERMINAL:-gnome-terminal}"
DRY_RUN="${DRY_RUN:-0}"

USE_RVIZ="${USE_RVIZ:-1}"
USE_CHASSIS="${USE_CHASSIS:-1}"
USE_JOY="${USE_JOY:-0}"
JOY_DEV="${JOY_DEV:-0}"
START_LIDAR="${START_LIDAR:-1}"
USE_SIM_TIME="${USE_SIM_TIME:-0}"
SAVE_PCD="${SAVE_PCD:-0}"
# 累积点云落盘路径（空 = 用 launch 的默认: point_lio/PCD/scans.pcd）
PCD_SAVE_PATH_CLI="${PCD_SAVE_PATH_CLI:-}"
# 里程计/配准点云前端：zlio（默认，与 start_real_nav.sh 一致）或 point_lio（回退）。
LIO_FRONTEND="${LIO_FRONTEND:-zlio}"
ZLIO_CONFIG="${ZLIO_CONFIG:-}"
# 2D 栅格吃哪路点云：raw（默认，原始 ~20k 点/帧）或 registered（LIO 的 ~1.7k 点/帧）。
# 原因见 real_mapping_launch.py 的 scan_cloud_topic：稀疏云会让相邻射线把薄/矮障碍刷成空闲。
# 留空 = 按是否起雷达决定（起雷达 raw；--no-lidar 回放时只能 registered）。
SCAN_CLOUD="${SCAN_CLOUD:-}"

PARAMS_FILE="${PARAMS_FILE:-}"
LIDAR_CONFIG_FILE="${LIDAR_CONFIG_FILE:-}"
# 雷达安装位姿的**唯一来源是 URDF** (real_mapping_launch.py -> srm_robot_description.py)。
# 这两个值默认留空 = 不给 launch 传 lidar_xyz/lidar_rpy, 链路就用模型里的默认值,
# 所以换了 URDF 只需要改模型, 不用改脚本。只有现场临时改安装角才显式传。
# 用 ${VAR-} 而不是 ${VAR:-}: 后者会把"显式给的空串"又吃回默认值。
LIDAR_XYZ="${LIDAR_XYZ-}"
LIDAR_RPY="${LIDAR_RPY-}"

SAVE_MAP_NAME="${SAVE_MAP_NAME:-}"
MAP_OUT_DIR="${MAP_OUT_DIR:-}"
FORCE_SAVE="${FORCE_SAVE:-0}"
# 存图前最多等 slam_toolbox 服务出现的秒数: 链路刚起时服务/发现都要一点时间,
# 一锤子判断会得到"找不到服务"的误报。0 = 不等, 立刻判断。
WAIT_FOR_SLAM="${WAIT_FOR_SLAM:-8}"
# slam_toolbox 节点名 (real_mapping_launch.py 里写死), 服务名随之确定。
SLAM_NODE_NAME="slam_toolbox"
SLAM_SERVICE="/slam_toolbox/save_map"

# 建图同步录包: 由 script/record_mapping_bag.sh 承担, 这里只决定要不要起、起在哪。
# 录的是原始点云/IMU, 是事后重建三维 PCD 的唯一来源 (见文件头说明)。
RECORD_BAG="${RECORD_BAG:-0}"
BAG_NAME="${BAG_NAME:-}"
BAG_DERIVED="${BAG_DERIVED:-0}"
BAG_OUT_DIR="${BAG_OUT_DIR:-}"
# --stop 等录包进程收尾 (写 metadata.yaml) 的最长秒数: 大包里这一步要刷缓存,
# 5 秒那轮统一超时不够, 单独给它留时间, 免得被后面那轮 SIGKILL 打断收尾。
BAG_FINALIZE_WAIT="${BAG_FINALIZE_WAIT:-30}"
BAG_SCRIPT="$SCRIPT_DIR/record_mapping_bag.sh"

# ---------- 进程识别 ----------
# 方括号包住首字符, 避免 pgrep 匹配到脚本自身命令行。
MAPPING_PATTERN='[r]eal_mapping_launch\.py'
NAV_PATTERN='[n]av2_stack_launch\.py|[n]av_real_launch\.py|[n]av2_slam_launch\.py|[s]tart_real_nav\.sh'
CHASSIS_PATTERN='[s]rm27_nav_protocol'
JOY_PATTERN='[j]oy_teleop_launch\.py'
# 录包进程有两条命令行要一起认: 外层是脚本本身, 内层是它 exec 出来的 ros2。
# 两个都得收到 SIGINT —— 只杀掉外层脚本的话, ros2 会变成孤儿继续录,
# 而 metadata.yaml 永远不写, 这一趟录制整体作废。
BAG_PATTERNS=(
  '[r]ecord_mapping_bag\.sh'
  '[r]os2 bag record'
)
# 其中真正写 metadata.yaml 的那个: --stop 的收尾等待只认它。
BAG_RECORDER_PATTERN='[r]os2 bag record'
# 停止时按名字兜底清理 (launch 被强杀时子节点可能残留)。
RESIDUAL_PATTERNS=(
  '[p]ointlio_mapping'
  '[s]ync_slam_toolbox_node'
  '[l]ivox_ros_driver2'
  '[l]oam_interface_node'
  '[s]ensor_scan_generation_node'
  '[p]ointcloud_to_laserscan_node'
  '[r]obot_state_publisher'
  '[r]viz2'
)

# ---------- 帮助 ----------
usage() {
  # 打印文件头注释块 (跳过 shebang 与首尾分隔线), 避免帮助文本与代码不同步。
  awk 'NR == 1 { next }
       /^# =+[[:space:]]*$/ { sep++; if (sep == 2) exit; next }
       { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

# ---------- 小工具 ----------
is_true() {
  case "$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')" in
    1|true|yes|on) return 0 ;;
    *)             return 1 ;;
  esac
}

# 归一化为 launch 需要的 True/False
bool_arg() {
  if is_true "$1"; then printf 'True'; else printf 'False'; fi
}

q() { printf '%q' "$1"; }

process_running() {
  pgrep -u "$(id -u)" -f "$1" >/dev/null 2>&1
}

# 在 source 过工作空间环境的子 shell 里执行命令 (脚本自身不 source, 避免污染)。
ros2_run() {
  (
    set +u
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash" >/dev/null 2>&1
    "$@"
  )
}

list_maps() {
  echo "工作区现有地图 (名称可直接作为 --save 的目标名或 start_real_nav.sh -m 的值):"
  echo
  local found=0 f name dir extra
  for f in "$WS_DIR"/maps/*/*.yaml "$WS_DIR"/maps/*.yaml; do
    [ -f "$f" ] || continue
    case "$f" in
      "$WS_DIR"/maps/*/*)
        [ "$(basename "$(dirname "$f")")" = "$(basename "$f" .yaml)" ] || continue
        ;;
    esac
    name="$(basename "$f" .yaml)"
    dir="$(dirname "$f")"
    extra=""
    [ -f "$dir/$name.posegraph" ] && extra="+位姿图"
    printf '  %-24s %s\n' "$name" "$f$extra"
    found=1
  done
  if [ "$found" = "0" ]; then
    echo "  (未找到任何 .yaml 地图)"
  fi
  echo
  echo "地图目录: $WS_DIR/maps  (布局: maps/<名字>/<名字>.{yaml,pgm,posegraph,data})"
}

# ---------- 模型外参 (只用于启动摘要) ----------
# 把"雷达到底装在哪儿"从 URDF 里算出来。用的是 real_mapping_launch.py 里**同一个**
# 入口 (srm_robot_description.build_description), 所以打印出来的就是链路真会用的
# 外参与 Point-LIO 重力 —— 换完 URDF 之后, 第一时间在这里核对一眼。
# 解析失败不影响启动: launch 起来时还会再算一次, 那时报的错更完整。
resolve_model_mounting() {
  ros2_run timeout 60 python3 - "$PARAMS_FILE" <<'PY' 2>/dev/null || true
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix

sys.path.insert(
    0, str(Path(get_package_prefix("srm27_nav_bringup")) / "lib" / "srm27_nav_bringup")
)
from srm_robot_description import build_description, gravity_from_mounting


def axis(values, digits):
    """一行定长小数; 顺手把 -0.000000 这种毛刺显示成 0.000000。"""
    zero_text = "0." + "0" * digits
    texts = ["%.*f" % (digits, value) for value in values]
    return " ".join(zero_text if text == "-" + zero_text else text for text in texts)


_, mounting = build_description(sys.argv[1], "", "")
print(
    "xyz=[%s] rpy=[%s] gravity=[%s]"
    % (
        axis(mounting["xyz"], 6),
        axis(mounting["rpy"], 6),
        axis(gravity_from_mounting(mounting["rpy"]), 4),
    )
)
PY
}

# ---------- 停止 ----------
# 自身与祖先 PID: pkill -f 是按整条命令行匹配的, 若调用本脚本的那个 shell
# 命令行里恰好含 "rviz2" / "pointlio_mapping" 之类的字样 (例如
# `bash -c '...; ./script/start_real_slam.sh --stop'`), 直接 pkill 会把
# 调用者自己一起杀掉。所以下面只按 PID 精确清理, 并跳过自身与全部祖先。
ancestor_pids() {
  local pid="$$" parent
  while [ -n "$pid" ] && [ "$pid" != "0" ] && [ "$pid" != "1" ]; do
    printf '%s\n' "$pid"
    parent="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')"
    [ -n "$parent" ] || break
    pid="$parent"
  done
}

# 匹配 pattern 的 PID, 已排除自身与祖先。
matching_pids() {
  local pattern="$1" pid
  local -A skip=()
  while read -r pid; do
    if [ -n "$pid" ]; then skip["$pid"]=1; fi
  done < <(ancestor_pids)
  while read -r pid; do
    # 用 if 而不是 `[ ... ] && ...`: 后者会让循环体最后一条命令的退出码
    # 变成 while 的退出码, 在 set -e 下会把脚本带崩。
    if [ -n "$pid" ] && [ -z "${skip[$pid]:-}" ]; then
      printf '%s\n' "$pid"
    fi
  done < <(pgrep -u "$(id -u)" -f "$pattern" 2>/dev/null || true)
}

# 对一组 pattern 命中的进程发信号; 有进程被命中时返回 0, 一个都没命中返回 1。
signal_matching() {
  local signal="$1"; shift
  local pids=() pid pattern
  for pattern in "$@"; do
    while read -r pid; do
      if [ -n "$pid" ]; then pids+=("$pid"); fi
    done < <(matching_pids "$pattern")
  done
  if [ "${#pids[@]}" = "0" ]; then
    return 1
  fi
  # 去重后逐个发送, 避免同一进程收到重复信号。
  printf '%s\n' "${pids[@]}" | sort -u | while read -r pid; do
    kill "-$signal" "$pid" 2>/dev/null || true
  done
  return 0
}

stop_mapping() {
  echo "[停止] 先发零速到 cmd_vel_chassis (看门狗 0.5 s 会兜底, 这里仍主动归零一次)..."
  if command -v ros2 >/dev/null 2>&1 && [ -f "$WS_DIR/install/setup.bash" ]; then
    ros2_run timeout 2 ros2 topic pub -r 20 /cmd_vel_chassis geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
  else
    echo "[警告] 无法发送零速 (ros2 不可用或工作空间未编译), 请手动确认底盘已停。" >&2
  fi

  local patterns=("$MAPPING_PATTERN" "$CHASSIS_PATTERN" "$JOY_PATTERN" "${RESIDUAL_PATTERNS[@]}")

  # 建图链路本来就要 Ctrl+C 结束; 这里先 SIGINT 让 Point-LIO / slam_toolbox
  # 走正常退出流程 (--save-pcd 的落盘就在这一步)。
  signal_matching INT "${patterns[@]}" || echo "[提示] 未发现正在运行的建图相关进程。"

  # 录包进程单独处理: rosbag2 是在退出时才把内存里的缓存刷盘并写 metadata.yaml 的,
  # 那一步可能要好几十秒 (几 GB 的包)。它混在下面那轮 5 秒超时里一定被 SIGKILL,
  # 结果是拿到一堆无法回放的 db3。所以先给它 SIGINT, 等它自己退出再走后面那轮。
  if signal_matching INT "${BAG_PATTERNS[@]}"; then
    echo "[停止] 已通知录包进程收尾 (写 metadata.yaml), 最多等 ${BAG_FINALIZE_WAIT} 秒..."
    local waited=0
    # 只等真正的 ros2 录包进程 —— metadata.yaml 是它写的; 外层
    # record_mapping_bag.sh 只是壳子, 它退没退跟包完不完整无关。
    # 这里用 matching_pids 而不是 pgrep: 后者会把"命令行里恰好含 ros2 bag record
    # 这几个字"的无关进程(比如调用者自己的 shell)也算成在跑, 于是白等满 30 秒
    # 再打一条吓人的假警告。
    while [ "$waited" -lt "$BAG_FINALIZE_WAIT" ]; do
      if [ -z "$(matching_pids "$BAG_RECORDER_PATTERN")" ]; then
        break
      fi
      sleep 2
      waited=$((waited + 2))
    done
    if [ -n "$(matching_pids "$BAG_RECORDER_PATTERN")" ]; then
      echo "[警告] 录包进程 ${BAG_FINALIZE_WAIT} 秒内没退完, 下面会被强制结束。" >&2
      echo "[警告] 被强杀的 bag 会缺 metadata.yaml; 事后用 ./script/build_pcd_from_bag.sh 可自动补 (ros2 bag reindex), 尾部几秒可能缺。" >&2
    else
      echo "[停止] 录包进程已收尾 (等待 ${waited} 秒)。"
    fi
  fi

  sleep 5

  local killed=0
  if signal_matching KILL "${patterns[@]}" "${BAG_PATTERNS[@]}"; then
    killed=1
  fi
  if [ "$killed" = "1" ]; then
    echo "[停止] 仍有节点未在 5 秒内响应 SIGINT, 已强制结束。"
  fi
  echo "[停止] 完成。请确认终端里的标签页也已退出 (必要时手动 Ctrl-C)。"
  if is_true "$SAVE_PCD" && [ -f "$PCD_SAVE_PATH" ]; then
    echo "[停止] 累积点云: $PCD_SAVE_PATH"
  fi
  # 录完必看的自检: 没有 metadata.yaml 的 bag 目录等于废数据, 别等回放时才发现。
  local bag_dir
  for bag_dir in "$WS_DIR"/log_diag/mapping_bag_*/; do
    [ -d "$bag_dir" ] || continue
    if [ -f "$bag_dir/metadata.yaml" ]; then
      printf '[停止] 录包完整: %s (%s)\n' "$bag_dir" "$(du -sh "$bag_dir" 2>/dev/null | cut -f1)"
    else
      printf '[停止] 警告: %s 缺 metadata.yaml; build_pcd_from_bag.sh 会自动补 (ros2 bag reindex), 不必重录。\n' "$bag_dir" >&2
    fi
  done
}

# ---------- 存图 ----------
# 目标目录: 默认 <工作空间>/maps/<名字> (与现有地图布局一致)。
resolve_out_dir() {
  local name="$1"
  if [ -n "$MAP_OUT_DIR" ]; then
    printf '%s' "$MAP_OUT_DIR"
    return 0
  fi
  printf '%s' "$WS_DIR/maps/$name"
}

# 服务是否真的在: 用 --no-daemon 做一次全新发现, 而不是查常驻 daemon。
# 常驻 daemon 的缓存里会留着已经退出节点的服务名 (实测: 节点进程早没了,
# `ros2 service list` 仍报 /slam_toolbox/save_map), 查 daemon 就会得到假阳性,
# 脚本随后对着一个不存在的服务发请求并一直等到超时。全新发现约 1 秒, 划算。
slam_service_present() {
  ros2_run timeout 20 ros2 service list --no-daemon 2>/dev/null | grep -qx "$SLAM_SERVICE"
}

# 有界轮询等服务出现, 避免"链路刚起、服务还没就绪"被误报成"没在跑"。
wait_for_slam_service() {
  local waited=0
  if slam_service_present; then
    return 0
  fi
  if [ "$WAIT_FOR_SLAM" -gt 0 ]; then
    printf '[存图] 尚未发现 %s, 最多等 %s 秒 (链路刚起时服务需要一点时间)...\n' \
      "$SLAM_SERVICE" "$WAIT_FOR_SLAM"
    while [ "$waited" -lt "$WAIT_FOR_SLAM" ]; do
      sleep 2
      waited=$((waited + 2))
      if slam_service_present; then
        printf '[存图] %s 已就绪 (等待 %s 秒)。\n' "$SLAM_SERVICE" "$waited"
        return 0
      fi
    done
  fi
  return 1
}

# 分清两种失败: 链路根本没跑 vs 跑了但没开 use_map_saver。
# 两者的处理办法完全不同, 所以不能共用一句"找不到服务"。
diagnose_slam_toolbox() {
  local nodes stale
  nodes="$(ros2_run timeout 20 ros2 node list --no-daemon 2>/dev/null | grep -i "$SLAM_NODE_NAME" || true)"
  if [ -n "$nodes" ]; then
    cat >&2 <<EOF
[错误] slam_toolbox 节点在跑, 但没有 $SLAM_SERVICE 服务:
[错误]   $nodes
[错误] 该服务由节点参数 use_map_saver 决定, 现在它不是 true (或加载的不是建图参数文件)。
[错误] 检查: grep -n use_map_saver $(q "$MAP_PARAMS_DEFAULT")
[错误] 正确做法是让建图链路自己加载这份参数: ./script/start_real_slam.sh
EOF
    return 0
  fi

  # 全新发现查不到、但 daemon 缓存里有: 那是已经退出进程的残留记录,
  # 这类残留会让"查 daemon"的判断出错, 值得单独提示一句。
  stale="$(ros2_run timeout 15 ros2 node list 2>/dev/null | grep -i "$SLAM_NODE_NAME" || true)"
  if [ -n "$stale" ]; then
    echo "[提示] ros2 daemon 缓存里还留着旧记录 ($stale), 但全新发现查不到, 说明该进程早已退出。" >&2
    echo "[提示] 需要清理时执行 ros2 daemon stop (下次调用 ros2 会自动重启 daemon)。" >&2
  fi

  cat >&2 <<EOF
[错误] 没有检测到 slam_toolbox 节点, 建图链路没在运行。
[错误] --save 只是调用在线服务, 把 slam_toolbox 内存里的位姿图写到磁盘,
[错误] 它不会自己起链路, 也不能"离线"从旧地图里取数据。
[错误] 先在另一个终端起链路并保持运行:
[错误]   ./script/start_real_slam.sh --joy     # 带手柄, 推杆走车、边走边建图
[错误] 等 RViz 里地图开始连成片之后, 再回来执行:
[错误]   ./script/start_real_slam.sh --save <名字>
[错误] 自查: ros2 node list | grep $SLAM_NODE_NAME     # 应能看到 /$SLAM_NODE_NAME
EOF
}

# 退出码 124 来自 timeout: 服务名能查到但请求没人应答, 最常见的原因是
# ros2 daemon 缓存里留着已退出节点的服务名 (对着尸体发请求)。
report_service_timeout() {
  if [ "$1" = "124" ]; then
    echo "[提示] 请求超时, 说明服务名能查到但没人应答;" >&2
    echo "[提示] 常见原因是 ros2 daemon 缓存里留着已退出节点的残留记录。" >&2
    echo "[提示] 处置: ros2 daemon stop, 确认建图链路真的在跑后重试。" >&2
  fi
}

save_map() {
  local name="$1"
  local out_dir target save_output serialize_output rc=0

  case "$name" in
    *[!A-Za-z0-9_.-]*|'')
      echo "[错误] --save 的名字只允许字母/数字/下划线/点/短横线, 收到: $name" >&2
      exit 2 ;;
  esac

  out_dir="$(resolve_out_dir "$name")"
  if [ "${out_dir#/}" = "$out_dir" ]; then
    echo "[错误] 存图目录必须是绝对路径: $out_dir" >&2
    exit 2
  fi
  target="$out_dir/$name"

  # slam_toolbox 的 save_map 需要节点参数 use_map_saver: true。
  if [ "$DRY_RUN" = "1" ]; then
    echo "--- [dry-run] 存图: $name"
    echo "  mkdir -p $(q "$out_dir")"
    echo "  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '$(q "$target")'}}\""
    echo "  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '$(q "$target")'}\""
    echo "  期望产出: $target.pgm  $target.yaml  $target.posegraph  $target.data"
    return 0
  fi

  if ! command -v ros2 >/dev/null 2>&1 || [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo "[错误] ros2 不可用或未找到 $WS_DIR/install/setup.bash, 无法存图。" >&2
    exit 1
  fi

  # ---- 等 slam_toolbox 的在线服务就绪 ----
  # 判据是 /slam_toolbox/save_map 是否存在 (save_map 由节点参数 use_map_saver: true 决定)。
  if ! wait_for_slam_service; then
    diagnose_slam_toolbox
    exit 1
  fi

  if [ "$FORCE_SAVE" != "1" ]; then
    local existing=""
    local ext
    for ext in pgm yaml posegraph data; do
      [ -f "$target.$ext" ] && existing="$existing $target.$ext"
    done
    if [ -n "$existing" ]; then
      echo "[错误] 以下文件已存在, 不覆盖:$existing" >&2
      echo "[提示] 换个名字: --save ${name}_$(date +%m%d)" >&2
      echo "[提示] 或确认要覆盖后加 --force。" >&2
      exit 1
    fi
  fi

  mkdir -p "$out_dir"
  echo "[存图] 二维栅格图 -> $target.{pgm,yaml}"
  # save_map 内部走 nav2 map_saver, 大图落盘可能几秒到几十秒, 超时给足。
  save_output="$(ros2_run timeout 180 ros2 service call /slam_toolbox/save_map \
      slam_toolbox/srv/SaveMap "{name: {data: '$target'}}" 2>&1)" || rc=$?
  if [ "$rc" != "0" ]; then
    echo "$save_output" >&2
    echo "[错误] save_map 服务调用失败 (退出码 $rc)。" >&2
    report_service_timeout "$rc"
    exit 1
  fi

  echo "[存图] 位姿图 -> $target.{posegraph,data}"
  serialize_output="$(ros2_run timeout 180 ros2 service call /slam_toolbox/serialize_map \
      slam_toolbox/srv/SerializePoseGraph "{filename: '$target'}" 2>&1)" || rc=$?
  if [ "$rc" != "0" ]; then
    echo "$serialize_output" >&2
    echo "[错误] serialize_map 服务调用失败 (退出码 $rc)。" >&2
    report_service_timeout "$rc"
    exit 1
  fi

  # 以文件落盘为准判定成败: 服务返回码在 slam_toolbox 里只区分
  # "没收到地图" 和 "写文件失败", 直接用文件更直观。
  local missing="" ext size
  for ext in pgm yaml posegraph data; do
    if [ ! -s "$target.$ext" ]; then
      missing="$missing $ext"
    fi
  done
  if [ -n "$missing" ]; then
    # 两个服务的返回码要分开看: save_map 的 1 = 还没收到地图,
    # serialize_map 的 255 = 写文件失败, 混在一起会看不出是哪一步的问题。
    echo "--- save_map 响应:" >&2
    echo "$save_output" >&2
    echo "--- serialize_map 响应:" >&2
    echo "$serialize_output" >&2
    echo "[错误] 存图不完整, 缺失:$missing" >&2
    if [ ! -s "$target.pgm" ]; then
      echo "[提示] 缺 pgm/yaml 通常是 slam_toolbox 还没收到第一帧 /scan;" >&2
      echo "[提示] 确认雷达在出点云、/scan 有数据后在 RViz 里看到地图再存。" >&2
    fi
    exit 1
  fi

  echo "[存图] 完成:"
  for ext in pgm yaml posegraph data; do
    size="$(stat -c %s "$target.$ext" 2>/dev/null || echo 0)"
    printf '  %-42s %8s 字节\n' "$target.$ext" "$size"
  done
  echo
  echo "下一步:"
  echo "  导航加载这张图: ./script/start_real_nav.sh -m $name"
  echo "  续建图 (接着这张位姿图跑): 用 slam_toolbox 的 deserialize_map, 见 docs/调试日志——by maori.md"
}

# ---------- 命令行参数 ----------
ACTION_START=1

while [ $# -gt 0 ]; do
  case "$1" in
    --save)          SAVE_MAP_NAME="${2:?--save 需要地图名字}"; ACTION_START=0; shift 2 ;;
    --out-dir)       MAP_OUT_DIR="${2:?--out-dir 需要绝对路径}"; shift 2 ;;
    --wait)          WAIT_FOR_SLAM="${2:?--wait 需要秒数}"; shift 2 ;;
    --force)         FORCE_SAVE=1; shift ;;
    --record-bag)    RECORD_BAG=1; shift ;;
    --bag-name)      BAG_NAME="${2:?--bag-name 需要名字}"; shift 2 ;;
    --bag-derived)   BAG_DERIVED=1; shift ;;
    --bag-out-dir)   BAG_OUT_DIR="${2:?--bag-out-dir 需要绝对路径}"; shift 2 ;;
    -p|--params)     PARAMS_FILE="${2:?--params 需要绝对路径}"; shift 2 ;;
    --lidar-config)  LIDAR_CONFIG_FILE="${2:?--lidar-config 需要路径}"; shift 2 ;;
    # ${2?} 而不是 ${2:?}: 允许显式给空串, 含义是"这一项跟随 URDF 模型"。
    --lidar-xyz)     LIDAR_XYZ="${2?--lidar-xyz 需要 \"X Y Z\"}"; shift 2 ;;
    --lidar-rpy)     LIDAR_RPY="${2?--lidar-rpy 需要 \"R P Y\"}"; shift 2 ;;
    --lidar)         START_LIDAR=1; shift ;;
    --no-lidar)      START_LIDAR=0; shift ;;
    --use-sim-time)  USE_SIM_TIME=1; shift ;;
    --save-pcd)      SAVE_PCD=1; shift ;;
    --save-pcd-path) PCD_SAVE_PATH_CLI="${2:?--save-pcd-path 需要路径}"; shift 2 ;;
    --lio-frontend)  LIO_FRONTEND="${2:?--lio-frontend 需要 point_lio|zlio}"; shift 2 ;;
    --zlio-config)   ZLIO_CONFIG="${2:?--zlio-config 需要路径}"; shift 2 ;;
    --scan-cloud)    SCAN_CLOUD="${2:?--scan-cloud 需要 raw|registered}"; shift 2 ;;
    --rviz)          USE_RVIZ=1; shift ;;
    --no-rviz)       USE_RVIZ=0; shift ;;
    --chassis)       USE_CHASSIS=1; shift ;;
    --no-chassis)    USE_CHASSIS=0; shift ;;
    --joy)           USE_JOY=1; shift ;;
    --no-joy)        USE_JOY=0; shift ;;
    --joy-dev)       JOY_DEV="${2:?--joy-dev 需要设备号}"; shift 2 ;;
    -n|--check)      DRY_RUN=1; shift ;;
    --list-maps)     list_maps; exit 0 ;;
    --stop)          stop_mapping; exit 0 ;;
    -h|--help)       usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 -h 查看用法)" >&2; exit 2 ;;
  esac
done

case "$WAIT_FOR_SLAM" in
  ''|*[!0-9]*)
    echo "[错误] --wait 必须是非负整数秒数, 收到: $WAIT_FOR_SLAM" >&2
    exit 2 ;;
esac

# 录包三个参数放在一起校验: 单独给 --bag-name / --bag-derived 而没加 --record-bag
# 是最容易犯的错, 而且错得很安静 (什么都录不到), 所以这里直接拦下来。
if ! is_true "$RECORD_BAG" && { [ -n "$BAG_NAME" ] || [ -n "$BAG_OUT_DIR" ] || is_true "$BAG_DERIVED"; }; then
  echo "[错误] 给了 --bag-name/--bag-out-dir/--bag-derived, 但没有 --record-bag。" >&2
  echo "[提示] 补上 --record-bag, 或去掉这些录包参数。" >&2
  exit 2
fi
if [ -n "$BAG_OUT_DIR" ] && [ "${BAG_OUT_DIR#/}" = "$BAG_OUT_DIR" ]; then
  echo "[错误] --bag-out-dir 必须是绝对路径: $BAG_OUT_DIR" >&2
  exit 2
fi
if is_true "$RECORD_BAG" && [ ! -x "$BAG_SCRIPT" ]; then
  echo "[错误] 录包脚本不可用: $BAG_SCRIPT" >&2
  echo "[提示] 需要可执行权限: chmod +x $BAG_SCRIPT" >&2
  exit 1
fi

# ---------- 默认路径 ----------
[ -n "$PARAMS_FILE" ] || PARAMS_FILE="$MAP_PARAMS_DEFAULT"
[ -n "$LIDAR_CONFIG_FILE" ] || LIDAR_CONFIG_FILE="$LIDAR_CONFIG_DEFAULT"

# ---------- 存图分支 (与启动互斥: --save 只存图, 不起新节点) ----------
if [ "$ACTION_START" = "0" ]; then
  if is_true "$RECORD_BAG"; then
    echo "[提示] --record-bag 只在启动链路时生效; 本次是 --save 分支, 不涉及录包。" >&2
  fi
  save_map "$SAVE_MAP_NAME"
  exit 0
fi

# ---------- 启动前校验 ----------
if [ "$DRY_RUN" != "1" ] && [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 未找到 $WS_DIR/install/setup.bash" >&2
  echo "[提示] 先编译工作空间:" >&2
  echo "[提示]   cd $WS_DIR && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2" >&2
  echo "[提示] 或用 SRM27_WS_DIR=/path/to/ws 指定工作空间。" >&2
  exit 1
fi

if [ "$DRY_RUN" != "1" ] && ! command -v "$TERMINAL" >/dev/null 2>&1; then
  echo "[错误] 未找到终端程序: $TERMINAL" >&2
  echo "[提示] 可用 TERMINAL=konsole 或 TERMINAL=xfce4-terminal 指定。" >&2
  exit 1
fi

if [ "$DRY_RUN" != "1" ] && is_true "$USE_RVIZ" \
   && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "[警告] 未检测到 DISPLAY / WAYLAND_DISPLAY, RViz 可能起不来 (可加 --no-rviz)。" >&2
fi

if [ "${PARAMS_FILE#/}" = "$PARAMS_FILE" ]; then
  echo "[错误] --params 必须是绝对路径: $PARAMS_FILE" >&2
  exit 1
fi
if [ ! -f "$PARAMS_FILE" ]; then
  echo "[错误] 建图参数文件不存在: $PARAMS_FILE" >&2
  exit 1
fi
case "$LIO_FRONTEND" in
  point_lio|zlio) ;;
  *) echo "[错误] --lio-frontend 只支持 point_lio / zlio, 收到: $LIO_FRONTEND" >&2; exit 2 ;;
esac
# 2D 栅格的点云来源。raw = 雷达原始云（密，矮/薄障碍能进图；未去畸变），
# registered = LIO 输出的 registered_scan（已去畸变，但只有 ~1.7k 点/帧）。
# 没显式给就按是否起雷达决定：--no-lidar 回放时 bag 里只有 CustomMsg，没有
# /livox/lidar/pointcloud，只能退回 registered_scan。
if [ -z "$SCAN_CLOUD" ]; then
  if [ "$START_LIDAR" = "1" ]; then SCAN_CLOUD=raw; else SCAN_CLOUD=registered; fi
fi
case "$SCAN_CLOUD" in
  raw)        SCAN_CLOUD_TOPIC="/livox/lidar/pointcloud" ;;
  registered) SCAN_CLOUD_TOPIC="registered_scan" ;;
  *) echo "[错误] --scan-cloud 只支持 raw / registered, 收到: $SCAN_CLOUD" >&2; exit 2 ;;
esac
if [ -z "$ZLIO_CONFIG" ]; then
  ZLIO_CONFIG="$PKG_DIR/config/real/zlio_mid360.yaml"
  [ -f "$ZLIO_CONFIG" ] || ZLIO_CONFIG="$PKG_SRC/config/real/zlio_mid360.yaml"
fi
if [ ! -f "$LIDAR_CONFIG_FILE" ]; then
  echo "[错误] 雷达驱动配置不存在: $LIDAR_CONFIG_FILE" >&2
  echo "[提示] 用 --lidar-config <路径> 指定。" >&2
  exit 1
fi

# 驱动外参必须全零: 非零时驱动自己转一次点云、URDF 再转一次, 建图必然重影,
# real_mapping_launch.py 也会直接抛异常拒绝启动 —— 这里提前给出更好懂的错。
if command -v python3 >/dev/null 2>&1; then
  EXTRINSIC_PROBLEM="$(python3 -c '
import json, sys

try:
    with open(sys.argv[1], encoding="utf-8") as stream:
        config = json.load(stream)
except Exception as ex:
    print("读取或解析失败: %s" % ex)
    sys.exit(0)

lidars = config.get("lidar_configs")
if not isinstance(lidars, list) or not lidars:
    print("缺少可用的 lidar_configs 列表")
    sys.exit(0)

bad = [str(item.get("ip", "?")) for item in lidars
       if any(item.get("extrinsic_parameter", {}).values())]
if bad:
    print("extrinsic_parameter 不是全零, 受影响雷达: %s" % ", ".join(bad))
' "$LIDAR_CONFIG_FILE" 2>&1 || true)"
  if [ -n "$EXTRINSIC_PROBLEM" ]; then
    echo "[错误] 雷达驱动配置不可用: $LIDAR_CONFIG_FILE" >&2
    echo "[错误] $EXTRINSIC_PROBLEM" >&2
    echo "[错误] 驱动外参必须全零, 安装位姿由 --lidar-xyz / --lidar-rpy 承担。" >&2
    exit 1
  fi
else
  echo "[警告] 未找到 python3, 跳过雷达驱动外参自检。" >&2
fi

# 建图必须独占 map->odom: 导航栈在跑的话两边抢同一条 TF。
if process_running "$NAV_PATTERN"; then
  cat >&2 <<'EOF'
[错误] 检测到导航栈正在运行 (nav2_stack_launch.py / nav_real_launch.py / start_real_nav.sh)。
[错误] 建图与导航都会发布 map->odom, 同时跑会互相打架, RViz 里地图来回跳。
[错误] 先停掉导航: ./script/start_real_nav.sh --stop
[错误] 再起建图:   ./script/start_real_slam.sh
EOF
  exit 1
fi

# 底盘串口: 设备存在性与权限, 只告警不阻断 (车可能还没插上 / 只做链路验证)
if is_true "$USE_CHASSIS" && [ "$DRY_RUN" != "1" ]; then
  PROTOCOL_CFG="$WS_DIR/src/srm27_nav_protocol/config/srm27_nav_protocol.yaml"
  [ -f "$PROTOCOL_CFG" ] || PROTOCOL_CFG="$WS_DIR/install/srm27_nav_protocol/share/srm27_nav_protocol/config/srm27_nav_protocol.yaml"
  SERIAL_DEV=""
  if [ -f "$PROTOCOL_CFG" ]; then
    SERIAL_DEV="$(sed -n 's/^[[:space:]]*device_name:[[:space:]]*//p' "$PROTOCOL_CFG" | head -n 1)"
    SERIAL_DEV="${SERIAL_DEV%\"}"; SERIAL_DEV="${SERIAL_DEV#\"}"
  fi
  SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
  if [ ! -e "$SERIAL_DEV" ]; then
    echo "[警告] 底盘串口设备不存在: $SERIAL_DEV (插好下位机 USB 后再启动底盘标签页)" >&2
  elif [ ! -r "$SERIAL_DEV" ] || [ ! -w "$SERIAL_DEV" ]; then
    echo "[警告] 无读写权限: $SERIAL_DEV" >&2
    echo "[警告] 处理: sudo usermod -aG dialout \$USER 后重新登录, 或临时 sudo chmod 666 $SERIAL_DEV" >&2
  fi
fi

if ! is_true "$USE_CHASSIS"; then
  echo "[警告] 未启动底盘串口: /cmd_vel_chassis 将没有消费者, 车不会动, 只能空转建图。" >&2
fi
if ! is_true "$USE_JOY" && is_true "$USE_CHASSIS"; then
  echo "[提示] 默认不起手柄: 想推杆走车加 --joy; 或用你自己的遥控发到 /cmd_vel_chassis。" >&2
fi

# ---------- 组装命令 ----------
CLOCK_ARG="$(bool_arg "$USE_SIM_TIME")"
MAPPING_CMD="ros2 launch srm27_nav_bringup real_mapping_launch.py"
MAPPING_CMD="$MAPPING_CMD params_file:=$(q "$PARAMS_FILE")"
MAPPING_CMD="$MAPPING_CMD lidar_config:=$(q "$LIDAR_CONFIG_FILE")"
# 外参为空 = 干脆不传这个参数, 让 launch 走模型里的值。
# 不能改传空串: ros2 launch 会直接报 "malformed launch argument 'lidar_xyz:='"。
if [ -n "$LIDAR_XYZ" ]; then
  MAPPING_CMD="$MAPPING_CMD lidar_xyz:=$(q "$LIDAR_XYZ")"
fi
if [ -n "$LIDAR_RPY" ]; then
  MAPPING_CMD="$MAPPING_CMD lidar_rpy:=$(q "$LIDAR_RPY")"
fi
MAPPING_CMD="$MAPPING_CMD lio_frontend:=$LIO_FRONTEND"
if [ -n "$PCD_SAVE_PATH_CLI" ]; then
  MAPPING_CMD="$MAPPING_CMD pcd_save_path:=$(q "$PCD_SAVE_PATH_CLI")"
fi
MAPPING_CMD="$MAPPING_CMD zlio_config:=$(q "$ZLIO_CONFIG")"
MAPPING_CMD="$MAPPING_CMD start_lidar:=$(bool_arg "$START_LIDAR")"
MAPPING_CMD="$MAPPING_CMD use_sim_time:=$CLOCK_ARG"
MAPPING_CMD="$MAPPING_CMD use_rviz:=$(bool_arg "$USE_RVIZ")"
MAPPING_CMD="$MAPPING_CMD save_pcd:=$(bool_arg "$SAVE_PCD")"
# 2D 栅格的点云来源 (--scan-cloud, 默认 raw)。换回 LIO 的稀疏云: --scan-cloud registered
MAPPING_CMD="$MAPPING_CMD scan_cloud_topic:=$(q "$SCAN_CLOUD_TOPIC")"

# 透传额外的 launch 参数, 例如临时换 2D 扫描的点云来源:
#   MAPPING_EXTRA_ARGS="scan_cloud_topic:=registered_scan" ./script/start_real_slam.sh
# 默认 raw 是原始点云 (~20k 点/帧, 720 个方位格覆盖 ~598); registered_scan 只有 ~1.7k 点
# (覆盖 ~264), 矮台阶/薄栏杆会被相邻射线刷成空闲。
if [ -n "${MAPPING_EXTRA_ARGS:-}" ]; then
  MAPPING_CMD="$MAPPING_CMD $MAPPING_EXTRA_ARGS"
fi

# 底盘串口节点固定在根命名空间, 订阅全局 /cmd_vel_chassis。
CHASSIS_CMD="ros2 launch srm27_nav_protocol srm27_nav_protocol.launch.py"

JOY_CMD="ros2 launch srm27_nav_bringup joy_teleop_launch.py"
JOY_CMD="$JOY_CMD use_sim_time:=$CLOCK_ARG"
JOY_CMD="$JOY_CMD joy_vel:=cmd_vel_chassis"
JOY_CMD="$JOY_CMD joy_dev:=$JOY_DEV"
JOY_CMD="$JOY_CMD joy_config_file:=$(q "$JOY_PARAMS_DEFAULT")"

# 录包: 真正的活都在 record_mapping_bag.sh 里 (等话题、查磁盘、收尾自检),
# 这里只把参数拼好并单独占一个标签页。目录名由子脚本按时间戳自己生成
# (它会把实际路径打在自己那一页上), 这里只做摘要显示。
# 注意: 下面一律用 if 而不是 `[ ... ] && ...` —— 后者条件为假时整行退出码是 1,
# 在 set -e 下会把脚本直接带崩。
BAG_NAME_PREFIX=""
if [ -n "$BAG_NAME" ]; then
  BAG_NAME_PREFIX="${BAG_NAME}_"
fi
# BAG_BASE_DIR / BAG_TOPICS / BAG_MIN_FREE_GB 由子脚本自己从环境变量读 (子进程继承),
# 这里只把摘要显示对: 用户显式给了基目录就照实说。
BAG_DIR_DESC="${BAG_OUT_DIR:-${BAG_BASE_DIR:-$WS_DIR/log_diag}/mapping_bag_${BAG_NAME_PREFIX}<时间戳>}"

BAG_CMD=""
if is_true "$RECORD_BAG"; then
  BAG_CMD="$(q "$BAG_SCRIPT")"
  if [ -n "$BAG_NAME" ]; then
    BAG_CMD="$BAG_CMD --map-name $(q "$BAG_NAME")"
  fi
  if [ -n "$BAG_OUT_DIR" ]; then
    BAG_CMD="$BAG_CMD --out-dir $(q "$BAG_OUT_DIR")"
  fi
  if is_true "$BAG_DERIVED"; then
    BAG_CMD="$BAG_CMD --derived"
  fi
  if is_true "$USE_SIM_TIME"; then
    BAG_CMD="$BAG_CMD --use-sim-time"
  fi
fi

# ---------- 启动摘要 ----------
# 雷达外参这一行的显示值: 默认跟随模型, 那就把模型里的实际数值算出来一起打;
# 显式给了 --lidar-xyz / --lidar-rpy 就照实说是"脚本覆盖模型"。
LIDAR_MOUNT_DESC="xyz=[$LIDAR_XYZ] rpy=[$LIDAR_RPY]  (脚本覆盖模型)"
if [ -z "$LIDAR_XYZ" ] && [ -z "$LIDAR_RPY" ]; then
  LIDAR_MOUNT_DESC="跟随 URDF 模型"
  if [ -f "$WS_DIR/install/setup.bash" ] && command -v ros2 >/dev/null 2>&1; then
    RESOLVED_MOUNT="$(resolve_model_mounting)"
    if [ -n "$RESOLVED_MOUNT" ]; then
      LIDAR_MOUNT_DESC="$LIDAR_MOUNT_DESC -> $RESOLVED_MOUNT"
    else
      LIDAR_MOUNT_DESC="$LIDAR_MOUNT_DESC (未解析成功, 启动后 tf2_echo base_link livox_frame 核对)"
    fi
  fi
fi

echo "================= SRM 实车 SLAM 建图 ================="
echo "  工作空间    : $WS_DIR"
echo "  建图参数    : $PARAMS_FILE"
echo "  雷达配置    : $LIDAR_CONFIG_FILE"
echo "  雷达驱动    : $([ "$START_LIDAR" = "1" ] && echo "启动 (真实 MID360)" || echo "不启动 (假定有别的点云源)")"
echo "  雷达安装    : $LIDAR_MOUNT_DESC"
echo "  use_sim_time: $CLOCK_ARG    RViz: $(bool_arg "$USE_RVIZ")"
echo "  底盘串口    : $([ "$USE_CHASSIS" = "1" ] && echo "启动 (cmd_vel_chassis -> 串口)" || echo "不启动")"
echo "  手柄遥控    : $([ "$USE_JOY" = "1" ] && echo "启动 (设备 $JOY_DEV, 输出 cmd_vel_chassis)" || echo "不启动")"
echo "  累积点云    : $([ "$SAVE_PCD" = "1" ] && echo "开 (退出时写 $PCD_SAVE_PATH; zlio 由 zlio_bridge 落盘)" || echo "关")"
if is_true "$RECORD_BAG"; then
  echo "  建图录包    : 开 -> $BAG_DIR_DESC"
  echo "                $(if is_true "$BAG_DERIVED"; then echo "原始 + 派生话题, 约 480 MB/分钟"; else echo "原始话题, 约 240 MB/分钟"; fi)"
else
  echo "  建图录包    : 关 (想要事后能重建 PCD 就加 --record-bag)"
fi
if [ "$LIO_FRONTEND" = "zlio" ]; then
  echo "  里程计前端  : ZLIO (zlio_node + zlio_bridge; 与导航链一致)"
else
  echo "  里程计前端  : Point-LIO (+ loam_interface)"
fi
echo "  TF 发布者   : map->odom = slam_toolbox, odom->base_link = sensor_scan_generation"
echo "======================================================"

# ---------- 终端 ----------
open_term() {
  local title="$1"
  local cmd="$2"
  # 先 cd 到工作空间并 source 环境; 命令结束后保留 shell 便于查看输出。
  local full_cmd="cd '$WS_DIR' && source install/setup.bash && ${cmd}; exec bash"

  if [ "$DRY_RUN" = "1" ]; then
    echo "--- [dry-run] 标签页: $title"
    echo "$full_cmd"
    return 0
  fi

  if [ "$OPEN_MODE" = "tab" ]; then
    "$TERMINAL" --tab --title="$title" -- bash -c "$full_cmd"
  else
    "$TERMINAL" --title="$title" -- bash -c "$full_cmd"
  fi
}

# ---------- 启动前清理残留 ----------
# 每次启动前先跑一遍 kill_gzb.sh / kill_rviz.sh: 上一次异常退出留下的 Gazebo
# (以及它的 /clock bridge) 和 RViz 不清掉, 会和这次启动的节点抢 /clock、抢 RViz
# 窗口。跳过本步骤: PREKILL=0 ./script/start_real_slam.sh
prekill_leftovers() {
  if [ "${DRY_RUN:-0}" = "1" ]; then
    echo "[dry-run] 跳过启动前清理 (kill_gzb.sh / kill_rviz.sh)。"
    return 0
  fi
  if [ "${PREKILL:-1}" = "0" ]; then
    echo "[提示] PREKILL=0, 跳过启动前清理 (kill_gzb.sh / kill_rviz.sh)。"
    return 0
  fi

  echo "[清理] 启动前先结束残留的 Gazebo / RViz 进程..."
  local name
  for name in kill_gzb.sh kill_rviz.sh; do
    if [ ! -x "$SCRIPT_DIR/$name" ]; then
      echo "[警告] 未找到可执行脚本: $SCRIPT_DIR/$name (跳过)" >&2
      continue
    fi
    if ! "$SCRIPT_DIR/$name"; then
      # 退出码非 0 只代表"还有进程没死透", 两个 kill 脚本该 SIGINT/SIGKILL 的都发过了;
      # 这里只告警不阻断, 免得清理失败反而连启动都做不了。
      echo "[警告] $SCRIPT_DIR/$name 退出码非 0: 可能有进程未能结束, 继续启动。" >&2
    fi
  done
}

STEP_TITLES=()
STEP_PATTERNS=()
STEP_CMDS=()
add_step() {
  STEP_TITLES+=("$1")
  STEP_PATTERNS+=("$2")
  STEP_CMDS+=("$3")
}

add_step "SRM27 SLAM 建图链路" "$MAPPING_PATTERN" "$MAPPING_CMD"
if is_true "$USE_CHASSIS"; then
  add_step "SRM27 底盘串口" "$CHASSIS_PATTERN" "$CHASSIS_CMD"
fi
if is_true "$USE_JOY"; then
  add_step "SRM27 手柄遥控" "$JOY_PATTERN" "$JOY_CMD"
fi
if is_true "$RECORD_BAG"; then
  # 录包放最后起: 让它排在雷达驱动之后, /livox/lidar 出现得更早, 少等一会儿。
  add_step "SRM27 建图录包" "${BAG_PATTERNS[0]}" "$BAG_CMD"
fi

TOTAL=${#STEP_TITLES[@]}

if [ "$DRY_RUN" = "1" ]; then
  echo
  echo "[--check] 仅解析, 不启动。将要执行的命令:"
  for i in "${!STEP_TITLES[@]}"; do
    printf '\n[%d/%d] %s\n  %s\n' "$((i + 1))" "$TOTAL" "${STEP_TITLES[$i]}" "${STEP_CMDS[$i]}"
  done
  echo
  echo "解析完成, 未启动任何进程。"
  exit 0
fi

prekill_leftovers

for i in "${!STEP_TITLES[@]}"; do
  title="${STEP_TITLES[$i]}"
  pattern="${STEP_PATTERNS[$i]}"
  cmd="${STEP_CMDS[$i]}"
  printf '[%d/%d] 启动 %s...\n' "$((i + 1))" "$TOTAL" "$title"

  if process_running "$pattern"; then
    echo "[跳过] 已检测到正在运行的进程: $title"
    # 以前只打这一行, 用户完全不知道是哪个进程、为什么 RViz 没起来。
    # 把 pid + 命令行打出来, 并说清"RViz 在那条 launch 里"。
    while read -r pid; do
      if [ -n "$pid" ]; then
        echo "        pid $pid  $(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | cut -c1-160 || true)"
      fi
    done < <(matching_pids "$pattern")
    if [ "$pattern" = "$MAPPING_PATTERN" ]; then
      echo "        RViz 是在这条 launch 里起的 (use_rviz:=True), 所以这里不会另起一个。"
      echo "        如果那个标签页已经关了 / RViz 崩了, 说明这是**残留的 launch 外壳**,"
      echo "        先停干净再重来:  ./script/start_real_slam.sh --stop"
    else
      echo "        要先重起这一步:  ./script/start_real_slam.sh --stop"
    fi
    continue
  fi

  open_term "$title" "$cmd"
done

cat <<EOF

启动完成。建图请确认:
  - RViz 里已经把车走一圈的区域连成一片墙; 有重影先停下查雷达外参/时间戳。
  - 位姿图在内存里, **不存就没了**: 建完先存图再关。

存图 (建图链路保持运行, 另开终端执行):
  ./script/start_real_slam.sh --save <名字>          # -> maps/<名字>/{pgm,yaml,posegraph,data}
  ./script/start_real_slam.sh --save <名字> --force  # 同名已存在时覆盖

结束 (先发零速再停节点, 录包进程会先收尾再退):
  ./script/start_real_slam.sh --stop
EOF

if is_true "$RECORD_BAG"; then
  cat <<EOF

录包 (这次开了; 收尾只认 Ctrl+C / --stop):
  - 原始数据落在 $BAG_DIR_DESC, 实际路径看"SRM27 建图录包"标签页第一行。
  - 这四类文件里只有它能在事后重建三维 PCD: slam_toolbox 的 .posegraph/.data
    是二维位姿图 (karto LocalizedRangeScan + Pose2), 没有点云, 转不出来。
  - 别用 kill -9 / 直接关标签页: rosbag2 只在 SIGINT 收尾时写 metadata.yaml,
    缺了它的 bag 无法回放, 整趟白录。
  - 事后重建 PCD:
      ros2 bag play <bag 目录> --clock --topics /livox/lidar /livox/imu
      ros2 launch srm27_nav_bringup real_mapping_launch.py \\
          start_lidar:=False use_sim_time:=True save_pcd:=True
    放完 Ctrl+C 正常退出 -> src/srm27_navigation/point_lio/PCD/scans.pcd
    注意它仍在 Point-LIO 的 camera_init 系, 不是 map 系, 重定位前还要转。
EOF
fi

cat <<EOF

链路自检 (新终端):
  cd '$WS_DIR' && source install/setup.bash
  ros2 topic hz /livox/lidar                              # 点云 ~10 Hz
  ros2 topic hz /scan                                     # 二维扫描 (建图的输入)
  ros2 run tf2_ros tf2_echo map odom                      # 漂移看这里: z/roll/pitch 应稳定
  ros2 run tf2_ros tf2_echo odom base_link                # Point-LIO 里程计
  ros2 run tf2_ros tf2_echo base_link livox_frame          # 雷达外参: 换过 URDF 就核对这一条
  ros2 topic echo /map --once --field info.width          # 地图是否在长
EOF
