#!/usr/bin/env bash
# =============================================================
# 一键启动实车导航: Livox 雷达驱动 + Nav2 导航栈 (+ RViz / 手柄)
#
# 用法:
#   ./script/start_real_nav.sh                            # 默认地图 srm_site_01, 不启 SLAM, 不启重定位
#   ./script/start_real_nav.sh --reloc                    # 先验 PCD 重定位 (small_gicp)
#   ./script/start_real_nav.sh --slam                     # SLAM 建图 (slam_toolbox)
#   ./script/start_real_nav.sh -m srm_site_01 --reloc     # 显式指定地图名
#   ./script/start_real_nav.sh --map /abs/m.yaml --prior-pcd /abs/m.pcd --reloc
#   ./script/start_real_nav.sh --list-maps                # 列出可选地图
#   ./script/start_real_nav.sh --check                    # 只解析并打印将要执行的命令
#
# 四种定位方式(互斥), 区别只在 map->odom 由谁发布:
#   --slam    : slam_toolbox 建图, 由 SLAM 发布 map->odom, 不需要先验地图
#   --reloc   : small_gicp 与先验 PCD 做 GICP 配准, 发布 map->odom;
#               需要 --prior-pcd, odom->base 由 point_lio 提供
#   --lio     : 只要 Point-LIO 的里程计, 不做重定位、不用先验点云;
#               map->odom 用静态 TF (--map-to-odom 决定)。实车推荐用这个
#   (默认)    : map_server 加载先验栅格图 + 静态 map->odom, 需要外部里程计
#               (只适用于仿真真值/底盘自带里程计的场景, 实车这样起车不会动)
#
# 常用可选项:
#   -m, --map NAME|PATH        地图名或路径 (默认 srm_site_01)
#                              按名字查找顺序: <share>/map/real/<name>.yaml,
#                              ~/srm_nav_27/maps/<name>.yaml,
#                              ~/srm_nav_27/maps/<name>/<name>.yaml
#       --prior-pcd PATH       重定位先验点云, 默认 pcd/real/<map>.pcd
#       --slam / --reloc / --lio   见上, 三者互斥
#       --map-to-odom X Y YAW  静态定位模式下 map->odom 初始位姿 (默认 2.30 2.00 0.0,
#                              即默认地图 srm_site_01 的场地中心、车头朝地图 +x)
#       --params-file PATH     nav2 参数文件 (默认 config/real/nav2_params_srm.yaml)
#       --lidar-xyz "X Y Z"    雷达在 base_link 下的安装位置 (默认 0.15 -0.15 0.22)
#       --lidar-rpy "R P Y"    雷达安装角, 弧度 (默认 -0.06981317007977318 0 -1.5707963267948966)
#       --namespace NS         命名空间 (默认空 = 根命名空间)
#       --no-robot-state-pub   不启动 robot_state_publisher
#                              (默认启动: 默认参数文件要用 base_link / livox_frame /
#                               livox_imu / livox_scan 这些 frame, 由 SRM 模型提供;
#                               若车体模块已自行发布 URDF/TF, 请加此开关避免重复)
#       --no-rviz              不启动 RViz (默认启动)
#       --joy                  启动手柄遥控 (默认关闭)
#       --no-chassis           不启动底盘串口节点
#                              (默认启动: /cmd_vel 的唯一消费者, 不启动车不会动)
#       --no-composition       不使用组合节点 (use_composition:=False)
#   -n, --check                只解析并打印将要执行的命令, 不真正启动
#   -h, --help                 显示帮助
#
# 环境变量:
#   OPEN_MODE=tab|window       终端标签页 / 独立窗口 (默认 tab)
#   TERMINAL=gnome-terminal    终端程序
#   SRM27_WS_DIR=<ws>         覆盖工作空间路径
#   LIDAR_CONFIG_FILE=<json>   覆盖雷达驱动网络配置 (默认 config/real/mid360_user_config.json)
#
# 说明:
#   - 导航栈通过 nav2_stack_launch.py 组装, 因此重定位开关真正可用
#     (nav_real_launch.py 未透传 use_pcd_localization, 传了也无效)。
#   - 车体模型与参数默认用 SRM 版本:
#       模型: real_robot_state_publisher_launch.py  (base_link / livox_frame / livox_imu / livox_scan)
#       参数: config/real/nav2_params_srm.yaml  (srm27 导航结构 + SRM 实车参数)
#     想回到上游 srm27 那套(base_footprint / gimbal_yaw / front_mid360)时,
#     加 --params-file .../config/real/nav2_params_upstream.yaml 并换成上游模型即可;
#     两套 frame 不能混用, 参数文件与模型必须配套。
#   - 每个命令都会先 source 工作空间的 install/setup.bash。
# =============================================================

set -euo pipefail

# 工作目录 = 脚本所在目录的上一级（即 ROS2 工作空间根目录）
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${SRM27_WS_DIR:-$WS_DIR}"

# 包源码目录 / 安装后的 share 目录 (本工作空间是 symlink 安装, 两者内容等价)
SRC_SHARE_DIR="$WS_DIR/src/srm27_navigation/srm27_nav_bringup"
INSTALL_SHARE_DIR="$WS_DIR/install/srm27_nav_bringup/share/srm27_nav_bringup"

# 窗口 / 标签页 模式: "tab" 同一窗口多个标签页(默认), "window" 每个命令一个独立窗口
OPEN_MODE="${OPEN_MODE:-tab}"
# 终端程序（可改成 konsole / xfce4-terminal 等）
TERMINAL="${TERMINAL:-gnome-terminal}"

# ---- 默认参数（允许用环境变量覆盖）----------------------------------------
MAP_NAME="${MAP_NAME:-${MAP:-srm_site_01}}"
PRIOR_PCD_FILE="${PRIOR_PCD_FILE:-}"
PARAMS_FILE="${PARAMS_FILE:-}"
LIDAR_CONFIG_FILE="${LIDAR_CONFIG_FILE:-}"
NAMESPACE="${NAMESPACE:-}"
START_SLAM="${START_SLAM:-0}"
START_RELOC="${START_RELOC:-0}"
START_LIO="${START_LIO:-0}"
USE_RVIZ="${USE_RVIZ:-1}"
USE_ROBOT_STATE_PUB="${USE_ROBOT_STATE_PUB:-1}"
USE_JOY="${USE_JOY:-0}"
# 底盘串口节点默认启动: 它是 /cmd_vel 的唯一消费者, 不启动车就不会动。
# 用的是 SRM 实车协议版本 (src/standard_robot_pp_ros2, 从 ~/srm_auto_sentry 移植),
# 当前仅保留 SRM 协议串口驱动。
USE_CHASSIS="${USE_CHASSIS:-1}"
USE_COMPOSITION="${USE_COMPOSITION:-1}"
# 起步位置 = 车现在停的地方在地图坐标系里的位姿 (x, y, yaw弧度)。
# 数值由"实时点云 vs 地图"扫描匹配算出(见 实车部署方案(ai).md 现场调试记录),
# 位置约 (0.10, 0.00)、朝向 283°。换停车位置后需要重算。
# 仅在静态 map->odom 生效的模式下有用(--lio 或默认模式)。
MAP_TO_ODOM_X="${MAP_TO_ODOM_X:-0.10}"
MAP_TO_ODOM_Y="${MAP_TO_ODOM_Y:-0.00}"
MAP_TO_ODOM_YAW="${MAP_TO_ODOM_YAW:-4.9393}"
DRY_RUN="${DRY_RUN:-0}"
# SRM 实车雷达安装位姿（与 real_mapping_launch.py 的默认值一致）
LIDAR_XYZ="${LIDAR_XYZ:-0.15 -0.15 0.22}"
LIDAR_RPY="${LIDAR_RPY:--0.06981317007977318 0.0 -1.5707963267948966}"

# ---- 路径解析 -------------------------------------------------------------
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 未找到工作空间的 install/setup.bash, 请确认路径: $WS_DIR" >&2
  exit 1
fi

# 优先用安装后的 share 目录（launch 运行时 get_package_share_directory 返回的也是它），
# 找不到再退回源码目录。
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

PACKAGE_DIR="$(pick_dir "$INSTALL_SHARE_DIR" "$SRC_SHARE_DIR")"
MAP_DIR="$PACKAGE_DIR/map/real"
PCD_DIR="$PACKAGE_DIR/pcd/real"
PACKAGE_SRC_DIR="$SRC_SHARE_DIR"

if [ -z "$PARAMS_FILE" ]; then
  PARAMS_FILE="$PACKAGE_DIR/config/real/nav2_params_srm.yaml"
fi

# 雷达驱动的网络配置文件。必须与参数文件 livox 段声明的外参保持一致（都要求驱动外参为全零，
# 安装位姿由 URDF / lidar_xyz / lidar_rpy 表达）。
if [ -z "$LIDAR_CONFIG_FILE" ]; then
  LIDAR_CONFIG_FILE="$PACKAGE_DIR/config/real/mid360_user_config.json"
fi

RVIZ_CONFIG="$PACKAGE_DIR/rviz/nav2_default_view.rviz"

# 输入校验用的布尔判断
is_true() {
  case "${1,,}" in
    1 | true | yes | on) return 0 ;;
    *) return 1 ;;
  esac
}

# shell 安全引用
q() { printf '%q' "$1"; }

usage() {
  # 打印文件开头的连续注释块（跳过 shebang）
  awk 'NR > 1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

# 列出可选地图: pcd/real 与工作空间 maps/ 下的 *.yaml
list_maps() {
  echo "可选地图（名称去掉 .yaml 后作为 --map 的值）:"
  echo
  local f name pcd_ok seen="" found=0
  for f in "$SRC_SHARE_DIR"/map/real/*.yaml "$WS_DIR"/maps/*.yaml "$WS_DIR"/maps/*/*.yaml; do
    [ -f "$f" ] || continue
    # maps/<名字>/<名字>.yaml 这种子目录布局: 只认"目录名与 yaml 同名"的那份,
    # 否则 rosbag 的 maps/<bag>/metadata.yaml 会被误当成地图列出来。
    case "$f" in
      "$WS_DIR"/maps/*/*)
        [ "$(basename "$(dirname "$f")")" = "$(basename "$f" .yaml)" ] || continue
        ;;
    esac
    name="$(basename "$f" .yaml)"
    # 同名地图可能同时存在于两处, 只列一次
    case " $seen " in *" $name "*) continue ;; esac
    seen="$seen $name"
    pcd_ok="无"
    if [ -f "$SRC_SHARE_DIR/pcd/real/$name.pcd" ] || [ -f "$WS_DIR/maps/$name.pcd" ] \
      || [ -f "$WS_DIR/maps/${name}_mapframe.pcd" ]; then
      pcd_ok="有"
    fi
    printf '  %-24s 先验PCD: %s\n' "$name" "$pcd_ok"
    printf '  %-24s   栅格图: %s\n' "" "$f"
    found=1
  done
  if [ "$found" = "0" ]; then
    echo "  (未找到任何 .yaml 地图)"
  fi
  echo
  echo "地图目录  : $SRC_SHARE_DIR/map/real"
  echo "          : $WS_DIR/maps  (也支持 $WS_DIR/maps/<名字>/<名字>.yaml)"
  echo "先验PCD目录: $SRC_SHARE_DIR/pcd/real"
  echo "注意: --reloc 需要先验 PCD, 且该 PCD 必须已经在地图坐标系下。"
}

# 解析地图 yaml: 支持直接给路径, 或给名称后按几个约定目录查找
resolve_map_yaml() {
  local name="$1" cand
  for cand in "$name" "$WS_DIR/$name"; do
    if [ -f "$cand" ]; then
      readlink -f "$cand"
      return 0
    fi
  done
  for cand in \
    "$MAP_DIR/$name.yaml" \
    "$SRC_SHARE_DIR/map/real/$name.yaml" \
    "$WS_DIR/maps/$name.yaml" \
    "$WS_DIR/maps/$name/$name.yaml" \
    "$PACKAGE_DIR/map/simulation/$name.yaml"; do
    if [ -f "$cand" ]; then
      readlink -f "$cand"
      return 0
    fi
  done
  return 1
}

# 解析先验 PCD: 注意 _mapframe 版本优先于 Point-LIO camera_init 系的原始点云,
# 因为 GICP 重定位要求先验点云与先验栅格图在同一坐标系。
resolve_prior_pcd() {
  local name="$1" cand
  for cand in "$name" "$WS_DIR/$name"; do
    if [ -f "$cand" ]; then
      readlink -f "$cand"
      return 0
    fi
  done
  for cand in \
    "$PCD_DIR/$name.pcd" \
    "$SRC_SHARE_DIR/pcd/real/$name.pcd" \
    "$WS_DIR/maps/${name}_mapframe.pcd" \
    "$WS_DIR/maps/$name.pcd"; do
    if [ -f "$cand" ]; then
      readlink -f "$cand"
      return 0
    fi
  done
  return 1
}

# ---- 参数解析 -------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    -m | --map) MAP_NAME="${2:?--map 需要参数}"; shift 2 ;;
    --prior-pcd) PRIOR_PCD_FILE="${2:?--prior-pcd 需要参数}"; shift 2 ;;
    --params-file) PARAMS_FILE="${2:?--params-file 需要参数}"; shift 2 ;;
    --lidar-xyz) LIDAR_XYZ="${2:?--lidar-xyz 需要 "X Y Z"}"; shift 2 ;;
    --lidar-rpy) LIDAR_RPY="${2:?--lidar-rpy 需要 "R P Y"}"; shift 2 ;;
    --namespace) NAMESPACE="${2:?--namespace 需要参数}"; shift 2 ;;
    --map-to-odom)
      MAP_TO_ODOM_X="${2:?--map-to-odom 需要 X Y YAW}"
      MAP_TO_ODOM_Y="${3:?--map-to-odom 需要 X Y YAW}"
      MAP_TO_ODOM_YAW="${4:?--map-to-odom 需要 X Y YAW}"
      shift 4
      ;;
    --slam) START_SLAM=1; shift ;;
    --no-slam) START_SLAM=0; shift ;;
    --reloc | --relocalization) START_RELOC=1; shift ;;
    --no-reloc) START_RELOC=0; shift ;;
    --lio | --lio-only) START_LIO=1; shift ;;
    --no-lio) START_LIO=0; shift ;;
    --rviz) USE_RVIZ=1; shift ;;
    --no-rviz) USE_RVIZ=0; shift ;;
    --robot-state-pub) USE_ROBOT_STATE_PUB=1; shift ;;
    --no-robot-state-pub) USE_ROBOT_STATE_PUB=0; shift ;;
    --joy) USE_JOY=1; shift ;;
    --no-joy) USE_JOY=0; shift ;;
    --chassis) USE_CHASSIS=1; shift ;;
    --no-chassis) USE_CHASSIS=0; shift ;;
    --composition) USE_COMPOSITION=1; shift ;;
    --no-composition) USE_COMPOSITION=0; shift ;;
    -n | --check) DRY_RUN=1; shift ;;
    --list-maps) list_maps; exit 0 ;;
    -h | --help) usage; exit 0 ;;
    *)
      echo "[错误] 未知参数: $1" >&2
      echo "        用 --help 查看用法。" >&2
      exit 1
      ;;
  esac
done

# ---- 参数校验 -------------------------------------------------------------
# 三种定位方式互斥(区别只在 map->odom 由谁发布), 先归一化成 0/1。
SLAM_ON=0
RELOC_ON=0
LIO_ON=0
if is_true "$START_SLAM"; then SLAM_ON=1; fi
if is_true "$START_RELOC"; then RELOC_ON=1; fi
if is_true "$START_LIO"; then LIO_ON=1; fi

if [ $((SLAM_ON + RELOC_ON + LIO_ON)) -gt 1 ]; then
  echo "[错误] --slam / --reloc / --lio 三者互斥, 因为它们都决定 map->odom 由谁发布:" >&2
  echo "[错误]   --slam : slam_toolbox 发布 map->odom (边跑边建图)" >&2
  echo "[错误]   --reloc: small_gicp 与先验 PCD 配准后发布 map->odom" >&2
  echo "[错误]   --lio  : Point-LIO 只提供里程计, map->odom 用静态 TF (不用先验点云)" >&2
  echo "[错误] 同时开启会导致 TF 冲突。" >&2
  exit 1
fi

SLAM_ARG=$([ "$SLAM_ON" = "1" ] && echo True || echo False)
RELOC_ARG=$([ "$RELOC_ON" = "1" ] && echo True || echo False)
LIO_ARG=$([ "$LIO_ON" = "1" ] && echo True || echo False)
COMPOSITION_ARG=$([ "$USE_COMPOSITION" = "1" ] && echo True || echo False)

if [ ! -f "$PARAMS_FILE" ]; then
  echo "[错误] nav2 参数文件不存在: $PARAMS_FILE" >&2
  echo "[错误] 用 --params-file 指定, 或确认工作空间已编译。" >&2
  exit 1
fi

if [ ! -f "$LIDAR_CONFIG_FILE" ]; then
  echo "[错误] 雷达驱动配置不存在: $LIDAR_CONFIG_FILE" >&2
  echo "[错误] 用 LIDAR_CONFIG_FILE=<路径> 指定。" >&2
  exit 1
fi

# 驱动外参自检: 必须全零。非零时驱动自己会旋转一次点云, URDF 再转一次,
# 而且 real_robot_state_publisher_launch.py 会直接拒绝启动 —— 这里提前报错信息更清楚。
if ! python3 -c '
import json, sys
cfg = json.load(open(sys.argv[1], encoding="utf-8"))
bad = [l.get("ip") for l in cfg["lidar_configs"] if any(l["extrinsic_parameter"].values())]
sys.exit(1 if bad else 0)
' "$LIDAR_CONFIG_FILE"; then
  echo "[错误] 该配置的 extrinsic_parameter 不是全零: $LIDAR_CONFIG_FILE" >&2
  echo "[错误] 驱动外参必须全零, 安装位姿由 --lidar-xyz / --lidar-rpy 承担," >&2
  echo "[错误] 否则点云会被旋转两次。" >&2
  exit 1
fi

# 参数文件与车体模型必须配套: 模型提供什么 frame, 参数文件就用什么 frame。
# 默认的 SRM 模型发 base_link/livox_*; 上游 nav2_params_upstream.yaml 要的是
# base_footprint/gimbal_yaw/front_mid360, 混用会满屏 TF 报错且重定位卡死。
if [ "$USE_ROBOT_STATE_PUB" = "1" ] && ! grep -qE "livox_imu|base_link" "$PARAMS_FILE"; then
  cat >&2 <<EOF
[错误] 参数文件与车体模型不配套:
[错误]   参数文件 $PARAMS_FILE
[错误]   里面没有 base_link / livox_imu, 看起来是上游 nav2_params_upstream.yaml 那份
[错误]   (要用 base_footprint / gimbal_yaw / front_mid360), 而本脚本启动的
[错误]   SRM 模型只发 base_link / livox_frame / livox_imu / livox_scan。
[错误] 请二选一:
[错误]   1) 用 SRM 参数: 不加 --params-file (默认 config/real/nav2_params_srm.yaml)
[错误]   2) 用上游模型: 本脚本暂不支持, 请改用 nav_real_launch.py
EOF
  exit 1
fi
# 地图: SLAM 模式下 nav2_stack_launch 仍要求该参数存在, 但不加载栅格图
MAP_YAML=""
if MAP_YAML="$(resolve_map_yaml "$MAP_NAME")"; then
  :
else
  if [ "$START_SLAM" = "1" ]; then
    # 建图模式不需要先验栅格图, 给个占位路径即可
    MAP_YAML="$MAP_DIR/$MAP_NAME.yaml"
    echo "[提示] SLAM 模式不加载先验栅格图, 未找到 $MAP_NAME.yaml 属正常。" >&2
  else
    echo "[错误] 未找到地图: $MAP_NAME" >&2
    case "$MAP_NAME" in
      */*) : ;;
      *)
        echo "[错误] 已查找: $MAP_DIR/$MAP_NAME.yaml" >&2
        echo "[错误]           $WS_DIR/maps/$MAP_NAME.yaml" >&2
        ;;
    esac
    echo "[错误] 用 --map <名称|路径> 指定, 或 --list-maps 查看可选地图。" >&2
    echo "[错误] 若确实要边建图边导航, 请改用 --slam。" >&2
    exit 1
  fi
fi

# 先验点云: 仅重定位需要
PRIOR_PCD=""
if [ "$START_RELOC" = "1" ]; then
  if [ -z "$PRIOR_PCD_FILE" ]; then
    PRIOR_PCD_FILE="$MAP_NAME"
  fi
  if PRIOR_PCD="$(resolve_prior_pcd "$PRIOR_PCD_FILE")"; then
    :
  else
    echo "[错误] 重定位需要先验点云, 但未找到: $PRIOR_PCD_FILE" >&2
    # 只有当入参是"地图名"时才列出按约定目录拼出的候选路径
    case "$PRIOR_PCD_FILE" in
      */*) : ;;
      *)
        echo "[错误] 已查找: $PCD_DIR/$PRIOR_PCD_FILE.pcd" >&2
        echo "[错误]           $WS_DIR/maps/${PRIOR_PCD_FILE}_mapframe.pcd" >&2
        echo "[错误]           $WS_DIR/maps/$PRIOR_PCD_FILE.pcd" >&2
        ;;
    esac
    echo "[错误] 用 --prior-pcd <路径> 指定。" >&2
    echo "[错误] 注意: PCD 必须在地图坐标系下, Point-LIO camera_init 系的原始点云不能直接用。" >&2
    exit 1
  fi
  # 先验 PCD 坐标系自检: mapframe 版本头部有坐标系标注, 原始点云通常没有
  if ! head -c 1024 "$PRIOR_PCD" | grep -aqiE "map frame|map_frame|地图坐标系"; then
    echo "[警告] $PRIOR_PCD" >&2
    echo "[警告] 头部未标注坐标系; 若它是 Point-LIO camera_init 系的原始点云," >&2
    echo "[警告] GICP 重定位初值会错。请先转到与栅格图一致的地图坐标系。" >&2
  fi
else
  # 非重定位模式: 该参数仅供 launch 声明, 传占位路径即可
  PRIOR_PCD="${PRIOR_PCD_FILE:-$PCD_DIR/$MAP_NAME.pcd}"
fi

# ---- 组装各条启动命令 -----------------------------------------------------
NS_ARG=""
[ -n "$NAMESPACE" ] && NS_ARG="namespace:=$NAMESPACE"

# 1) Livox 雷达驱动: nav2_stack_launch 不负责起驱动, 必须单独启动。
#    这里有两件事必须显式处理, 否则驱动会"进程活着但一个点都不发", 而且不报错:
#      a) 驱动代码里的节点名是 livox_driver_node, 而参数文件顶层键是 livox_ros_driver2。
#         ROS 2 的 --params-file 按节点名匹配顶层键, 对不上就整段忽略
#         => 用 -r __node:=livox_ros_driver2 把节点改名, 让参数生效;
#      b) 参数文件里 user_config_path 写的是 $(find-pkg-share ...)/..., 这是 launch 专有
#         替换语法, ros2 run 走的是 rcl 的参数解析, 不会展开它
#         => 这里显式补一个绝对路径覆盖。
#    参数取值仍来自 nav2 参数文件的 livox_ros_driver2 段 (xfer_format / frame_id / 频率)。
DRIVER_CMD="ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args"
DRIVER_CMD="$DRIVER_CMD -r __node:=livox_ros_driver2"
DRIVER_CMD="$DRIVER_CMD --params-file $(q "$PARAMS_FILE")"
DRIVER_CMD="$DRIVER_CMD -p user_config_path:=$(q "$LIDAR_CONFIG_FILE")"
if [ -n "$NAMESPACE" ]; then
  DRIVER_CMD="$DRIVER_CMD -r __ns:=$(q "$NAMESPACE")"
fi

# 2) robot_state_publisher: 默认启动，用 SRM 模型（real_robot_state_publisher_launch.py，
#    与 real_mapping_launch.py 同一套构建逻辑）。默认参数文件里的 base_link / livox_frame /
#    livox_imu / livox_scan 全部由它提供; 缺了这些 frame，重定位节点会在构造时一直等 TF
#    而卡住整个 launch。车体模块若已自己发 URDF/TF 请用 --no-robot-state-pub 关闭，
#    否则会出现两个重复的 TF 发布者。
#    注意: extrinsic_T 与 lidar_xyz/lidar_rpy 必须来自同一份标定 —— 模型里 livox_imu 的
#    偏移取自参数文件的 point_lio.mapping.extrinsic_T，安装位姿取自这里。
RSP_CMD="ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py $NS_ARG use_sim_time:=False"
RSP_CMD="$RSP_CMD params_file:=$(q "$PARAMS_FILE")"
RSP_CMD="$RSP_CMD lidar_config:=$(q "$LIDAR_CONFIG_FILE")"
RSP_CMD="$RSP_CMD lidar_xyz:=$(q "$LIDAR_XYZ")"
RSP_CMD="$RSP_CMD lidar_rpy:=$(q "$LIDAR_RPY")"

# 3) 导航栈
NAV_CMD="ros2 launch srm27_nav_bringup nav2_stack_launch.py"
NAV_CMD="$NAV_CMD $NS_ARG"
NAV_CMD="$NAV_CMD slam:=$SLAM_ARG"
NAV_CMD="$NAV_CMD use_pcd_localization:=$RELOC_ARG"
NAV_CMD="$NAV_CMD use_lio_odometry:=$LIO_ARG"
NAV_CMD="$NAV_CMD map:=$(q "$MAP_YAML")"
NAV_CMD="$NAV_CMD prior_pcd_file:=$(q "$PRIOR_PCD")"
NAV_CMD="$NAV_CMD params_file:=$(q "$PARAMS_FILE")"
NAV_CMD="$NAV_CMD use_sim_time:=False"
NAV_CMD="$NAV_CMD autostart:=true"
NAV_CMD="$NAV_CMD use_composition:=$COMPOSITION_ARG"
NAV_CMD="$NAV_CMD use_respawn:=false"
NAV_CMD="$NAV_CMD map_to_odom_x:=$MAP_TO_ODOM_X"
NAV_CMD="$NAV_CMD map_to_odom_y:=$MAP_TO_ODOM_Y"
NAV_CMD="$NAV_CMD map_to_odom_yaw:=$MAP_TO_ODOM_YAW"

# 4) RViz
RVIZ_CMD="ros2 launch srm27_nav_bringup rviz_launch.py $NS_ARG use_sim_time:=False rviz_config:=$(q "$RVIZ_CONFIG")"

# 5) 手柄遥控
#    joy_vel 默认是 cmd_vel, 但底盘串口节点订阅的是 cmd_vel_chassis(SRM 老工程命名),
#    这里显式指过去, 否则手柄推杆车也不动。
JOY_CMD="ros2 launch srm27_nav_bringup joy_teleop_launch.py $NS_ARG use_sim_time:=False joy_vel:=cmd_vel_chassis joy_config_file:=$(q "$PARAMS_FILE")"

# 6) 底盘串口通信: /cmd_vel 的唯一消费者, 把速度写进串口发给 C 板。
#    协议是 SRM 实车那套 (帧头 0xA5 / 长度 2 字节 / 速度报文 0x0302 / 带 is_recovering),
#    与官方 srm27 的 0x5A 协议不同, 不能互换。启动必须走它自己的 launch:
#    参数文件顶层键是 /standard_robot_pp_ros2, 而代码里的节点名是 StandardRobotPpRos2Node,
#    两者能对上全靠 launch 里那行 name="standard_robot_pp_ros2"。
#    直接 ros2 run 起的话参数不生效 (设备名空串/波特率 0), 串口根本打不开。
CHASSIS_CMD="ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py"

# ---- 打印配置摘要 ---------------------------------------------------------
if [ "$SLAM_ON" = "1" ]; then
  MODE_DESC="SLAM 建图 (slam_toolbox 发布 map->odom)"
elif [ "$RELOC_ON" = "1" ]; then
  MODE_DESC="先验 PCD 重定位 (small_gicp 发布 map->odom)"
elif [ "$LIO_ON" = "1" ]; then
  MODE_DESC="纯 Point-LIO 里程计 + 静态地图 (静态 map->odom, 不用先验点云)"
else
  MODE_DESC="静态地图定位 (静态 map->odom, 无里程计源)"
fi

echo "==================== 实车导航启动配置 ===================="
echo "  工作空间    : $WS_DIR"
echo "  定位方式    : $MODE_DESC"
if [ "$SLAM_ON" = "1" ]; then
  echo "  地图 (yaml) : $MAP_YAML  (SLAM 模式未加载)"
else
  echo "  地图 (yaml) : $MAP_YAML"
fi
if [ "$RELOC_ON" = "1" ]; then
  echo "  先验 PCD    : $PRIOR_PCD"
else
  echo "  先验 PCD    : $PRIOR_PCD  (未使用)"
fi
echo "  参数文件    : $PARAMS_FILE"
echo "  雷达配置    : $LIDAR_CONFIG_FILE"
if [ "$USE_ROBOT_STATE_PUB" = "1" ]; then
  echo "  车体模型    : real_robot_state_publisher_launch.py (SRM)"
  echo "  雷达安装    : xyz=[$LIDAR_XYZ]  rpy=[$LIDAR_RPY]"
else
  echo "  车体模型    : 不启动 (假定其他模块已发布 URDF/TF)"
fi
echo "  命名空间    : ${NAMESPACE:-<根命名空间>}"
echo "  slam        : $SLAM_ARG    use_pcd_localization: $RELOC_ARG    use_lio_odometry: $LIO_ARG"
echo "  组合节点    : $COMPOSITION_ARG"
if [ "$USE_CHASSIS" = "1" ]; then
  echo "  底盘串口    : standard_robot_pp_ros2.launch.py (SRM 协议, /dev/ttyACM0)"
else
  echo "  底盘串口    : 不启动"
  echo "  [警告] 未启动底盘串口节点: /cmd_vel 将没有消费者, 车不会动。" >&2
fi
if [ "$RELOC_ON" != "1" ] && [ "$SLAM_ON" != "1" ]; then
  echo "  map->odom   : x=$MAP_TO_ODOM_X y=$MAP_TO_ODOM_Y yaw=$MAP_TO_ODOM_YAW"
  if [ "$LIO_ON" != "1" ]; then
    echo "  [警告] 未指定 --slam / --reloc / --lio 中任何一个: 没有里程计来源," >&2
    echo "  [警告] odom->base_link 不会有人发布, 导航起不来。实车请用 --lio。" >&2
  fi
fi
echo "========================================================="

# ---- 启动 ---------------------------------------------------------------
# 打开一个终端执行指定命令
open_term() {
  local title="$1"
  local cmd="$2"
  # 在终端内先 cd 到工作空间, source 环境, 再执行命令; 结束后保留 shell 便于查看输出
  local full_cmd="cd '$WS_DIR' && source install/setup.bash && ${cmd}; exec bash"

  if [ "$OPEN_MODE" = "tab" ]; then
    "$TERMINAL" --tab --title="$title" -- bash -c "$full_cmd"
  else
    "$TERMINAL" --title="$title" -- bash -c "$full_cmd"
  fi
}

# 检查命令是否已经在运行。
# 使用带方括号的正则，避免 pgrep 把自身的匹配命令算进去。
process_running() {
  pgrep -u "$(id -u)" -f "$1" >/dev/null 2>&1
}

# 收集启动步骤, 便于统一编号与 dry-run
STEP_TITLES=()
STEP_PATTERNS=()
STEP_CMDS=()

add_step() {
  STEP_TITLES+=("$1")
  STEP_PATTERNS+=("$2")
  STEP_CMDS+=("$3")
}

add_step "Livox 雷达驱动" '[l]ivox_ros_driver2_node' "$DRIVER_CMD"
if [ "$USE_ROBOT_STATE_PUB" = "1" ]; then
  add_step "SRM 车体模型 (robot_state_publisher)" '[r]eal_robot_state_publisher_launch\.py' "$RSP_CMD"
fi
if [ "$USE_CHASSIS" = "1" ]; then
  add_step "底盘串口 (SRM 协议)" '[s]tandard_robot_pp_ros2\.launch\.py' "$CHASSIS_CMD"
fi
add_step "导航栈 (Nav2)" '[n]av2_stack_launch\.py' "$NAV_CMD"
if [ "$USE_RVIZ" = "1" ]; then
  add_step "RViz" '[r]viz_launch\.py' "$RVIZ_CMD"
fi
if [ "$USE_JOY" = "1" ]; then
  add_step "手柄遥控" '[j]oy_teleop_launch\.py' "$JOY_CMD"
fi

TOTAL=${#STEP_TITLES[@]}

if [ "$DRY_RUN" = "1" ]; then
  echo
  echo "[--check] 仅解析, 不启动。将要执行的命令:"
  for i in "${!STEP_TITLES[@]}"; do
    printf '\n[%d/%d] %s\n  %s\n' "$((i + 1))" "$TOTAL" "${STEP_TITLES[$i]}" "${STEP_CMDS[$i]}"
  done
  echo
  echo "解析完成，未启动任何进程。"
  exit 0
fi

for i in "${!STEP_TITLES[@]}"; do
  title="${STEP_TITLES[$i]}"
  pattern="${STEP_PATTERNS[$i]}"
  cmd="${STEP_CMDS[$i]}"
  printf '[%d/%d] 启动 %s...\n' "$((i + 1))" "$TOTAL" "$title"

  if process_running "$pattern"; then
    echo "[跳过] 已检测到正在运行的进程: $title"
    continue
  fi

  open_term "$title" "$cmd"
done

echo "启动流程处理完成（终端模式: ${OPEN_MODE}）。"
