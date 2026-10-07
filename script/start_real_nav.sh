#!/usr/bin/env bash
# =============================================================
# SRM 实车 NAV 导航测试 (一键启动)
#
#   步骤 1: Livox Mid-360 雷达驱动
#   步骤 2: SRM 车体 TF (robot_state_publisher: base_link / livox_* 等 frame)
#   步骤 3: 底盘串口 srm27_nav_protocol (/cmd_vel_chassis 的唯一消费者)
#   步骤 4: 导航栈 Nav2 (定位方式见下)
#   步骤 5: RViz
#   步骤 6: 手柄遥控 (可选, 默认关闭)
#
# 定位方式 (四者互斥, 区别只在 map->odom 由谁发布):
#   --lio     (默认) Point-LIO 只提供里程计, map->odom 用静态 TF —— 实车推荐
#   --reloc   small_gicp 与先验 PCD 做 GICP 配准, 由它发布 map->odom (需要 --prior-pcd)
#   --slam    slam_toolbox 边跑边建图, 由它发布 map->odom (不需要先验地图)
#   --static  只加载先验栅格图 + 静态 map->odom, 没有任何里程计来源
#             (仅当车体模块自己发 odom->base_link 时可用; 直接用本脚本起车不会动)
#
# 实车速度链路:
#   controller_server ─cmd_vel_controller─► velocity_smoother ─cmd_vel_nav2_result─┐
#                                                                                  v
#                                       fake_vel_transform ─► cmd_vel_chassis ─► srm27_nav_protocol ─► 串口 ─► C 板
#   默认经过 fake_vel_transform (SRM 实车参数文件就是按这条链路配的);
#   --no-fake-vel-transform 时改由 velocity_smoother 直接发 cmd_vel_chassis,
#   两级出口话题都对准底盘节点实际订阅的 cmd_vel_chassis, 不会出现"链路断开、车不动"。
#
# ⚠ 安全须知 (务必先读):
#   1) srm27_nav_protocol 在后台线程里按固定频率重发"最近一次"收到的速度,
#      源码中没有超时清零逻辑 (sendData() 的 while 循环)。所以**发命令的节点退出后
#      车不会自己停**, 会一直按最后速度跑。任何"停节点"操作前先发零速。
#   2) 本脚本的 --stop 会先发零速再结束进程; 物理急停按钮始终是第一手段。
#   3) 底盘能动之后, 先在 RViz 用很小的目标点试一次, 确认车头方向/正负号都对。
#
# 用法:
#   ./script/start_real_nav.sh                       # 默认 --lio, 地图自动选, 启动全部
#   ./script/start_real_nav.sh -m xjl0914            # 指定地图 (maps/ 下的名字)
#   ./script/start_real_nav.sh --reloc --prior-pcd /abs/map.pcd
#   ./script/start_real_nav.sh --slam                # 边建图边导航
#   ./script/start_real_nav.sh --map-to-odom 0 0 0   # 指定起步位姿 (map 系, 弧度)
#   ./script/start_real_nav.sh --no-rviz --no-joy    # 只起链路, 不开图形界面
#   ./script/start_real_nav.sh --list-maps           # 列出可用地图与先验 PCD
#   ./script/start_real_nav.sh -n                    # 只解析并打印将执行的命令, 不启动
#   ./script/start_real_nav.sh --stop                # 先发零速, 再结束实车链路节点
#
# 参数:
#   -m, --map <名字|绝对路径>   地图名(在 maps/ 下解析)或 YAML 绝对路径; 缺省自动选择
#       --prior-pcd <名字|路径> 重定位先验点云, 默认与地图同名
#       --map-to-odom X Y YAW   静态 map->odom 初值 (m, m, rad), 即"车现在停的位置
#                               在地图坐标系里的位姿"; 默认 0 0 0 (车在 map 原点)
#   -p, --params <绝对路径>     Nav2 参数, 默认 config/real/nav2_params_srm.yaml
#       --slam / --reloc / --lio / --static   定位方式, 四者互斥 (默认 --lio)
#       --lidar-xyz "X Y Z"     雷达在 base_link 下的安装位置 (默认 0.15 -0.15 0.22)
#       --lidar-rpy "R P Y"     雷达安装角, 弧度 (默认 -0.06981317007977318 0 -1.5707963267948966)
#       --lidar-config <路径>   雷达驱动网络配置, 默认 config/real/mid360_user_config.json
#       --namespace <ns>        命名空间 (默认空 = 根命名空间)
#       --rviz / --no-rviz             是否启动 RViz (默认启动)
#       --robot-state-pub / --no-robot-state-pub   是否启动车体 TF (默认启动)
#       --chassis / --no-chassis       是否启动底盘串口 (默认启动)
#       --joy / --no-joy               是否启动手柄遥控 (默认关闭)
#       --smoother / --no-smoother     是否串联 velocity_smoother (默认串联)
#       --fake-vel-transform / --no-fake-vel-transform  (默认启用, 见上文速度链路)
#       --composition / --no-composition   导航栈是否用组合节点 (默认组合)
#       --log-level <级别>      导航栈日志级别 (默认 info)
#   -n, --check                 只解析并打印将执行的命令 (等同 DRY_RUN=1)
#       --list-maps             列出可选地图与先验 PCD
#       --stop                  发零速后结束本链路进程 (不起新节点)
#   -h, --help                  显示本帮助
#
# 环境变量 (同名参数均可由环境变量给默认值):
#   MAP MAP_NAME PRIOR_PCD_FILE PARAMS_FILE LIDAR_CONFIG_FILE NAMESPACE
#   START_SLAM START_RELOC START_LIO START_STATIC
#   USE_RVIZ USE_ROBOT_STATE_PUB USE_CHASSIS USE_JOY USE_COMPOSITION
#   USE_VELOCITY_SMOOTHER USE_FAKE_VEL_TRANSFORM
#   LIDAR_XYZ LIDAR_RPY MAP_TO_ODOM_X MAP_TO_ODOM_Y MAP_TO_ODOM_YAW LOG_LEVEL
#   OPEN_MODE(tab|window) TERMINAL LOG_LEVEL DRY_RUN SRM27_WS_DIR
#
# 说明:
#   - 本脚本走 nav2_stack_launch.py 而不是 nav_real_launch.py, 因为后者没有透传
#     use_pcd_localization / use_lio_odometry (传了也无效), --reloc / --lio 会失效。
#   - 雷达驱动单独启动: 驱动代码里的节点名是 livox_driver_node, 而参数文件顶层键是
#     livox_ros_driver2, ROS 2 按节点名匹配 --params-file, 对不上整段参数被忽略,
#     因此这里显式 -r __node:=livox_ros_driver2; 参数文件里的 user_config_path 用的是
#     launch 专有的 $(find-pkg-share ...) 语法, ros2 run 不会展开, 故再用
#     -p user_config_path:=<绝对路径> 覆盖一次。
#   - map / prior_pcd_file / params_file 一律传绝对路径。
#   - 每个标签页都会先 source 工作空间的 install/setup.bash。
#   - 想建图请只用 --slam; 建图与导航不要各起一套, 否则多个节点争抢 map->odom。
# =============================================================

set -euo pipefail

# ---------- 路径 ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"

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
MAP_REAL_DIR="$PKG_DIR/map/real"
MAP_SIM_DIR="$PKG_DIR/map/simulation"
PCD_REAL_DIR="$PKG_DIR/pcd/real"
SRC_MAP_REAL_DIR="$PKG_SRC/map/real"

# ---------- 可配置项 ----------
OPEN_MODE="${OPEN_MODE:-tab}"                # tab: 同一窗口多标签页; window: 每个命令独立窗口
TERMINAL="${TERMINAL:-gnome-terminal}"
NAMESPACE="${NAMESPACE:-}"
LOG_LEVEL="${LOG_LEVEL:-info}"
DRY_RUN="${DRY_RUN:-0}"

USE_RVIZ="${USE_RVIZ:-1}"
USE_ROBOT_STATE_PUB="${USE_ROBOT_STATE_PUB:-1}"
USE_CHASSIS="${USE_CHASSIS:-1}"
USE_JOY="${USE_JOY:-0}"
USE_COMPOSITION="${USE_COMPOSITION:-1}"
USE_VELOCITY_SMOOTHER="${USE_VELOCITY_SMOOTHER:-1}"
USE_FAKE_VEL_TRANSFORM="${USE_FAKE_VEL_TRANSFORM:-1}"

START_SLAM="${START_SLAM:-0}"
START_RELOC="${START_RELOC:-0}"
START_LIO="${START_LIO:-0}"
START_STATIC="${START_STATIC:-0}"
# 命令行是否显式给过定位方式。没给才在解析之后套用默认 --lio
# (默认值不能在这里就写进 START_LIO, 否则 --slam 会叠成"两个模式同时开")。
MODE_GIVEN=0

MAP_NAME="${MAP_NAME:-${MAP:-}}"
PRIOR_PCD_FILE="${PRIOR_PCD_FILE:-}"
PARAMS_FILE="${PARAMS_FILE:-}"
LIDAR_CONFIG_FILE="${LIDAR_CONFIG_FILE:-}"
MAP_YAML=""
PRIOR_PCD=""

# SRM 实车雷达安装位姿 (与 real_robot_state_publisher_launch.py 默认值一致)
LIDAR_XYZ="${LIDAR_XYZ:-0.15 -0.15 0.22}"
LIDAR_RPY="${LIDAR_RPY:--0.06981317007977318 0.0 -1.5707963267948966}"
# 起步位姿: map->odom 静态变换。仅 --lio / --static 模式下有意义,
# 默认 0 0 0 = 认为 Point-LIO 的起点就在地图原点。
# 若车停在别处, 必须改成"该点在 map 系下的位姿", 否则 RViz 里车不在真实位置,
# 全局代价地图也会错位。
MAP_TO_ODOM_X="${MAP_TO_ODOM_X:-0.0}"
MAP_TO_ODOM_Y="${MAP_TO_ODOM_Y:-0.0}"
MAP_TO_ODOM_YAW="${MAP_TO_ODOM_YAW:-0.0}"

ROBOT_STATE_PUB_PATTERN='[r]eal_robot_state_publisher_launch\.py'
LIVOX_PATTERN='[l]ivox_ros_driver2'
CHASSIS_PATTERN='[s]rm27_nav_protocol'
NAV_PATTERN='[n]av2_stack_launch\.py'
RVIZ_PATTERN='[r]viz_launch\.py'
JOY_PATTERN='[j]oy_teleop_launch\.py'

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

# 地图候选: maps/<名>/<名>.yaml 与 maps/<名>.yaml (现场采集的实车地图)
list_map_candidates() {
  local f
  for f in "$WS_DIR"/maps/*/*.yaml "$WS_DIR"/maps/*.yaml "$MAP_REAL_DIR"/*.yaml; do
    [ -f "$f" ] || continue
    # maps/<名字>/<名字>.yaml 这种布局只认"目录名与 yaml 同名"的那份,
    # 否则 rosbag 的 maps/<bag>/metadata.yaml 会被误当成地图。
    case "$f" in
      "$WS_DIR"/maps/*/*)
        [ "$(basename "$(dirname "$f")")" = "$(basename "$f" .yaml)" ] || continue
        ;;
    esac
    printf '%s\n' "$f"
  done
}

resolve_map() {
  local name="$1" cand
  if [ "${name#/}" != "$name" ]; then          # 绝对路径
    [ -f "$name" ] && { readlink -f "$name"; return 0; }
    return 1
  fi
  if [ "${name#*/}" != "$name" ]; then         # 相对路径不受支持
    return 1
  fi
  for cand in \
    "$WS_DIR/maps/$name/$name.yaml" \
    "$WS_DIR/maps/$name.yaml" \
    "$MAP_REAL_DIR/$name.yaml" \
    "$MAP_SIM_DIR/$name.yaml"; do
    [ -f "$cand" ] && { readlink -f "$cand"; return 0; }
  done
  return 1
}

resolve_prior_pcd() {
  local name="$1" cand
  if [ "${name#/}" != "$name" ]; then
    [ -f "$name" ] && { readlink -f "$name"; return 0; }
    return 1
  fi
  for cand in \
    "$WS_DIR/maps/${name}_mapframe.pcd" \
    "$WS_DIR/maps/$name.pcd" \
    "$PCD_REAL_DIR/$name.pcd"; do
    [ -f "$cand" ] && { readlink -f "$cand"; return 0; }
  done
  return 1
}

list_maps() {
  echo "可选地图 (名称去掉 .yaml 后作为 --map 的值):"
  echo
  local f name pcd seen="" found=0
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    name="$(basename "$f" .yaml)"
    case " $seen " in *" $name "*) continue ;; esac
    seen="$seen $name"
    pcd="无"
    if [ -f "$WS_DIR/maps/${name}_mapframe.pcd" ] || [ -f "$WS_DIR/maps/$name.pcd" ] \
       || [ -f "$PCD_REAL_DIR/$name.pcd" ]; then
      pcd="有"
    fi
    printf '  %-22s 先验PCD: %-3s %s\n' "$name" "$pcd" "$f"
    found=1
  done < <(list_map_candidates)
  if [ "$found" = "0" ]; then
    echo "  (未找到任何 .yaml 地图)"
  fi
  echo
  echo "地图目录  : $WS_DIR/maps  (支持 maps/<名字>.yaml 与 maps/<名字>/<名字>.yaml)"
  echo "          : $MAP_REAL_DIR"
  echo "先验PCD   : $WS_DIR/maps/<名字>_mapframe.pcd / $PCD_REAL_DIR/<名字>.pcd"
  echo "注意: --reloc 需要先验 PCD, 且该 PCD 必须已经在地图坐标系下。"
}

# ---------- 停止 ----------
# 先发零速再停进程: srm27_nav_protocol 没有超时清零, 直接杀节点车会保持最后速度。
stop_real_nav() {
  echo "[停止] 先发零速到 cmd_vel_chassis (防止底盘保持最后速度)..."
  if command -v ros2 >/dev/null 2>&1 && [ -f "$WS_DIR/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    ( set +u; source "$WS_DIR/install/setup.bash" >/dev/null 2>&1
      timeout 2 ros2 topic pub -r 20 /cmd_vel_chassis geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 ) || true
  else
    echo "[警告] 无法发送零速 (ros2 不可用或工作空间未编译), 请手动确认底盘已停。" >&2
  fi

  local patterns=(
    "$LIVOX_PATTERN"
    "$ROBOT_STATE_PUB_PATTERN"
    "$CHASSIS_PATTERN"
    "$NAV_PATTERN"
    "$RVIZ_PATTERN"
    "$JOY_PATTERN"
    '[n]av2_container'
    '[p]oint_lio'
    '[l]oam_interface'
    '[s]ensor_scan_generation'
    '[s]mall_gicp_relocalization'
    '[t]errain_analysis'
    '[l]ifecycle_manager'
    '[f]ake_vel_transform'
    '[c]ontroller_server'
    '[p]lanner_server'
    '[b]t_navigator'
    '[v]elocity_smoother'
    '[b]ehavior_server'
    '[w]aypoint_follower'
    '[m]ap_server'
    '[r]obot_state_publisher'
  )
  local p
  for p in "${patterns[@]}"; do
    pkill -INT -u "$(id -u)" -f "$p" 2>/dev/null || true
  done
  sleep 5
  local killed=0
  for p in "${patterns[@]}"; do
    if pgrep -u "$(id -u)" -f "$p" >/dev/null 2>&1; then
      pkill -KILL -u "$(id -u)" -f "$p" 2>/dev/null || true
      killed=1
    fi
  done
  if [ "$killed" = "1" ]; then
    echo "[停止] 部分节点未响应 SIGINT, 已强制结束。"
  fi
  echo "[停止] 完成。请确认终端里的标签页也已退出 (必要时手动 Ctrl-C)。"
}

# ---------- 命令行参数 ----------
while [ $# -gt 0 ]; do
  case "$1" in
    -m|--map)         MAP_NAME="${2:?--map 需要地图名或绝对路径}"; shift 2 ;;
    --prior-pcd)      PRIOR_PCD_FILE="${2:?--prior-pcd 需要名字或路径}"; shift 2 ;;
    -p|--params)      PARAMS_FILE="${2:?--params 需要绝对路径}"; shift 2 ;;
    --map-to-odom)
      MAP_TO_ODOM_X="${2:?--map-to-odom 需要 X Y YAW}"
      MAP_TO_ODOM_Y="${3:?--map-to-odom 需要 X Y YAW}"
      MAP_TO_ODOM_YAW="${4:?--map-to-odom 需要 X Y YAW}"
      shift 4 ;;
    # 模式开关各自独立置位, 不互相覆盖: 同时给两个会被下面的互斥检查明确拒绝,
    # 比"后者悄悄覆盖前者"更适合实车 (起错定位方式 = 车不动或 TF 打架)。
    --slam)           START_SLAM=1; MODE_GIVEN=1; shift ;;
    --reloc|--relocalization) START_RELOC=1; MODE_GIVEN=1; shift ;;
    --lio|--lio-only) START_LIO=1; MODE_GIVEN=1; shift ;;
    --static)         START_STATIC=1; MODE_GIVEN=1; shift ;;
    --lidar-xyz)      LIDAR_XYZ="${2:?--lidar-xyz 需要 \"X Y Z\"}"; shift 2 ;;
    --lidar-rpy)      LIDAR_RPY="${2:?--lidar-rpy 需要 \"R P Y\"}"; shift 2 ;;
    --lidar-config)   LIDAR_CONFIG_FILE="${2:?--lidar-config 需要路径}"; shift 2 ;;
    --namespace)      NAMESPACE="${2:?--namespace 需要命名空间}"; shift 2 ;;
    --rviz)           USE_RVIZ=1; shift ;;
    --no-rviz)        USE_RVIZ=0; shift ;;
    --robot-state-pub)    USE_ROBOT_STATE_PUB=1; shift ;;
    --no-robot-state-pub) USE_ROBOT_STATE_PUB=0; shift ;;
    --chassis)        USE_CHASSIS=1; shift ;;
    --no-chassis)     USE_CHASSIS=0; shift ;;
    --joy)            USE_JOY=1; shift ;;
    --no-joy)         USE_JOY=0; shift ;;
    --smoother)       USE_VELOCITY_SMOOTHER=1; shift ;;
    --no-smoother)    USE_VELOCITY_SMOOTHER=0; shift ;;
    --fake-vel-transform)    USE_FAKE_VEL_TRANSFORM=1; shift ;;
    --no-fake-vel-transform) USE_FAKE_VEL_TRANSFORM=0; shift ;;
    --composition)    USE_COMPOSITION=1; shift ;;
    --no-composition) USE_COMPOSITION=0; shift ;;
    --log-level)      LOG_LEVEL="${2:?--log-level 需要级别}"; shift 2 ;;
    -n|--check)       DRY_RUN=1; shift ;;
    --list-maps)      list_maps; exit 0 ;;
    --stop)           stop_real_nav; exit 0 ;;
    -h|--help)        usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 -h 查看用法)" >&2; exit 2 ;;
  esac
done

# ---------- 定位方式互斥 ----------
to01() { if is_true "$1"; then printf '1'; else printf '0'; fi; }
SLAM_ON="$(to01 "$START_SLAM")"
RELOC_ON="$(to01 "$START_RELOC")"
LIO_ON="$(to01 "$START_LIO")"
STATIC_ON="$(to01 "$START_STATIC")"

# 一个模式都没给 (命令行没给、环境变量也没给) -> 默认 --lio, 实车推荐。
if [ $((SLAM_ON + RELOC_ON + LIO_ON + STATIC_ON)) -eq 0 ] && [ "$MODE_GIVEN" = "0" ]; then
  LIO_ON=1
  START_LIO=1
  echo "[提示] 未指定定位方式, 默认按 --lio 启动 (Point-LIO 里程计 + 静态 map->odom)。"
  echo "[提示] 其他选择: --reloc / --slam / --static, 见 --help。"
fi

if [ $((SLAM_ON + RELOC_ON + LIO_ON + STATIC_ON)) -gt 1 ]; then
  cat >&2 <<'EOF'
[错误] --slam / --reloc / --lio / --static 互斥, 因为它们都决定 map->odom 由谁发布:
[错误]   --slam  : slam_toolbox 发布 map->odom (边跑边建图)
[错误]   --reloc : small_gicp 与先验 PCD 配准后发布 map->odom
[错误]   --lio   : Point-LIO 只给里程计, map->odom 用静态 TF
[错误]   --static: 只加载栅格图, map->odom 用静态 TF, 且没有里程计来源
[错误] 同时开启会导致 TF 冲突。
[错误] 只选一个; 若没在命令行给多个, 检查环境变量 START_SLAM / START_RELOC /
[错误] START_LIO / START_STATIC 是否被导出过。
EOF
  exit 1
fi

if [ "$STATIC_ON" = "1" ]; then
  echo "[警告] --static 模式下没有任何里程计来源, odom->base_link 不会有人发布;" >&2
  echo "[警告] 除非车体模块自己发里程计, 否则导航起不来, 车也不会动。" >&2
fi

SLAM_ARG="$(bool_arg "$SLAM_ON")"
RELOC_ARG="$(bool_arg "$RELOC_ON")"
LIO_ARG="$(bool_arg "$LIO_ON")"
COMPOSITION_ARG="$(bool_arg "$USE_COMPOSITION")"
RVIZ_ARG="$(bool_arg "$USE_RVIZ")"
SMOOTHER_ARG="$(bool_arg "$USE_VELOCITY_SMOOTHER")"
FAKE_ARG="$(bool_arg "$USE_FAKE_VEL_TRANSFORM")"

# 导航链路的最终出口必须落在底盘节点订阅的 cmd_vel_chassis 上。
if [ "$USE_FAKE_VEL_TRANSFORM" = "1" ]; then
  CMD_VEL_NAV_TOPIC="cmd_vel_nav2_result"    # 由 fake_vel_transform 转成 cmd_vel_chassis
else
  CMD_VEL_NAV_TOPIC="cmd_vel_chassis"        # 直接对准底盘
fi

case "$LOG_LEVEL" in
  debug|info|warn|error|fatal) ;;
  *) echo "[错误] --log-level 只支持 debug/info/warn/error/fatal" >&2; exit 2 ;;
esac

# ---------- 默认路径 ----------
if [ -z "$PARAMS_FILE" ]; then
  PARAMS_FILE="$PKG_DIR/config/real/nav2_params_srm.yaml"
  if [ ! -f "$PARAMS_FILE" ]; then
    PARAMS_FILE="$PKG_SRC/config/real/nav2_params_srm.yaml"
  fi
fi
if [ -z "$LIDAR_CONFIG_FILE" ]; then
  LIDAR_CONFIG_FILE="$PKG_DIR/config/real/mid360_user_config.json"
  if [ ! -f "$LIDAR_CONFIG_FILE" ]; then
    LIDAR_CONFIG_FILE="$PKG_SRC/config/real/mid360_user_config.json"
  fi
fi

# ---------- 环境检查 ----------
if [ "$DRY_RUN" != "1" ] && [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[错误] 未找到 $WS_DIR/install/setup.bash" >&2
  echo "[提示] 先构建工作空间:" >&2
  echo "[提示]   cd $WS_DIR && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2" >&2
  echo "[提示] 或用 SRM27_WS_DIR=/path/to/ws 指定工作空间。" >&2
  exit 1
fi
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[警告] 未找到 $WS_DIR/install/setup.bash (仅 --check 可继续)。" >&2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[警告] 当前 shell 里找不到 ros2; 每个标签页都会 source install/setup.bash, 正常可继续。" >&2
fi

if [ "$DRY_RUN" != "1" ] && ! command -v "$TERMINAL" >/dev/null 2>&1; then
  echo "[错误] 未找到终端程序: $TERMINAL" >&2
  echo "[提示] 可用 TERMINAL=konsole 或 TERMINAL=xfce4-terminal 指定。" >&2
  exit 1
fi

if [ "$DRY_RUN" != "1" ] && [ "$USE_RVIZ" = "1" ] \
   && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "[警告] 未检测到 DISPLAY / WAYLAND_DISPLAY, RViz 可能起不来 (可加 --no-rviz)。" >&2
fi

# ---------- 参数 / 模型 / 地图 校验 ----------
# 先查"是不是绝对路径": nav2 的参数文件会被 launch 多次重写落盘, 相对路径在不同 cwd 下
# 会解析成不同文件, 报错也更难懂, 所以这里先给规则明确的那条错误。
if [ "${PARAMS_FILE#/}" = "$PARAMS_FILE" ]; then
  echo "[错误] --params 必须是绝对路径: $PARAMS_FILE" >&2
  exit 1
fi
if [ ! -f "$PARAMS_FILE" ]; then
  echo "[错误] Nav2 参数文件不存在: $PARAMS_FILE" >&2
  exit 1
fi

if [ ! -f "$LIDAR_CONFIG_FILE" ]; then
  echo "[错误] 雷达驱动配置不存在: $LIDAR_CONFIG_FILE" >&2
  echo "[提示] 用 --lidar-config <路径> 指定。" >&2
  exit 1
fi

# 驱动外参必须全零: 非零时驱动自己转一次点云、URDF 再转一次,
# 而且 real_robot_state_publisher_launch.py 会直接拒绝启动 —— 这里提前报错更清楚。
# 自检脚本把"文件坏了/缺字段/外参非零"分别说明, 避免都归到"外参非零"上误导排查。
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
    echo "[错误] 驱动外参必须全零, 安装位姿由 --lidar-xyz / --lidar-rpy 承担," >&2
    echo "[错误] 否则点云会被旋转两次。" >&2
    exit 1
  fi
else
  echo "[警告] 未找到 python3, 跳过雷达驱动外参自检。" >&2
fi

# 参数文件与车体模型必须配套: 默认 SRM 模型发 base_link / livox_*;
# 上游那份 nav2_params_upstream.yaml 要的是 base_footprint / gimbal_yaw / front_mid360,
# 混用会满屏 TF 报错且重定位卡死。
if [ "$USE_ROBOT_STATE_PUB" = "1" ] && ! grep -qE "livox_imu|base_link" "$PARAMS_FILE"; then
  cat >&2 <<EOF
[错误] 参数文件与车体模型不配套:
[错误]   参数文件 $PARAMS_FILE
[错误]   里面没有 base_link / livox_imu, 看起来是 nav2_params_upstream.yaml 那份
[错误]   (需要 base_footprint / gimbal_yaw / front_mid360), 而本脚本启动的
[错误]   SRM 模型只发 base_link / livox_frame / livox_imu / livox_scan。
[错误] 请用默认的 config/real/nav2_params_srm.yaml。
EOF
  exit 1
fi

# 地图: --slam 仍要求该参数存在, 但不加载栅格图
if [ -z "$MAP_NAME" ]; then
  # 自动选择: 只有一个候选就用它, 多个则取最近修改的一份, 并明确打印出来。
  best=""; best_time=-1; count=0
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    count=$((count + 1))
    t="$(stat -c %Y "$f" 2>/dev/null || echo 0)"
    if [ "$t" -gt "$best_time" ]; then best_time="$t"; best="$f"; fi
  done < <(list_map_candidates)
  if [ "$count" = "0" ]; then
    echo "[错误] 没有自动找到任何实车地图, 请用 -m 指定。" >&2
    list_maps >&2
    exit 1
  fi
  MAP_NAME="$(basename "$best" .yaml)"
  if [ "$count" = "1" ]; then
    echo "[提示] 自动选择唯一的地图: $MAP_NAME"
  else
    echo "[提示] 找到 $count 份地图, 自动选择最近修改的: $MAP_NAME ($best)"
    echo "[提示] 要用其他地图请加 -m <名字>, 用 --list-maps 查看全部。"
  fi
fi

if MAP_YAML="$(resolve_map "$MAP_NAME")"; then
  case "$MAP_YAML" in
    "$MAP_SIM_DIR"/*)
      echo "[警告] 选中的是仿真地图: $MAP_YAML" >&2
      echo "[警告] 实车请用 maps/ 下的场地地图 (-m <名字>, 或 --list-maps 查看)。" >&2
      ;;
  esac
else
  if [ "$SLAM_ON" = "1" ]; then
    MAP_YAML="$SRC_MAP_REAL_DIR/$MAP_NAME.yaml"   # 占位: slam 不加载栅格图
    echo "[提示] SLAM 模式不加载先验栅格图, 未找到 $MAP_NAME.yaml 属正常。"
  else
    echo "[错误] 未找到地图: $MAP_NAME" >&2
    list_maps >&2
    exit 1
  fi
fi

# 先验点云: 仅重定位需要
if [ "$RELOC_ON" = "1" ]; then
  [ -n "$PRIOR_PCD_FILE" ] || PRIOR_PCD_FILE="$MAP_NAME"
  if ! PRIOR_PCD="$(resolve_prior_pcd "$PRIOR_PCD_FILE")"; then
    echo "[错误] 重定位需要先验点云, 但未找到: $PRIOR_PCD_FILE" >&2
    echo "[错误] 已查找: $WS_DIR/maps/${PRIOR_PCD_FILE}_mapframe.pcd" >&2
    echo "[错误]           $WS_DIR/maps/$PRIOR_PCD_FILE.pcd" >&2
    echo "[错误]           $PCD_REAL_DIR/$PRIOR_PCD_FILE.pcd" >&2
    echo "[错误] 用 --prior-pcd <路径> 指定。" >&2
    echo "[错误] 注意: PCD 必须在地图坐标系下, Point-LIO camera_init 系的原始点云不能直接用。" >&2
    exit 1
  fi
  # 坐标系自检: mapframe 版本头部有标注, camera_init 系的原始点云通常没有。
  if ! head -c 1024 "$PRIOR_PCD" 2>/dev/null | grep -aqiE "map frame|map_frame|地图坐标系"; then
    echo "[警告] $PRIOR_PCD 头部未标注坐标系;" >&2
    echo "[警告] 若它是 Point-LIO camera_init 系的原始点云, GICP 初值会错, 请先转到地图坐标系。" >&2
  fi
else
  PRIOR_PCD="${PRIOR_PCD_FILE:-$PCD_REAL_DIR/$MAP_NAME.pcd}"   # 仅供 launch 声明
fi

# 底盘串口: 设备存在性与权限, 只告警不阻断 (车可能还没插上 / 只做链路验证)
if [ "$USE_CHASSIS" = "1" ]; then
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

if [ "$USE_CHASSIS" = "0" ]; then
  echo "[警告] 未启动底盘串口节点: /cmd_vel_chassis 将没有消费者, 车不会动。" >&2
fi

# srm27_nav_protocol.launch.py 里 namespace 写死为空, 且订阅的是全局 cmd_vel_chassis。
# 一旦导航栈加了命名空间, 速度出口变成 /<ns>/cmd_vel_chassis, 底盘就收不到了。
if [ -n "$NAMESPACE" ] && [ "$USE_CHASSIS" = "1" ]; then
  echo "[警告] 指定了 --namespace=$NAMESPACE, 但底盘串口节点固定在根命名空间下," >&2
  echo "[警告] 它订阅 /cmd_vel_chassis, 而导航出口会变成 /$NAMESPACE/cmd_vel_chassis," >&2
  echo "[警告] 除非另有转发, 否则车不会动。实车建议不加 --namespace。" >&2
fi

# ---------- 组装命令 ----------
NS_ARG=""
[ -n "$NAMESPACE" ] && NS_ARG="namespace:=$NAMESPACE"

# 1) 雷达驱动: nav2_stack_launch.py 不负责起驱动, 必须单独启动。
DRIVER_CMD="ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args"
DRIVER_CMD="$DRIVER_CMD -r __node:=livox_ros_driver2"
DRIVER_CMD="$DRIVER_CMD --params-file $(q "$PARAMS_FILE")"
DRIVER_CMD="$DRIVER_CMD -p user_config_path:=$(q "$LIDAR_CONFIG_FILE")"
if [ -n "$NAMESPACE" ]; then
  DRIVER_CMD="$DRIVER_CMD -r __ns:=$(q "$NAMESPACE")"
fi

# 2) 车体 TF: 提供 base_link / livox_frame / livox_imu / livox_scan。
#    缺了这些 frame, small_gicp 会在构造时一直等 TF 而卡住整个 launch。
RSP_CMD="ros2 launch srm27_nav_bringup real_robot_state_publisher_launch.py"
RSP_CMD="$RSP_CMD use_sim_time:=False"
RSP_CMD="$RSP_CMD params_file:=$(q "$PARAMS_FILE")"
RSP_CMD="$RSP_CMD lidar_config:=$(q "$LIDAR_CONFIG_FILE")"
RSP_CMD="$RSP_CMD lidar_xyz:=$(q "$LIDAR_XYZ")"
RSP_CMD="$RSP_CMD lidar_rpy:=$(q "$LIDAR_RPY")"

# 3) 底盘串口: cmd_vel_chassis 的唯一消费者, 把速度写进串口发给 C 板。
CHASSIS_CMD="ros2 launch srm27_nav_protocol srm27_nav_protocol.launch.py"

# 4) 导航栈
NAV_CMD="ros2 launch srm27_nav_bringup nav2_stack_launch.py"
[ -n "$NS_ARG" ] && NAV_CMD="$NAV_CMD $NS_ARG"
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
NAV_CMD="$NAV_CMD use_fake_vel_transform:=$FAKE_ARG"
NAV_CMD="$NAV_CMD use_velocity_smoother:=$SMOOTHER_ARG"
NAV_CMD="$NAV_CMD cmd_vel_nav_topic:=$CMD_VEL_NAV_TOPIC"
NAV_CMD="$NAV_CMD map_to_odom_x:=$MAP_TO_ODOM_X"
NAV_CMD="$NAV_CMD map_to_odom_y:=$MAP_TO_ODOM_Y"
NAV_CMD="$NAV_CMD map_to_odom_yaw:=$MAP_TO_ODOM_YAW"
NAV_CMD="$NAV_CMD log_level:=$LOG_LEVEL"

# 5) RViz
RVIZ_CMD="ros2 launch srm27_nav_bringup rviz_launch.py"
[ -n "$NS_ARG" ] && RVIZ_CMD="$RVIZ_CMD $NS_ARG"
RVIZ_CMD="$RVIZ_CMD use_sim_time:=False"
RVIZ_CMD="$RVIZ_CMD rviz_config:=$(q "$PKG_DIR/rviz/nav2_default_view.rviz")"

# 6) 手柄遥控: 底盘订阅的是 cmd_vel_chassis, 这里显式指过去, 否则推杆车不动。
JOY_CMD="ros2 launch srm27_nav_bringup joy_teleop_launch.py"
[ -n "$NS_ARG" ] && JOY_CMD="$JOY_CMD $NS_ARG"
JOY_CMD="$JOY_CMD use_sim_time:=False"
JOY_CMD="$JOY_CMD joy_vel:=cmd_vel_chassis joy_config_file:=$(q "$PARAMS_FILE")"

# ---------- 配置摘要 ----------
if [ "$SLAM_ON" = "1" ]; then
  MODE_DESC="SLAM 建图 (slam_toolbox 发布 map->odom)"
elif [ "$RELOC_ON" = "1" ]; then
  MODE_DESC="先验 PCD 重定位 (small_gicp 发布 map->odom)"
elif [ "$LIO_ON" = "1" ]; then
  MODE_DESC="Point-LIO 里程计 + 静态 map->odom (实车推荐)"
else
  MODE_DESC="静态地图, 无里程计来源"
fi

if [ "$USE_VELOCITY_SMOOTHER" = "1" ]; then
  VEL_CHAIN="controller_server → cmd_vel_controller → velocity_smoother"
else
  VEL_CHAIN="controller_server (跳过 velocity_smoother)"
fi
if [ "$USE_FAKE_VEL_TRANSFORM" = "1" ]; then
  VEL_CHAIN="$VEL_CHAIN → cmd_vel_nav2_result → fake_vel_transform → cmd_vel_chassis"
else
  VEL_CHAIN="$VEL_CHAIN → cmd_vel_chassis (不经 fake_vel_transform)"
fi

echo "================= SRM 实车 NAV 导航测试 ================="
echo "  工作空间    : $WS_DIR"
echo "  定位方式    : $MODE_DESC"
echo "  slam        : $SLAM_ARG   use_pcd_localization: $RELOC_ARG   use_lio_odometry: $LIO_ARG"
echo "  地图 (yaml) : $MAP_YAML$([ "$SLAM_ON" = "1" ] && echo '   (SLAM 模式未加载)')"
if [ "$RELOC_ON" = "1" ]; then
  echo "  先验 PCD    : $PRIOR_PCD"
else
  echo "  先验 PCD    : $PRIOR_PCD   (未使用)"
fi
echo "  参数文件    : $PARAMS_FILE"
echo "  雷达配置    : $LIDAR_CONFIG_FILE"
if [ "$USE_ROBOT_STATE_PUB" = "1" ]; then
  echo "  车体 TF     : real_robot_state_publisher_launch.py (SRM 模型)"
  echo "  雷达安装    : xyz=[$LIDAR_XYZ]  rpy=[$LIDAR_RPY]"
else
  echo "  车体 TF     : 不启动 (假定其他模块已发布 URDF/TF)"
fi
echo "  底盘串口    : $([ "$USE_CHASSIS" = "1" ] && echo 'srm27_nav_protocol.launch.py' || echo '不启动')"
echo "  命名空间    : ${NAMESPACE:-<根命名空间>}"
echo "  组合节点    : $COMPOSITION_ARG    RViz: $RVIZ_ARG    手柄: $USE_JOY"
if [ "$RELOC_ON" != "1" ] && [ "$SLAM_ON" != "1" ]; then
  echo "  map->odom   : x=$MAP_TO_ODOM_X y=$MAP_TO_ODOM_Y yaw=$MAP_TO_ODOM_YAW (静态)"
  if [ "$LIO_ON" = "1" ] && [ "$MAP_TO_ODOM_X" = "0.0" ] && [ "$MAP_TO_ODOM_Y" = "0.0" ] \
     && [ "$MAP_TO_ODOM_YAW" = "0.0" ]; then
    echo "  [提示] map->odom 取的是原点。若车不是停在地图原点, 请用"
    echo "  [提示] --map-to-odom X Y YAW 指定「起步点在 map 系下的位姿」, 否则 RViz 里车的位置是错的。"
  fi
fi
echo "  速度链路    : $VEL_CHAIN"
echo "========================================================="

# ---------- 启动 ----------
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

STEP_TITLES=()
STEP_PATTERNS=()
STEP_CMDS=()
add_step() {
  STEP_TITLES+=("$1")
  STEP_PATTERNS+=("$2")
  STEP_CMDS+=("$3")
}

add_step "SRM27 雷达驱动 (Livox MID360)" "$LIVOX_PATTERN" "$DRIVER_CMD"
if [ "$USE_ROBOT_STATE_PUB" = "1" ]; then
  add_step "SRM27 车体 TF" "$ROBOT_STATE_PUB_PATTERN" "$RSP_CMD"
fi
if [ "$USE_CHASSIS" = "1" ]; then
  add_step "SRM27 底盘串口" "$CHASSIS_PATTERN" "$CHASSIS_CMD"
fi
add_step "SRM27 导航栈 (Nav2)" "$NAV_PATTERN" "$NAV_CMD"
if [ "$USE_RVIZ" = "1" ]; then
  add_step "SRM27 RViz" "$RVIZ_PATTERN" "$RVIZ_CMD"
fi
if [ "$USE_JOY" = "1" ]; then
  add_step "SRM27 手柄遥控" "$JOY_PATTERN" "$JOY_CMD"
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

cat <<EOF

启动流程处理完成 (终端模式: $OPEN_MODE)。

安全提醒:
  - 底盘没有超时清零: 杀掉发速度的节点后车会保持最后速度继续跑。
    停任何节点前先发零速, 或直接按急停:
      ros2 topic pub -r 20 /cmd_vel_chassis geometry_msgs/msg/Twist "{}"
  - 一键停整条链路 (先发零速再结束进程):
      ./script/start_real_nav.sh --stop

链路自检 (新终端):
  cd '$WS_DIR' && source install/setup.bash
  ros2 topic hz /livox/lidar                                # 雷达点云 ~10 Hz
  ros2 topic hz /livox/imu                                  # 雷达 IMU ~200 Hz
  ros2 run tf2_ros tf2_echo odom base_link                   # LIO 里程计 TF (--lio/--reloc)
  ros2 run tf2_ros tf2_echo map base_link                    # 完整 TF 链
  ros2 topic echo /cmd_vel_controller --once                 # 控制器原始输出
  ros2 topic echo /cmd_vel_chassis --once                    # 底盘实际收到的速度 (最终出口)
  ros2 topic hz /cmd_vel_chassis                             # 应有持续输出
  ros2 topic echo /odometry --once                           # 里程计
  ros2 node list                                             # 各节点是否都起来了

上层状态:
  ros2 topic echo /local_costmap/scan --once                # 局部代价地图是否有数据
  ros2 topic echo /terrain_map --once                       # 地形分析输出
  ros2 lifecycle get /map_server                            # map_server 是否 active

下发目标点:
  在 RViz 用 "Nav2 Goal" / "2D Goal Pose" 点一个近处目标,
  先确认车头方向与正负号都对 (y 方向横移、原地转向是否与预期一致), 再放开跑。

清理残留 (本条会结束实车链路节点, 含 RViz; 请不要在仿真同时运行时用):
  ./script/start_real_nav.sh --stop
  ./script/kill_rviz.sh
EOF
