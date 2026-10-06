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
#   ./script/start_real_slam.sh -n                    # 只解析并打印将执行的命令, 不启动
#   ./script/start_real_slam.sh --list-maps           # 列出工作区现有地图
#   ./script/start_real_slam.sh --stop                # 先发零速, 再结束建图链路节点
#
# 参数:
#       --save <名字>           保存当前地图到 maps/<名字>/ (要求建图链路正在跑)
#       --out-dir <绝对路径>    存图输出目录, 默认 <工作空间>/maps/<名字>
#       --wait <秒>             存图前最多等 slam_toolbox 服务出现的秒数 (默认 8; 0 = 不等)
#       --force                 存图时允许覆盖已存在的同名地图文件
#   -p, --params <绝对路径>     建图参数, 默认 config/real/mapping_params.yaml
#       --lidar-config <路径>   雷达驱动网络配置, 默认 config/real/mid360_user_config.json
#       --lidar-xyz "X Y Z"     雷达在 base_link 下的安装位置 (默认 0.15 -0.15 0.22)
#       --lidar-rpy "R P Y"     雷达安装角, 弧度 (默认 -0.06981317007977318 0 -1.5707963267948966)
#       --no-lidar              不起雷达驱动 (回放 rosbag 时用, 配合 --use-sim-time)
#       --use-sim-time          使用 /clock (回放 bag 时用, 默认 False)
#       --save-pcd              让 Point-LIO 退出时落盘累积点云 (见下文说明)
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
#   LIDAR_XYZ LIDAR_RPY START_LIDAR USE_SIM_TIME SAVE_PCD
#   USE_RVIZ USE_CHASSIS USE_JOY JOY_DEV
#   OPEN_MODE(tab|window) TERMINAL DRY_RUN SRM27_WS_DIR
#
# 说明:
#   - 雷达驱动、车体 TF、Point-LIO、slam_toolbox 全在 real_mapping_launch.py 里,
#     本脚本只负责"参数校验 + 补齐底盘/手柄 + 存图/停止", 不再重复起这些节点。
#   - 雷达驱动外参必须为零 (安装位姿走 --lidar-xyz / --lidar-rpy),
#     否则点云被驱动和 URDF 各转一次, 建图必然重影; launch 会直接拒绝, 本脚本提前报错。
#   - 所有路径一律传绝对路径; 每个标签页都会先 source 工作空间的 install/setup.bash。
#   - 存图的 PCD (--save-pcd) 落在 src/srm27_navigation/point_lio/PCD/scans.pcd,
#     它是 Point-LIO 自己坐标系下的原始点云, **不是**地图坐标系下的先验点云,
#     直接拿给 --reloc 用会错位; 需要先按 docs 里的流程转到地图坐标系。
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

PARAMS_FILE="${PARAMS_FILE:-}"
LIDAR_CONFIG_FILE="${LIDAR_CONFIG_FILE:-}"
LIDAR_XYZ="${LIDAR_XYZ:-0.15 -0.15 0.22}"
LIDAR_RPY="${LIDAR_RPY:--0.06981317007977318 0.0 -1.5707963267948966}"

SAVE_MAP_NAME="${SAVE_MAP_NAME:-}"
MAP_OUT_DIR="${MAP_OUT_DIR:-}"
FORCE_SAVE="${FORCE_SAVE:-0}"
# 存图前最多等 slam_toolbox 服务出现的秒数: 链路刚起时服务/发现都要一点时间,
# 一锤子判断会得到"找不到服务"的误报。0 = 不等, 立刻判断。
WAIT_FOR_SLAM="${WAIT_FOR_SLAM:-8}"
# slam_toolbox 节点名 (real_mapping_launch.py 里写死), 服务名随之确定。
SLAM_NODE_NAME="slam_toolbox"
SLAM_SERVICE="/slam_toolbox/save_map"

# ---------- 进程识别 ----------
# 方括号包住首字符, 避免 pgrep 匹配到脚本自身命令行。
MAPPING_PATTERN='[r]eal_mapping_launch\.py'
NAV_PATTERN='[n]av2_stack_launch\.py|[n]av_real_launch\.py|[n]av2_slam_launch\.py|[s]tart_real_nav\.sh'
CHASSIS_PATTERN='[s]rm27_nav_protocol'
JOY_PATTERN='[j]oy_teleop_launch\.py'
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
  sleep 5

  local killed=0
  if signal_matching KILL "${patterns[@]}"; then
    killed=1
  fi
  if [ "$killed" = "1" ]; then
    echo "[停止] 仍有节点未在 5 秒内响应 SIGINT, 已强制结束。"
  fi
  echo "[停止] 完成。请确认终端里的标签页也已退出 (必要时手动 Ctrl-C)。"
  if is_true "$SAVE_PCD" && [ -f "$PCD_SAVE_PATH" ]; then
    echo "[停止] Point-LIO 累积点云: $PCD_SAVE_PATH"
  fi
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
    -p|--params)     PARAMS_FILE="${2:?--params 需要绝对路径}"; shift 2 ;;
    --lidar-config)  LIDAR_CONFIG_FILE="${2:?--lidar-config 需要路径}"; shift 2 ;;
    --lidar-xyz)     LIDAR_XYZ="${2:?--lidar-xyz 需要 \"X Y Z\"}"; shift 2 ;;
    --lidar-rpy)     LIDAR_RPY="${2:?--lidar-rpy 需要 \"R P Y\"}"; shift 2 ;;
    --lidar)         START_LIDAR=1; shift ;;
    --no-lidar)      START_LIDAR=0; shift ;;
    --use-sim-time)  USE_SIM_TIME=1; shift ;;
    --save-pcd)      SAVE_PCD=1; shift ;;
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

# ---------- 默认路径 ----------
[ -n "$PARAMS_FILE" ] || PARAMS_FILE="$MAP_PARAMS_DEFAULT"
[ -n "$LIDAR_CONFIG_FILE" ] || LIDAR_CONFIG_FILE="$LIDAR_CONFIG_DEFAULT"

# ---------- 存图分支 (与启动互斥: --save 只存图, 不起新节点) ----------
if [ "$ACTION_START" = "0" ]; then
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
MAPPING_CMD="$MAPPING_CMD lidar_xyz:=$(q "$LIDAR_XYZ")"
MAPPING_CMD="$MAPPING_CMD lidar_rpy:=$(q "$LIDAR_RPY")"
MAPPING_CMD="$MAPPING_CMD start_lidar:=$(bool_arg "$START_LIDAR")"
MAPPING_CMD="$MAPPING_CMD use_sim_time:=$CLOCK_ARG"
MAPPING_CMD="$MAPPING_CMD use_rviz:=$(bool_arg "$USE_RVIZ")"
MAPPING_CMD="$MAPPING_CMD save_pcd:=$(bool_arg "$SAVE_PCD")"

# 底盘串口节点固定在根命名空间, 订阅全局 /cmd_vel_chassis。
CHASSIS_CMD="ros2 launch srm27_nav_protocol srm27_nav_protocol.launch.py"

JOY_CMD="ros2 launch srm27_nav_bringup joy_teleop_launch.py"
JOY_CMD="$JOY_CMD use_sim_time:=$CLOCK_ARG"
JOY_CMD="$JOY_CMD joy_vel:=cmd_vel_chassis"
JOY_CMD="$JOY_CMD joy_dev:=$JOY_DEV"
JOY_CMD="$JOY_CMD joy_config_file:=$(q "$JOY_PARAMS_DEFAULT")"

# ---------- 启动摘要 ----------
echo "================= SRM 实车 SLAM 建图 ================="
echo "  工作空间    : $WS_DIR"
echo "  建图参数    : $PARAMS_FILE"
echo "  雷达配置    : $LIDAR_CONFIG_FILE"
echo "  雷达驱动    : $([ "$START_LIDAR" = "1" ] && echo "启动 (真实 MID360)" || echo "不启动 (假定有别的点云源)")"
echo "  雷达安装    : xyz=[$LIDAR_XYZ]  rpy=[$LIDAR_RPY]"
echo "  use_sim_time: $CLOCK_ARG    RViz: $(bool_arg "$USE_RVIZ")"
echo "  底盘串口    : $([ "$USE_CHASSIS" = "1" ] && echo "启动 (cmd_vel_chassis -> 串口)" || echo "不启动")"
echo "  手柄遥控    : $([ "$USE_JOY" = "1" ] && echo "启动 (设备 $JOY_DEV, 输出 cmd_vel_chassis)" || echo "不启动")"
echo "  累积点云    : $([ "$SAVE_PCD" = "1" ] && echo "开 (退出时写 $PCD_SAVE_PATH)" || echo "关")"
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

启动完成。建图请确认:
  - RViz 里已经把车走一圈的区域连成一片墙; 有重影先停下查雷达外参/时间戳。
  - 位姿图在内存里, **不存就没了**: 建完先存图再关。

存图 (建图链路保持运行, 另开终端执行):
  ./script/start_real_slam.sh --save <名字>          # -> maps/<名字>/{pgm,yaml,posegraph,data}
  ./script/start_real_slam.sh --save <名字> --force  # 同名已存在时覆盖

结束 (先发零速再停节点):
  ./script/start_real_slam.sh --stop

链路自检 (新终端):
  cd '$WS_DIR' && source install/setup.bash
  ros2 topic hz /livox/lidar                              # 点云 ~10 Hz
  ros2 topic hz /scan                                     # 二维扫描 (建图的输入)
  ros2 run tf2_ros tf2_echo map odom                      # 漂移看这里: z/roll/pitch 应稳定
  ros2 run tf2_ros tf2_echo odom base_link                # Point-LIO 里程计
  ros2 topic echo /map --once --field info.width          # 地图是否在长
EOF
