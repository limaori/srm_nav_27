#!/usr/bin/env bash
# =============================================================
# 仿真 SLAM 建图一键启动（Gazebo 世界 → Point-LIO 建 3D 点云 + slam_toolbox 建 2D 栅格）
#
# 链路（详见 srm27_nav_bringup/launch/sim_slam_launch.py 的注释）:
#   Gazebo gpu_lidar ─bridge─► livox/lidar ─ign_sim_pointcloud_tool─► velodyne_points
#   livox/imu ─────────────────────────────────────────────────────────┘
#                              ▼
#                          Point-LIO ──► odom->base_link（经 loam_interface + sensor_scan_generation）
#                              └────────► 3D 点云地图: ~/srm_nav_27_data/generated_pcd/point_lio/scans.pcd（退出时落盘）
#                          slam_toolbox ──► 2D 栅格 map（另存 pgm/yaml）
#
# 用法:
#   ./script/start_sim_slam.sh                        # 默认 rmuc_2025 世界, 底盘与云台都不转
#   ./script/start_sim_slam.sh -w rmuc_2025           # 指定世界
#   ./script/start_sim_slam.sh -w rmuc_2025 --no-gui  # 关掉 Gazebo GUI，省一大半 GPU
#   ./script/start_sim_slam.sh --no-clean             # 不清理上次残留
#   ./script/start_sim_slam.sh --chassis-spin         # 底盘 2.0 rad/s 自转 (边转边建图)
#   ./script/start_sim_slam.sh --gimbal-spin          # 云台 -1.0 rad/s 自转 (雷达跟着转)
#   ./script/start_sim_slam.sh --chassis-spin 1.0 --gimbal-spin 0.5   # 自己给转速
#   DRY_RUN=1 ./script/start_sim_slam.sh              # 只打印将执行的命令
#
#   建完图之后（关掉 slam 标签页，Point-LIO 收到 Ctrl-C 才落盘）:
#   ./script/start_sim_slam.sh --collect              # 把 scans.pcd 收回 maps/pcd/
#   ./script/start_sim_slam.sh --collect --name rmuc_2025
#   ./script/start_sim_slam.sh --view maps/pcd/rmuc_2025.pcd   # 在 RViz 里看这个 PCD
#
# 参数:
#   -w, --world <name>     rmuc_2025 / rmuc_2024 / rmul_2024 / rmul_2025 / srm_empty
#   --chassis-spin [rad/s] 底盘恒速自转，不给数值时默认 2.0 rad/s（负值=反向）。
#                          **不加这个参数 = 底盘不转**（默认行为）。
#                          建图栈里没有 MINCO（rotation_cmd 无人订阅），所以脚本把
#                          恒速 wz 注入 fakeyaw 链入口 cmd_vel_nav_fake：
#                          cmd_vel_nav_fake → fake_vel_transform → cmd_vel_nav_body
#                          → srm_cmd_mux → cmd_vel_sim → Gazebo（与导航下发同一条链）。
#                          停转: SPIN_CMD_TOPIC=/<ns>/cmd_vel_nav_fake script/spin_control.sh off
#   --no-chassis-spin      等价于 --chassis-spin 0（显式关掉，覆盖环境变量）。
#   --gimbal-spin [rad/s]  云台恒速自转，不给数值时默认 -1.0 rad/s（负值=反向）。
#                          **不加这个参数 = 云台不转**（默认行为）。
#                          由仿真器 SrmVelocitySystem 直接施加，雷达跟着转。
#   --no-gimbal-spin       等价于 --gimbal-spin 0。
#   --gui / --no-gui       Gazebo 是否带 GUI (默认 no-gui，建图只看 RViz 更流畅)
#   --rviz / --no-rviz     是否启动 RViz (默认启动)
#   --params <路径>        用别的参数文件 (默认 config/simulation/nav2_params_srm.yaml)
#   --no-clean             跳过启动前的 kill_gzb/kill_nav/kill_rviz
#   -o, --out <目录>       --collect 的输出目录 (默认 <工作空间>/maps/pcd)
#   -n, --name <名字>      --collect 的输出文件名 (默认 sim_<世界>_<时间戳>)
#   --collect              把外部数据目录里的 scans.pcd 拷到输出目录并报告
#   --view <文件.pcd>      用 pcl_ros 把 PCD 发成 PointCloud2，配合 RViz 查看
#
# 环境变量:
#   --min-free-gb <数值>   启动前要求的最小可用内存（GB），默认 3；0 = 不检查
#   CHASSIS_SPIN_WZ / GIMBAL_SPIN_WZ   等价于两个自转参数的数值形式
#   CHASSIS_SPIN_READY_TIMEOUT         等 fake_vel_transform 上线的最长秒数 (默认 120)
#   PREKILL=0              等价于 --no-clean
#   DRY_RUN=1              只打印命令
#   TERMINAL               终端程序 (默认 gnome-terminal)
# =============================================================

set -u

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
DATA_DIR="${SRM27_DATA_DIR:-$HOME/srm_nav_27_data}"
PCD_DATA_DIR="$DATA_DIR/generated_pcd/point_lio"
export SRM27_PCD_DIR="$PCD_DATA_DIR"
mkdir -p "$PCD_DATA_DIR"

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

# 底盘 / 云台自转：**默认都不转**（2026-11-05 起；旧版云台默认 2.0 rad/s 一直转已取消）。
# 显式给参数但不带数值时用各自的默认转速：--chassis-spin -> 2.0，--gimbal-spin -> -1.0。
CHASSIS_SPIN_WZ="${CHASSIS_SPIN_WZ:-}"
[ -n "$CHASSIS_SPIN_WZ" ] && CHASSIS_SPIN_GIVEN=1
CHASSIS_SPIN_GIVEN="${CHASSIS_SPIN_GIVEN:-0}"
CHASSIS_SPIN_DEFAULT="2.0"
GIMBAL_SPIN_WZ="${GIMBAL_SPIN_WZ:-}"
[ -n "$GIMBAL_SPIN_WZ" ] && GIMBAL_SPIN_GIVEN=1
GIMBAL_SPIN_GIVEN="${GIMBAL_SPIN_GIVEN:-0}"
GIMBAL_SPIN_DEFAULT="-1.0"
# 等 fake_vel_transform 上线（订阅 cmd_vel_nav_fake）的最长秒数。
CHASSIS_SPIN_READY_TIMEOUT="${CHASSIS_SPIN_READY_TIMEOUT:-120}"
# 建图栈没有 MINCO，自转请求从 fakeyaw 链入口注入（见文件头）。
CHASSIS_SPIN_TOPIC="/${ROBOT_NS}/cmd_vel_nav_fake"

# Point-LIO 通过运行期 SRM27_PCD_DIR 落盘（见 point_lio/src/laserMapping.cpp）
PCD_SRC="$PCD_DATA_DIR/scans.pcd"

log() { printf '%s\n' "$*"; }
die() { printf '[错误] %s\n' "$*" >&2; exit 1; }

usage() {
  # 打印文件头注释块 (跳过 shebang 与首尾分隔线), 避免帮助文本与代码不同步。
  awk 'NR == 1 { next }
       /^# =+[[:space:]]*$/ { sep++; if (sep == 2) exit; next }
       { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

# --chassis-spin / --gimbal-spin 是"可带数值"的开关: 下一个参数是数字就当转速,
# 否则用各自的默认值（`--gimbal-spin --no-gui` 不会被误吞成转速）。
is_spin_rate() {
  case "${1:-}" in
    ''|*[!0-9.eE+-]*) return 1 ;;
  esac
  awk -v v="$1" 'BEGIN{ exit !(v == v + 0) }'
}

while [ $# -gt 0 ]; do
  case "$1" in
    -w|--world)   WORLD="${2:?--world 需要世界名}"; shift 2 ;;
    --gui)        USE_GUI="true"; shift ;;
    --no-gui)     USE_GUI="false"; shift ;;
    --rviz)       USE_RVIZ="True"; shift ;;
    --no-rviz)    USE_RVIZ="False"; shift ;;
    --params)     PARAMS_FILE="${2:?--params 需要绝对路径}"; shift 2 ;;
    --no-clean)   PREKILL="0"; shift ;;
    --chassis-spin)
                  CHASSIS_SPIN_GIVEN=1
                  if [ $# -ge 2 ] && is_spin_rate "$2"; then
                    CHASSIS_SPIN_WZ="$2"; shift 2
                  else
                    CHASSIS_SPIN_WZ="$CHASSIS_SPIN_DEFAULT"; shift
                  fi ;;
    --no-chassis-spin) CHASSIS_SPIN_WZ="0"; CHASSIS_SPIN_GIVEN=1; shift ;;
    --gimbal-spin)
                  GIMBAL_SPIN_GIVEN=1
                  if [ $# -ge 2 ] && is_spin_rate "$2"; then
                    GIMBAL_SPIN_WZ="$2"; shift 2
                  else
                    GIMBAL_SPIN_WZ="$GIMBAL_SPIN_DEFAULT"; shift
                  fi ;;
    --no-gimbal-spin) GIMBAL_SPIN_WZ="0"; GIMBAL_SPIN_GIVEN=1; shift ;;
    -o|--out)     OUT_DIR="${2:?--out 需要目录}"; shift 2 ;;
    -n|--name)    CLIP_NAME="${2:?--name 需要名字}"; shift 2 ;;
    --min-free-gb) MIN_FREE_GB="${2:?--min-free-gb 需要数值(GB)}"; shift 2 ;;
    --collect)    ACTION="collect"; shift ;;
    --view)       ACTION="view"; VIEW_FILE="${2:?--view 需要 .pcd 路径}"; shift 2 ;;
    -h|--help)    usage; exit 0 ;;
    *)            die "未知参数: $1（--help 看用法）" ;;
  esac
done

# 没给参数 = 不转；给了但不带数值 = 用默认转速。
if [ "$CHASSIS_SPIN_GIVEN" = "1" ]; then
  [ -n "$CHASSIS_SPIN_WZ" ] || CHASSIS_SPIN_WZ="$CHASSIS_SPIN_DEFAULT"
else
  CHASSIS_SPIN_WZ="0"
fi
if [ "$GIMBAL_SPIN_GIVEN" = "1" ]; then
  [ -n "$GIMBAL_SPIN_WZ" ] || GIMBAL_SPIN_WZ="$GIMBAL_SPIN_DEFAULT"
else
  GIMBAL_SPIN_WZ="0"
fi

check_spin_rate() {
  local label="$1" value="$2"
  case "$value" in
    ''|*[!0-9.eE+-]*) die "${label}必须是数字 (rad/s): $value" ;;
  esac
  awk -v v="$value" 'BEGIN{ exit !(v == v + 0) }' || die "${label}必须是数字 (rad/s): $value"
}
check_spin_rate "底盘自转速度" "$CHASSIS_SPIN_WZ"
check_spin_rate "云台自转速度" "$GIMBAL_SPIN_WZ"

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
if awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ exit (v == 0) ? 0 : 1 }'; then
  log "底盘自转  : 关 (默认; 要转加 --chassis-spin [rad/s], 缺省 2.0)"
else
  log "底盘自转  : ${CHASSIS_SPIN_WZ} rad/s (注入 ${CHASSIS_SPIN_TOPIC} -> fake_vel_transform -> mux)"
fi
if awk -v v="$GIMBAL_SPIN_WZ" 'BEGIN{ exit (v == 0) ? 0 : 1 }'; then
  log "云台自转  : 关 (默认; 要转加 --gimbal-spin [rad/s], 缺省 -1.0)"
else
  log "云台自转  : ${GIMBAL_SPIN_WZ} rad/s (仿真器直接施加, 雷达随之转动)"
fi
log ""

# 云台转速必须传给这里：Gazebo 是单独一条命令拉起的，漏了就一直用 launch 默认值。
GAZEBO_CMD="ros2 launch srm27_gazebo_simulator srm_sim.launch.py world:=${WORLD} gui:=${USE_GUI} run_immediately:=true gimbal_spin_rate:=${GIMBAL_SPIN_WZ}"
RVIZ_ARG=""
[ "$USE_RVIZ" = "False" ] && RVIZ_ARG=" rviz:=false"
SLAM_CMD="ros2 launch srm27_nav_bringup sim_slam_launch.py namespace:=${ROBOT_NS} world:=${WORLD} params_file:=\"${PARAMS_FILE}\"${RVIZ_ARG}"

log "[1/4] Gazebo 世界 + 车 + 传感器桥..."
open_term "SRM27 建图-Gazebo" "$GAZEBO_CMD"

if [ "$DRY_RUN" != "1" ]; then
  log "[提示] 等 Gazebo 起来（约 8 s）再拉建图栈..."
  sleep 8
fi

log "[2/4] 建图栈: Point-LIO + loam_interface + sensor_scan_generation + slam_toolbox + RViz..."
open_term "SRM27 建图-SLAM" "$SLAM_CMD"

# ---------- 底盘自转（小陀螺）----------
# 建图栈里**没有 MINCO**：rotation_cmd 只被 MINCO 订阅，往那儿发没人听。所以改从
# fakeyaw 链入口注入恒速 wz —— 与导航下发命令走的是同一条链：
#   cmd_vel_nav_fake -> fake_vel_transform -> cmd_vel_nav_body -> srm_cmd_mux
#     -> cmd_vel_sim -> 速度适配器 -> Gazebo
# 与手动方式等价（同一发布器、同一 PID 文件）:
#   SPIN_CMD_TOPIC=<topic> script/spin_control.sh on <rad/s> | off | status
# 没给 --chassis-spin 时这里什么都不做（默认不转）。
if awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ exit (v == 0) ? 0 : 1 }'; then
  : # 默认不转（未给参数或显式 --no-chassis-spin）
else
  chassis_spin_abs="$(awk -v v="$CHASSIS_SPIN_WZ" 'BEGIN{ printf "%.3f", (v < 0) ? -v : v }')"
  if awk -v v="$chassis_spin_abs" 'BEGIN{ exit !(v > 6.0) }'; then
    log "[提示] 请求 ${CHASSIS_SPIN_WZ} rad/s 超过 mux 的 wz_max（speed_limits.yaml 当前 6.0），实际执行会被限幅。" >&2
  fi
  if [ "$DRY_RUN" = "1" ]; then
    log "--- [dry-run] 底盘自转: 等 ${CHASSIS_SPIN_TOPIC} 有订阅者(fake_vel_transform)后执行"
    log "SRM27_WS_DIR='$WS_DIR' SPIN_CMD_TOPIC='$CHASSIS_SPIN_TOPIC' SPIN_MAX_WZ='$chassis_spin_abs' '$SCRIPT_DIR/../manual/spin_control.sh' on '$CHASSIS_SPIN_WZ'"
  else
    log "[3/4] 底盘自转: 等 fake_vel_transform 上线（${CHASSIS_SPIN_TOPIC} 有订阅者，最长 ${CHASSIS_SPIN_READY_TIMEOUT}s）..."
    spin_deadline=$(( $(date +%s) + CHASSIS_SPIN_READY_TIMEOUT ))
    spin_subs=""
    while [ "$(date +%s)" -lt "$spin_deadline" ]; do
      spin_subs="$(bash -c "source '$WS_DIR/install/setup.bash' >/dev/null 2>&1; timeout 6 ros2 topic info '$CHASSIS_SPIN_TOPIC' 2>/dev/null" 2>/dev/null \
        | sed -n 's/^Subscription count: //p' | head -n 1)" || true
      if [ -n "$spin_subs" ] && [ "$spin_subs" != "0" ]; then
        break
      fi
      sleep 2
    done
    if [ -z "$spin_subs" ] || [ "$spin_subs" = "0" ]; then
      log "[警告] ${CHASSIS_SPIN_READY_TIMEOUT}s 内没等到 ${CHASSIS_SPIN_TOPIC} 的订阅者（fake_vel_transform 还没起来？）" >&2
      log "       仍会启动发布器：等它上线后请求会自动被消费。" >&2
    fi
    if SRM27_WS_DIR="$WS_DIR" SPIN_CMD_TOPIC="$CHASSIS_SPIN_TOPIC" SPIN_MAX_WZ="$chassis_spin_abs" \
       "$SCRIPT_DIR/../manual/spin_control.sh" on "$CHASSIS_SPIN_WZ"; then
      log "底盘自转已下发 ${CHASSIS_SPIN_WZ} rad/s；停转: SPIN_CMD_TOPIC=${CHASSIS_SPIN_TOPIC} $SCRIPT_DIR/../manual/spin_control.sh off"
    else
      log "[警告] spin_control.sh on 失败，底盘不会自转。" >&2
    fi
  fi
fi

log "[4/4] 操作指引"
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
       ls -lh ~/srm_nav_27_data/generated_pcd/point_lio/scans.pcd

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
