#!/usr/bin/env bash
# =============================================================
# 录一份「定位诊断包」：只收定位/云台链路上判断需要的几路，录完自动出结论。
#
# 为什么不用 record_nav_bag.sh
# ============================
# 那个脚本是给**仿真/命名空间**用的：话题全在 /red_standard_robot1/ 下，收的是
# plan / minco_trajectory / cmd_vel 这些"规划控制"层。实车 `start_real_nav.sh` 起的
# 是不带命名空间的一套，话题名也不一样（/tf、/odometry、/backend_node/...），
# 拿那个脚本录实车会一路 ✘，甚至录出一个空包。
#
# 本脚本对应的是**定位问题**：map→odom 到底转不转、是冻结反算还是校正被吃、
# 点云有没有跟着云台转。这几件事分别需要：
#
#   /tf /tf_static                   map→odom、odom→body（判据的甲乙两列，必须有）
#   /backend_node/localization_state 状态机 + reason（冻结/恢复/冲突的原文，必须有）
#   /backend_node/localization_pose  后端 200 Hz 输出位姿（判"车真的被转走"的丙列）
#   /registered_scan                 世界点云（判"点云跟着云台转"是否为真 bug）
#   /lidar_odometry /odometry        前端原始位姿与桥接输出（对账用）
#   /zlio_bridge/events /matches     桥接丢帧、单帧跳变、匹配点数
#   /livox/imu                       云台角速度/冲击（对时间轴）
#   /map                             先验栅格图（后处理对齐用，latched，可能只有 1 帧）
#   /joint_states /gimbal_joint_angle /chassis_speed   有就收，用于把转速对上
#
# 体积：实车实测约 0.85~0.9 MB/s（≈50 MB/分钟）。只 /registered_scan 是大头。
#
# 用法
# ====
#   ./scripts/bringup/record_zloc_bag.sh                  # 录到 ~/srm_nav_27_data/bags/real_zloc_<时间>
#   ./scripts/bringup/record_zloc_bag.sh --name spin_1
#   ./scripts/bringup/record_zloc_bag.sh --list           # 只打印话题在线情况，不录
#   ./scripts/bringup/record_zloc_bag.sh --no-analyze     # 录完不自动分析
#   ./scripts/bringup/record_zloc_bag.sh --dir /data/bags # 换输出目录
#
# 顺序（很重要）
# ==============
#   1) 先起导航：./scripts/bringup/start_real_nav.sh -m 227to219_1125 \
#                   --lio --lio-frontend zlio --zloc-backend --zloc-prior --yastar-planner
#      （注意：诊断"旋转/跳变"必须用 --zloc-prior，不能带 --zloc-prior-once，
#        后者 map→odom 会被锁死，永远不会动，录了也白录）
#   2) 等定位进 NORMAL（看 state），**再**开本脚本开始录
#   3) 录着的时候再动车/转云台——顺序反了，现象就发生在录像之前
#   4) 结束时按一次 **Ctrl-C**。必须让它自己退：被 kill -9 的包不会有
#      metadata.yaml，ros2 bag info 都打不开（2026-11-27 01:26 那个包就是这样废的）
#
# 录完自动做什么
# ==============
#   - diag_map_odom_spin.py --bag   三列判据：0 稳定 / 1 校正被吃 / 2 冻结反算 / 4 锁存
#   - plot_bag_trajectory.py --bag  map 系与 odom 系轨迹图 + 逐点 CSV
#   两者输出都 tee 到 <包>/analysis.txt，图存进 user_test_log/。
# =============================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
NAME="real_zloc_$(date +%m%d_%H%M%S)"
DEST="${SRM27_BAG_DIR:-$HOME/srm_nav_27_data/bags}"
LIST_ONLY=0
ANALYZE=1

while [ $# -gt 0 ]; do
  case "$1" in
    --name)        NAME="${2:?--name 需要名字}"; shift 2 ;;
    --dir)         DEST="${2:?--dir 需要目录}"; shift 2 ;;
    --list)        LIST_ONLY=1; shift ;;
    --no-analyze)  ANALYZE=0; shift ;;
    -h|--help)     sed -n '1,48p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[错误] 未知参数: $1" >&2; exit 2 ;;
  esac
done

# 必录项：缺任何一路都判不了定位问题，直接报错退出。
REQUIRED=(
  /tf
  /backend_node/localization_state
)
# 可选：在线才录。前几路缺了会显著降低可诊断性，只给警告。
WANTED=(
  /tf_static
  /backend_node/localization_pose
  /registered_scan
  /lidar_odometry
  /odometry
  /zlio_bridge/events
  /zlio_bridge/matches
  /livox/imu
  /map
  /joint_states
  /gimbal_joint_angle
  /chassis_speed
)

# 实车的 start_real_nav.sh 不设 ROS_DOMAIN_ID（默认 0），仿真才是 87。
# 这里沿用当前环境，不硬写，避免和导航进程发现不了、安静地录出空包。
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS_DIR/.tmp_roslog}"
mkdir -p "$ROS_LOG_DIR"

set +e
source /opt/ros/humble/setup.bash 2>/dev/null
source "$WS_DIR/install/setup.bash" 2>/dev/null
set -e

online="$(ros2 topic list 2>/dev/null || true)"

miss_req=()
for t in "${REQUIRED[@]}"; do
  printf '%s\n' "$online" | grep -qx "$t" || miss_req+=("$t")
done

if [ "$LIST_ONLY" = "1" ]; then
  echo "[话题] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-未设}"
  for t in "${REQUIRED[@]}"; do
    printf '%s\n' "$online" | grep -qx "$t" && echo "  ✔ $t  (必录)" || echo "  ✘ $t  (必录，缺了不能录)"
  done
  for t in "${WANTED[@]}"; do
    printf '%s\n' "$online" | grep -qx "$t" && echo "  ✔ $t" || echo "  ✘ $t"
  done
  exit 0
fi

if [ "${#miss_req[@]}" -gt 0 ]; then
  cat >&2 <<EOF

[错误] 缺少必录话题：${miss_req[*]}
       这说明导航/定位没起来，或者录包终端和导航不是同一个 DDS 环境。

       排查：
         1) 导航那个终端还活着吗？定位进 NORMAL 了吗
            ros2 topic echo /backend_node/localization_state --once
         2) 两边 domain / localhost_only 要一致：
            当前 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-未设}
            实车 start_real_nav.sh 不设 domain（=0）；仿真 run.sh 是 87
         3) 先看清单：./scripts/bringup/record_zloc_bag.sh --list
EOF
  exit 1
fi

# 可选话题只收在线的：给不存在的名字让 rosbag2 建空订阅没有意义。
TOPICS=("${REQUIRED[@]}")
for t in "${WANTED[@]}"; do
  if printf '%s\n' "$online" | grep -qx "$t"; then
    TOPICS+=("$t")
  else
    echo "[提示] 跳过不在线的话题: $t"
  fi
done

mkdir -p "$DEST"
OUT="$DEST/$NAME"
if [ -e "$OUT" ]; then
  echo "[错误] 输出目录已存在: $OUT" >&2
  exit 1
fi

echo "================= 录定位诊断包 ================="
echo "  输出      : $OUT"
echo "  domain    : ${ROS_DOMAIN_ID:-0}  localhost_only=${ROS_LOCALHOST_ONLY:-未设}"
echo "  话题      : ${#TOPICS[@]} 路"
printf '              %s\n' "${TOPICS[@]}"
echo "  体积估算  : 约 0.9 MB/s（50 MB/分钟）；30 s 约 25 MB"
echo "  ⚠ 结束请按一次 Ctrl-C（写 metadata.yaml）；kill -9 的包打不开"
echo "==============================================="

trap '' INT
# rosbag2 在 Ctrl-C 时一般返回 0；个别版本返回非 0。别让 set -e 把后面的
# 自动分析打断——成败一律以 metadata.yaml 是否存在为准。
ros2 bag record -o "$OUT" --max-cache-size 100000000 "${TOPICS[@]}" || true
trap - INT

# 走到这里说明 ros2 bag 自己正常退出了（Ctrl-C）。
if [ ! -f "$OUT/metadata.yaml" ]; then
  cat >&2 <<EOF

[错误] 没有生成 metadata.yaml —— 这次录制不算成功。
       多半是被 kill -9 / 断电 / 进程被别的脚本 pkill 掉了。
       .db3 里仍然有数据，可以用 sqlite3 直接读，但 ros2 bag 系工具和本仓库的
       离线分析脚本都打不开。录的时候别再 pkill ros2 bag。
EOF
  exit 1
fi

size="$(du -h "$OUT" 2>/dev/null | cut -f1)"
echo "[完成] $OUT  ($size)"
ros2 bag info "$OUT" 2>/dev/null | sed -n '1,8p' || true

if [ "$ANALYZE" = "1" ]; then
  echo
  echo "================= 自动分析 ================="
  cd "$WS_DIR"
  {
    echo "### diag_map_odom_spin"
    ./debug_tools/diag_map_odom_spin.py --bag "$OUT" 2>&1 | grep -v '^\[INFO\]'
    echo
    echo "### plot_bag_trajectory"
    MPLCONFIGDIR=/tmp/mplcfg ./debug_tools/plot_bag_trajectory.py --bag "$OUT" 2>&1 | grep -v '^\[INFO\]'
  } | tee "$OUT/analysis.txt"
fi

cat <<EOF

下一步：
  · 判据结论在 $OUT/analysis.txt
  · 轨迹图与逐点 CSV 在 $WS_DIR/user_test_log/traj_$(basename "$OUT")_*.png / .csv
  · 想要更细的时间线：ros2 topic echo /backend_node/localization_state
    （reason 里会写 observation model invalid / conflicts n/7 / recovered via / over correction budget）
EOF
