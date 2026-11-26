#!/usr/bin/env bash
# =============================================================
# 录一次导航「黑盒」：路径四层 + 速度三层 + TF + SFC 约束
#
# 为什么是这几路
# ==============
# 判断"拐弯不对"要能分层归因，缺一路就得猜：
#
#   /plan                        全局规划（YAstar / ThetaStar），map
#   /plan_smoothed               MinimumSnap 平滑后，map
#   FollowPath/planning_input_path  控制器真正喂给 MINCO 的参考（裁剪 3 m + 净空预推），odom
#   FollowPath/minco_trajectory  MINCO 优化出来的轨迹（RViz 蓝粗线），odom
#   FollowPath/mpc_prediction    MPC 未来 0.6 s 预测，odom
#   FollowPath/diagnostics       净空/耗时等诊断量（DiagnosticArray）
#   /odometry                    实际走的轨迹（真值/定位）
#   cmd_vel_nav_fake / nav_body / chassis
#                                速度三层：导航出口 → fakeyaw 旋转后 → 底盘实发
#   /tf /tf_static               复算位姿/偏差用（注意本仓库 TF 是命名空间内的）
#   smoother_server/sfc_corridor SFC 方块（黄=初始自由方块，橙=真正当约束用的）
#   /clock                       仿真时间基准
#
# 体积：实测 21 s = 4.5 MiB / 24300 条（约 0.2 MB/s），其中 /clock 占 20400 条。
#       3 分钟约 40 MB。想再省就删掉 /clock 和 mpc_prediction 两路。
#
# 用法
# ====
#   # 另开一个终端（顺序无所谓，先录后跑也行）
#   ./scripts/bringup/record_nav_bag.sh
#   ./scripts/bringup/record_nav_bag.sh --name corner_test_1
#   ./scripts/bringup/record_nav_bag.sh --list        # 只打印将录的话题和当前是否在线
#
# 注意：必须与导航进程同一个 ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY。
#   run.sh 默认 ROS_DOMAIN_ID=87、ROS_LOCALHOST_ONLY=1；
#   本脚本默认沿用 87，可用 ROS_DOMAIN_ID=xx 覆盖，但两边必须一致。
#
# 录完：
#   ros2 bag info <目录>                             # 帧数/时长
#   python3 debug_tools/capture_nav_paths.py         # 或直接把它交给分析脚本
# =============================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
NS="${SRM27_NS:-red_standard_robot1}"
NAME="nav_$(date +%m%d_%H%M%S)"
DEST="${SRM27_BAG_DIR:-$HOME/srm_nav_27_data/bags}"
LIST_ONLY=0

while [ $# -gt 0 ]; do
  case "$1" in
    --name) NAME="${2:?--name 需要名字}"; shift 2 ;;
    --dir)  DEST="${2:?--dir 需要目录}"; shift 2 ;;
    --ns)   NS="${2:?--ns 需要命名空间}"; shift 2 ;;
    --list) LIST_ONLY=1; shift ;;
    -h|--help) sed -n '1,40p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[错误] 未知参数: $1" >&2; exit 2 ;;
  esac
done

TOPICS=(
  "/${NS}/plan"
  "/${NS}/plan_smoothed"
  "/${NS}/FollowPath/planning_input_path"
  "/${NS}/FollowPath/minco_trajectory"
  "/${NS}/FollowPath/mpc_prediction"
  "/${NS}/FollowPath/diagnostics"
  # MINCO 迁到规划器后（2026-11-26）的新三路：规划器发布的轨迹、可视化路径、诊断。
  # 控制器还在 internal 模式时这几路也有（只是没被控制器消费），A/B 对照要用。
  "/${NS}/planning/minco_trajectory"
  "/${NS}/planning/minco_path"
  "/${NS}/planning/diagnostics"
  "/${NS}/odometry"
  "/${NS}/cmd_vel_nav_fake"
  "/${NS}/cmd_vel_nav_body"
  "/${NS}/cmd_vel_chassis"
  "/${NS}/smoother_server/sfc_corridor"
  "/${NS}/tf"
  "/${NS}/tf_static"
  "/clock"
)

# 录制端必须是同一套环境：domain / localhost-only / 消息定义。
# ⚠ ROS_LOCALHOST_ONLY 用**专属变量**控制，不继承环境变量：run.sh 里固定是 1，
#   如果这里不小心继承了 0（.bashrc/别的脚本留下的），DDS 互相发现不了，
#   录出来的就是一个 0 消息的空包 —— 这个坑第一次就踩到了（2026-11-26）。
export ROS_DOMAIN_ID="${SRM27_DOMAIN_ID:-${ROS_DOMAIN_ID:-87}}"
export ROS_LOCALHOST_ONLY="${SRM27_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS_DIR/.tmp_roslog}"
mkdir -p "$ROS_LOG_DIR"

set +e
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
set -e

if [ "$LIST_ONLY" = "1" ]; then
  echo "[话题] 纳秒空间 $NS，domain=$ROS_DOMAIN_ID："
  online="$(ros2 topic list 2>/dev/null || true)"
  for t in "${TOPICS[@]}"; do
    if printf '%s\n' "$online" | grep -qx "$t"; then
      echo "  ✔ $t"
    else
      echo "  ✘ $t  （当前不在线：导航没起或 domain/namespace 不一致）"
    fi
  done
  exit 0
fi

mkdir -p "$DEST"
OUT="$DEST/$NAME"

echo "[录包] 命名空间 $NS   domain=$ROS_DOMAIN_ID   localhost_only=$ROS_LOCALHOST_ONLY"
echo "[录包] 输出     $OUT"
echo "[录包] 话题     ${#TOPICS[@]} 路"
printf '         %s\n' "${TOPICS[@]}"
echo "[录包] Ctrl-C 结束（结束前会写 metadata.yaml，别用 kill -9）"

# 起录前先确认真的能看到导航进程：DDS 配错时 rosbag2 会安静地录出一个空包。
probe="/${NS}/odometry"
ready=0
for _ in $(seq 1 20); do
  if ros2 topic list 2>/dev/null | grep -qx "$probe"; then ready=1; break; fi
  sleep 0.5
done
if [ "$ready" != "1" ]; then
  cat >&2 <<EOF

[错误] 10 s 内没发现 $probe，导航进程没起来或 DDS 配置不一致。
       当前: ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY
       排查:
         1) 导航起了吗（run.sh 的那个终端还活着吗）
         2) 两边域名要一致: run.sh 默认 87 → 这里 ROS_DOMAIN_ID=87
         3) ROS_LOCALHOST_ONLY 两边要一致，run.sh 是 1 → 这里默认也是 1；
            若你的导航用 0 起，这里要 SRM27_LOCALHOST_ONLY=0 加 SRM27_DOMAIN_ID=...
         4) ./scripts/bringup/record_nav_bag.sh --list 看哪几路在线
EOF
  exit 1
fi

exec ros2 bag record -o "$OUT" "${TOPICS[@]}"
