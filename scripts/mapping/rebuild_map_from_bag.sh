#!/usr/bin/env bash
# =============================================================
# 从录好的 rosbag 重建「一张图和它的先验点云」—— 图和云来自**同一次回放、同一个前端**。
#
# 为什么需要这个脚本
# =================
# 建图时如果一边跑实时链路（出 pgm），一边事后用 `build_pcd_from_bag.sh` 重建 PCD，两者的
# **里程计不是同一趟**：`build_pcd_from_bag.sh` 强制 point_lio，而实车建图默认用 zlio。
# 两套 LIO 的去畸变/特征/滤波都不同，同一条走廊在两张产物里能差 1 m 量级 —— 202to227
# 就是这个病：pgm 是 zlio 建的、PCD 是 point_lio 回放的，先验包和栅格图在 227 段对不上。
#
# 本脚本把「回放 → 2D 图 → 先验点云」放在同一次回放里完成，前端由 --lio-frontend 指定；
# 想和导航（zlio）完全一致就用默认 zlio。
#
# 它会替你处理的四个坑（都是实测踩过的）
# =====================================
#   1. 回放没有雷达驱动 → 没有 `/livox/lidar/pointcloud`。若 --scan-cloud raw（默认），
#      脚本会自己起 scripts/mapping/custommsg_to_pointcloud2.py 把 CustomMsg 转成
#      PointCloud2，保住 ~20k 点/帧的密度（否则 2D 图会稀成细射线）。
#   2. 放完包 sim time 就停住 → `save_map`/`map_saver_cli` 都收不到 `/map`。所以存图放在
#      **放包途中**（--save-at，默认 时长-20 s），此时 /clock 还在走。
#   3. `pointcloud_to_laserscan` 的 `transform_tolerance: 0.1` 在回放里太紧，TF 跟不上会
#      丢帧（实测 86 次）。脚本在临时参数副本里改成 0.3 —— **不动仓库里的参数文件**。
#   4. 机器上若有残留的 `/slam_toolbox`，`slam_toolbox/save_map` 这个硬编码服务名会被抢走。
#      脚本把节点改名 replay_slam，并用独立 ROS_DOMAIN_ID（默认 91）隔离。
#
# 用法
# ====
#   # 最常用：zlio 前端 + 原 Figure 密度，出图出云
#   ./scripts/mapping/rebuild_map_from_bag.sh \
#       --bag ~/srm_nav_27_data/bags/mapping_bag_227_1113_20261125_1600 --name 227_1113
#
#   # 想用 point_lio（需要和 point_lio 建的那张图配对时才用）
#   ./scripts/mapping/rebuild_map_from_bag.sh --bag <目录> --name foo --lio-frontend point_lio
#
#   # 只重建、不建先验包
#   ./scripts/mapping/rebuild_map_from_bag.sh --bag <目录> --name foo --no-bundle
#
#   # 先看会跑什么，不下场
#   ./scripts/mapping/rebuild_map_from_bag.sh --bag <目录> --name foo -n
#
# 参数
# ====
#       --bag <目录>          rosbag2 目录（含 metadata.yaml + *.db3）
#       --name <名字>         地图名；产出 maps/<名字>/{pgm,yaml} 与 maps/pcd/<名字>.pcd
#       --lio-frontend <zlio|point_lio>   回放用哪个前端（默认 zlio，与导航链一致）
#       --scan-cloud <raw|registered>     2D 栅格吃哪路点云（默认 raw = 稠密云）
#       --rate <倍速>         回放倍速（默认 1.0；**别加速**，快了 LIO 会丢帧）
#       --save-at <秒>        放包第几秒存图（默认 时长-20；至少 10）
#       --bundle              出图出云后接着做 mapframe → level → 建先验包
#       --force               同名地图/pcd/包已存在时允许覆盖
#       --domain <ID>         ROS_DOMAIN_ID（默认 91，避开实车链路）
#   -n, --check               只打印计划，不起任何节点
#   -h, --help                显示本帮助
#
# 产物
# ====
#   maps/<名字>/<名字>.pgm / .yaml                     2D 栅格图（Nav2 用）
#   maps/pcd/<名字>.pcd                                该前端回放的累积点云（原始件，非 map 系）
#   --bundle 时再加：
#   maps/pcd/<名字>_mapframe.pcd / _mapframe_leveled.pcd
#   maps/pcd/map_bundles/<名字>/                        zloc backend 先验地图包
#
# 验收（脚本最后会打印，自己也要跑一遍）
# ====
#   python3 scripts/mapping/verify_map_vs_pcd.py --yaml maps/<名字>/<名字>.yaml \
#       --pcd maps/pcd/<名字>_mapframe_leveled.pcd      # 两臂墙-墙中位都要 <0.2 m
# =============================================================

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

MAP_PARAMS="$WS_DIR/src/bringup/srm27_nav_bringup/config/real/mapping_params.yaml"
MAP_LAUNCH="$WS_DIR/src/bringup/srm27_nav_bringup/launch/real_mapping_launch.py"
BACKEND_CONFIG="$WS_DIR/src/localization/backend/srm27_zlocalization_backend/config/srm27_localization_real.yaml"
SETUP_FILE="$WS_DIR/install/setup.bash"

BAG=""
NAME=""
LIO_FRONTEND="zlio"
SCAN_CLOUD="raw"
RATE="1.0"
SAVE_AT=""
DO_BUNDLE="1"
FORCE="0"
DOMAIN="${SRM27_REPLAY_DOMAIN:-91}"
DRY_RUN="0"

usage() {
  awk 'NR == 1 { next }
       /^# =+[[:space:]]*$/ { sep++; if (sep == 2) exit; next }
       { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

die() { echo "[错误] $*" >&2; exit 1; }
q() { printf '%q' "$1"; }

# ---------- 参数 ----------
while [ $# -gt 0 ]; do
  case "$1" in
    --bag)          BAG="${2:?--bag 需要目录}"; shift 2 ;;
    --name)         NAME="${2:?--name 需要名字}"; shift 2 ;;
    --lio-frontend) LIO_FRONTEND="${2:?}"; shift 2 ;;
    --scan-cloud)   SCAN_CLOUD="${2:?}"; shift 2 ;;
    --rate)         RATE="${2:?}"; shift 2 ;;
    --save-at)      SAVE_AT="${2:?}"; shift 2 ;;
    --bundle)       DO_BUNDLE="1"; shift ;;
    --no-bundle)    DO_BUNDLE="0"; shift ;;
    --force)        FORCE="1"; shift ;;
    --domain)       DOMAIN="${2:?}"; shift 2 ;;
    -n|--check)     DRY_RUN="1"; shift ;;
    -h|--help)      usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 -h 查看用法)" >&2; exit 2 ;;
  esac
done

[ -n "$BAG" ] || die "需要 --bag <rosbag2 目录>"
[ -n "$NAME" ] || die "需要 --name <地图名>"
case "$NAME" in
  *[!A-Za-z0-9_.-]*|'') die "--name 只允许字母/数字/下划线/点/短横线: $NAME" ;;
esac
case "$LIO_FRONTEND" in
  zlio|point_lio) ;;
  *) die "--lio-frontend 只支持 zlio / point_lio, 收到: $LIO_FRONTEND" ;;
esac
case "$SCAN_CLOUD" in
  raw)        SCAN_CLOUD_TOPIC="/livox/lidar/pointcloud" ;;
  registered) SCAN_CLOUD_TOPIC="registered_scan" ;;
  *) die "--scan-cloud 只支持 raw / registered, 收到: $SCAN_CLOUD" ;;
esac
case "$RATE" in
  ''|*[!0-9.]*|0|0.0) die "--rate 必须是正数: $RATE" ;;
esac

BAG="$(readlink -f "$BAG")"
[ -d "$BAG" ] || die "包目录不存在: $BAG"
[ -f "$BAG/metadata.yaml" ] || die "缺 metadata.yaml（包没正常收尾）: $BAG"
ls "$BAG"/*.db3 >/dev/null 2>&1 || die "包里没有 *.db3: $BAG"

MAP_DIR="$WS_DIR/maps/$NAME"
MAP_PREFIX="$MAP_DIR/$NAME"
PCD_OUT="$WS_DIR/maps/pcd/$NAME.pcd"

# 时长/话题：只读 metadata，不碰数据库
BAG_INFO="$(python3 - "$BAG" <<'PY'
import sys, yaml
info = yaml.safe_load(open(sys.argv[1] + '/metadata.yaml', encoding='utf-8'))['rosbag2_bagfile_information']
dur = info['duration']['nanoseconds'] / 1e9
topics = sorted(t['topic_metadata']['name'] for t in info.get('topics_with_message_count', []))
print('%.1f' % dur)
print(' '.join(topics))
PY
)" || die "读不了 metadata.yaml"
BAG_DURATION="$(printf '%s\n' "$BAG_INFO" | head -1)"
BAG_TOPICS="$(printf '%s\n' "$BAG_INFO" | tail -1)"

printf '%s' "$BAG_TOPICS" | grep -q "/livox/lidar" || die "包里没有 /livox/lidar，不能重建"
printf '%s' "$BAG_TOPICS" | grep -q "/livox/imu" || die "包里没有 /livox/imu，不能重建"

if [ -z "$SAVE_AT" ]; then
  SAVE_AT="$(awk -v d="$BAG_DURATION" 'BEGIN{ v = d - 20; if (v < 10) v = 10; printf "%d", v }')"
fi
case "$SAVE_AT" in
  ''|*[!0-9]*) die "--save-at 必须是整数秒: $SAVE_AT" ;;
esac
awk -v s="$SAVE_AT" -v d="$BAG_DURATION" 'BEGIN{ exit !(s < d) }' \
  || die "--save-at ($SAVE_AT s) 必须小于包时长 (${BAG_DURATION} s)"

# ---------- 已有产物 ----------
existing=""
for f in "$MAP_PREFIX.pgm" "$MAP_PREFIX.yaml" "$PCD_OUT"; do
  [ -e "$f" ] && existing="$existing $f"
done
if [ -n "$existing" ] && [ "$FORCE" != "1" ]; then
  echo "[错误] 以下产物已存在，不覆盖:$existing" >&2
  echo "[提示] 换 --name，或确认后加 --force。" >&2
  exit 1
fi

# ---------- 摘要 ----------
cat <<EOF
================ 从 rosbag 重建地图（图 + 云 同一趟） ================
  包目录      : $BAG
  包时长      : ${BAG_DURATION} s  (话题: $BAG_TOPICS)
  地图名      : $NAME
  前端        : $LIO_FRONTEND
  2D 点云     : $SCAN_CLOUD ($SCAN_CLOUD_TOPIC)
  回放倍速    : $RATE
  存图时刻    : 第 ${SAVE_AT} s（放包途中）
  产物        : $MAP_PREFIX.pgm/.yaml
                $PCD_OUT
  先验包      : $([ "$DO_BUNDLE" = 1 ] && echo "会接着建 (mapframe → level → bundle)" || echo "本次不建 (--no-bundle)")
  ROS_DOMAIN  : $DOMAIN（隔离，不影响实车链路）
=====================================================================
EOF

if [ "$DRY_RUN" = "1" ]; then
  echo
  echo "[--check] 仅打印，不起节点、不写文件。内部步骤："
  echo "  1) 生成临时参数副本（transform_tolerance 0.3 + slam 节点改名 replay_slam）"
  echo "  2) ROS_DOMAIN_ID=$DOMAIN, ROS_LOG_DIR=<工作区>/.tmp_roslog"
  [ "$SCAN_CLOUD" = "raw" ] && echo "  3) 起 CustomMsg->PointCloud2 转换节点（保住 ~20k 点/帧）"
  echo "  4) ros2 launch <临时 launch> start_lidar:=False use_sim_time:=True use_rviz:=False"
  echo "     save_pcd:=True lio_frontend:=$LIO_FRONTEND scan_cloud_topic:=$SCAN_CLOUD_TOPIC"
  echo "  5) ros2 bag play $BAG --clock --topics /livox/lidar /livox/imu --rate $RATE"
  echo "  6) 第 ${SAVE_AT} s: ros2 service call /slam_toolbox/save_map ..."
  echo "  7) 放完 → SIGINT → 等 PCD 落盘"
  [ "$DO_BUNDLE" = "1" ] && echo "  8) pcd_to_mapframe → level_map_pcd → build_zloc_map_bundle"
  exit 0
fi

[ -f "$SETUP_FILE" ] || die "找不到 $SETUP_FILE（先 colcon build）"
[ -f "$MAP_PARAMS" ] || die "找不到建图参数: $MAP_PARAMS"
[ -f "$MAP_LAUNCH" ] || die "找不到建图 launch: $MAP_LAUNCH"

# ---------- 临时参数/launch ----------
WORK="$(mktemp -d -t srm27_replay.XXXXXX)" || die "mktemp 失败"
cleanup() { rm -rf "$WORK"; }
trap cleanup EXIT

python3 - "$MAP_PARAMS" "$MAP_LAUNCH" "$WORK" <<'PY' || die "生成临时配置失败"
import pathlib, sys
params, launch, work = (pathlib.Path(p) for p in sys.argv[1:4])
text = params.read_text(encoding='utf-8')
if 'transform_tolerance: 0.1' in text:
    text = text.replace('transform_tolerance: 0.1', 'transform_tolerance: 0.3')
if '\nslam_toolbox:\n' in text:
    text = text.replace('\nslam_toolbox:\n', '\nreplay_slam:\n')
(work / 'params.yaml').write_text(text, encoding='utf-8')
code = launch.read_text(encoding='utf-8')
old = 'executable="sync_slam_toolbox_node",\n            name="slam_toolbox",'
new = 'executable="sync_slam_toolbox_node",\n            name="replay_slam",'
if old not in code:
    raise SystemExit('launch 里没找到 slam_toolbox 节点定义（上游改过？）')
(work / 'launch.py').write_text(code.replace(old, new), encoding='utf-8')
PY

PARAMS_COPY="$WORK/params.yaml"
LAUNCH_COPY="$WORK/launch.py"
LOG="$WS_DIR/.diag_tmp/rebuild_${NAME}_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$WS_DIR/.diag_tmp"
: > "$LOG"

# ---------- 环境 ----------
set +u
# shellcheck disable=SC1091
source "$WS_DIR/scripts/setup/env_srm27.sh" > /dev/null 2>&1
set -u
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOG_DIR="$WS_DIR/.tmp_roslog"
mkdir -p "$ROS_LOG_DIR"
ros2 daemon stop > /dev/null 2>&1

mkdir -p "$MAP_DIR" "$(dirname "$PCD_OUT")"
[ "$FORCE" = "1" ] && rm -f "$MAP_PREFIX.pgm" "$MAP_PREFIX.yaml" "$PCD_OUT"

# point_lio 的落盘目录由环境变量给（它的文件名固定 scans.pcd）
PCD_TMP_DIR=""
if [ "$LIO_FRONTEND" = "point_lio" ]; then
  PCD_TMP_DIR="$(dirname "$PCD_OUT")"
  export SRM27_PCD_DIR="$PCD_TMP_DIR"
  rm -f "$PCD_TMP_DIR/scans.pcd"
fi

CONV_PID=""
LAUNCH_PID=""
PLAY_PID=""
MAP_DONE="0"

shutdown_all() {
  [ -n "$PLAY_PID" ] && kill -INT "$PLAY_PID" 2>/dev/null
  [ -n "$CONV_PID" ] && kill -INT "$CONV_PID" 2>/dev/null
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  sleep 2
  pkill -INT -f "[s]ync_slam_toolbox_node" 2>/dev/null
  pkill -INT -f "[p]ointlio_mapping" 2>/dev/null
  pkill -INT -f "[z]lio_node" 2>/dev/null
  sleep 3
}
trap 'shutdown_all; cleanup' EXIT

echo "[1/8] 起 CustomMsg -> PointCloud2 转换（$SCAN_CLOUD 模式）"
if [ "$SCAN_CLOUD" = "raw" ]; then
  python3 "$SCRIPT_DIR/custommsg_to_pointcloud2.py" --ros-args \
    -p in_topic:=/livox/lidar -p out_topic:="$SCAN_CLOUD_TOPIC" \
    > "$WORK/converter.log" 2>&1 &
  CONV_PID=$!
else
  echo "      registered 模式不需要转换节点"
fi

echo "[2/8] 起建图链路（$LIO_FRONTEND）"
ros2 launch "$LAUNCH_COPY" \
  start_lidar:=False use_sim_time:=True use_rviz:=False save_pcd:=True \
  lio_frontend:="$LIO_FRONTEND" params_file:="$PARAMS_COPY" \
  scan_cloud_topic:="$SCAN_CLOUD_TOPIC" pcd_save_path:="$PCD_OUT" \
  >> "$LOG" 2>&1 &
LAUNCH_PID=$!

echo "[3/8] 等链路就绪"
ready=0
for i in $(seq 1 45); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    tail -20 "$LOG" >&2
    die "链路启动就退出（日志: $LOG）"
  fi
  if ros2 node list --no-daemon 2>/dev/null | grep -qx "/replay_slam" \
     && ros2 service list 2>/dev/null | grep -qx "/slam_toolbox/save_map" \
     && ros2 topic info /livox/lidar 2>/dev/null | grep -q "Subscription count: [1-9]"; then
    ready=1
    break
  fi
  sleep 2
done
[ "$ready" = "1" ] || { tail -20 "$LOG" >&2; die "90 s 内链路没就绪（日志: $LOG）"; }
sleep 3

if [ "$SCAN_CLOUD" = "raw" ]; then
  ros2 topic info "$SCAN_CLOUD_TOPIC" 2>/dev/null | grep -q "Subscription count: [1-9]" \
    || echo "[警告] $SCAN_CLOUD_TOPIC 暂时没有订阅者，检查 pointcloud_to_laserscan 是否起来了"
fi

echo "[4/8] 放包（${BAG_DURATION} s @ ${RATE}x）"
ros2 bag play "$BAG" --clock --topics /livox/lidar /livox/imu --rate "$RATE" >> "$LOG" 2>&1 &
PLAY_PID=$!

echo "[5/8] 等到第 ${SAVE_AT} s 存图"
sleep "$SAVE_AT"
if kill -0 "$PLAY_PID" 2>/dev/null; then
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '$MAP_PREFIX'}}" >> "$LOG" 2>&1
  if [ -s "$MAP_PREFIX.pgm" ]; then
    MAP_DONE="1"
    echo "      save_map 出图: $MAP_PREFIX.pgm"
  else
    echo "      save_map 没出图，用 map_saver_cli 兜底"
    timeout 40 ros2 run nav2_map_server map_saver_cli -f "$MAP_PREFIX" >> "$LOG" 2>&1
    [ -s "$MAP_PREFIX.pgm" ] && MAP_DONE="1" && echo "      map_saver_cli 出图: $MAP_PREFIX.pgm"
  fi
else
  echo "[警告] 放包已经结束，来不及中途存图；只能试 map_saver_cli（可能拿不到 /map）"
  timeout 40 ros2 run nav2_map_server map_saver_cli -f "$MAP_PREFIX" >> "$LOG" 2>&1
  [ -s "$MAP_PREFIX.pgm" ] && MAP_DONE="1"
fi

echo "[6/8] 等放包结束"
wait "$PLAY_PID" 2>/dev/null
sleep 5

echo "[7/8] 停链路并等点云落盘"
[ -n "$CONV_PID" ] && kill -INT "$CONV_PID" 2>/dev/null
kill -INT "$LAUNCH_PID" 2>/dev/null
waited=0
while [ "$waited" -lt 180 ]; do
  if [ "$LIO_FRONTEND" = "point_lio" ]; then
    [ -s "$PCD_TMP_DIR/scans.pcd" ] && break
  else
    [ -s "$PCD_OUT" ] && break
  fi
  sleep 3
  waited=$((waited + 3))
done
sleep 2
if [ "$LIO_FRONTEND" = "point_lio" ] && [ -s "$PCD_TMP_DIR/scans.pcd" ]; then
  mv -f "$PCD_TMP_DIR/scans.pcd" "$PCD_OUT"
fi
kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 2
PLAY_PID=""

echo "[8/8] 结果"
rc=0
[ -s "$MAP_PREFIX.pgm" ] && [ -s "$MAP_PREFIX.yaml" ] || { echo "[错误] 没出 2D 图" >&2; rc=1; }
[ -s "$PCD_OUT" ] || { echo "[错误] 没出点云: $PCD_OUT（退出方式不对？必须 SIGINT 正常退出）" >&2; rc=1; }
if [ "$rc" != "0" ]; then
  echo "[提示] 完整日志: $LOG"
  exit "$rc"
fi
ls -la "$MAP_PREFIX.pgm" "$MAP_PREFIX.yaml" "$PCD_OUT"

if [ "$DO_BUNDLE" = "1" ] && [ -f "$BACKEND_CONFIG" ]; then
  echo
  echo "[附加] 转地图系 → 摆正 → 建先验包"
  python3 "$WS_DIR/scripts/mapping/pcd_to_mapframe.py" \
    --pcd "$PCD_OUT" --map "$MAP_PREFIX.yaml" || echo "[警告] pcd_to_mapframe 失败，先看它打印的残差"
  if [ -s "$WS_DIR/maps/pcd/${NAME}_mapframe.pcd" ]; then
    python3 "$WS_DIR/scripts/mapping/level_map_pcd.py" \
      --pcd "$WS_DIR/maps/pcd/${NAME}_mapframe.pcd" || echo "[警告] level_map_pcd 失败"
    if [ -s "$WS_DIR/maps/pcd/${NAME}_mapframe_leveled.pcd" ]; then
      "$WS_DIR/scripts/mapping/build_zloc_map_bundle.sh" --name "$NAME" \
        --pcd "$WS_DIR/maps/pcd/${NAME}_mapframe_leveled.pcd" \
        --config "$BACKEND_CONFIG" --sensor-z 0.2966 --force \
        || echo "[警告] 建先验包失败（看上面输出）"
    fi
  fi
fi

cat <<EOF

重建完成（图和云来自同一次回放、前端 $LIO_FRONTEND）。
  图    : $MAP_PREFIX.pgm / .yaml
  点云  : $PCD_OUT
  日志  : $LOG

下一步验收（两臂墙-墙中位都要 <0.2 m）：
  python3 scripts/mapping/verify_map_vs_pcd.py --yaml $MAP_PREFIX.yaml \\
      --pcd $WS_DIR/maps/pcd/${NAME}_mapframe_leveled.pcd

起导航（先验包按 -m 名字自动找）：
  ./scripts/bringup/start_real_nav.sh -m $NAME --lio --lio-frontend zlio \\
      --zloc-backend --zloc-prior
EOF
