#!/usr/bin/env bash
# =============================================================
# SRM 实车建图同步录包 (供 start_real_slam.sh --record-bag 调用, 也可单独跑)
#
# 为什么建图要顺手录包:
#   slam_toolbox 存出来的 .posegraph / .data 是二维位姿图 (karto 的
#   LocalizedRangeScan + Pose2), 里面没有三维点云 —— 事后"把地图转成 PCD"
#   这条路不存在。Point-LIO 的 --save-pcd 只在**这一趟正常退出**时落一次盘
#   (src/srm27_navigation/point_lio/PCD/scans.pcd), 参数不对、图建跑了、
#   进程被 kill -9, 都只能重跑。录了原始数据才能事后反复重建。
#
# 最小可重建集 (默认主题):
#   /livox/lidar        livox_ros_driver2/msg/CustomMsg   含逐点 offset_time, 重建点云的关键
#   /livox/imu          sensor_msgs/msg/Imu
#   /tf /tf_static      体量很小, 留作事后核对
#
# --derived 追加 (省掉重跑 LIO, 但体积翻倍):
#   /cloud_registered /aft_mapped_to_init /scan /map
#
# 体量参考 (Mid-360 满速约 20 万点/秒, publish_freq 10 Hz):
#   CustomMsg 每点约 20 字节 → 约 400 KB/帧 → **约 240 MB/分钟 (≈14 GB/小时)**
#   默认 4096 MB 分卷; 盘不够时脚本直接拒绝启动 (--min-free-gb)。
#
# 录完之后怎么用 (离线重建 PCD, 不需要真车):
#   # 1) 只放原始话题, 别放 /tf —— 新起的链路自己会发 odom->base_link 与
#   #    base_link->livox_frame, 重放旧的 /tf 会和它抢同一条 TF。
#   ros2 bag play <bag 目录> --clock --topics /livox/lidar /livox/imu
#   # 2) 另开终端: 不起雷达驱动, 用仿真时间, 开落盘
#   ros2 launch srm27_nav_bringup real_mapping_launch.py \
#       start_lidar:=False use_sim_time:=True save_pcd:=True
#   # 3) 放完 Ctrl+C 正常退出 → src/srm27_navigation/point_lio/PCD/scans.pcd
#   ⚠ 这份 PCD 在 Point-LIO 的 camera_init 系, 不是 map 系; 拿给 --reloc 用之前
#     还要按 nav2_params_srm.yaml 里 small_gicp_relocalization 那段注释处理。
#
# 用法:
#   ./script/record_mapping_bag.sh                          # 默认主题集, 目录名带时间戳
#   ./script/record_mapping_bag.sh --map-name 227_1010      # 目录名带上地图名, 便于配对
#   ./script/record_mapping_bag.sh --derived                # 追加派生话题 (体积翻倍)
#   ./script/record_mapping_bag.sh --out-dir /abs/bag_dir   # 直接指定输出目录
#   ./script/record_mapping_bag.sh --topics "/livox/lidar /livox/imu"   # 完全自定义
#   ./script/record_mapping_bag.sh -n                       # 只打印将执行的命令
#
# 参数:
#       --map-name <名字>       输出目录名带上地图名 (建议与稍后 --save 的名字一致)
#       --out-dir <绝对路径>    直接指定输出目录 (优先于 --map-name)
#       --base-dir <绝对路径>   输出根目录, 默认 <工作空间>/log_diag (与 real_bag_* 同处)
#       --derived              额外录 /cloud_registered /aft_mapped_to_init /scan /map
#       --topics "<a b c>"     完全自定义主题表 (给了就覆盖默认集, 空格分隔)
#       --wait <秒>            启动前最多等 /livox/lidar 出现的秒数 (默认 60; 0 = 不等)
#       --max-size-mb <MB>     单文件分卷阈值 (默认 4096; 0 = 不分卷)
#       --min-free-gb <GB>     可用空间低于该值就拒绝启动 (默认 5)
#       --use-sim-time         给 ros2 bag record 传 --use-sim-time
#   -n, --check                只打印将执行的命令, 不启动录制
#   -h, --help                 显示本帮助
#
# 环境变量 (同名参数均可由环境变量给默认值):
#   BAG_MAP_NAME BAG_OUT_DIR BAG_BASE_DIR BAG_DERIVED BAG_TOPICS BAG_WAIT
#   BAG_MAX_SIZE_MB BAG_MIN_FREE_GB BAG_USE_SIM_TIME DRY_RUN SRM27_WS_DIR
#
# 说明:
#   - 必须在建图链路(至少雷达驱动)起来之后再录; 脚本自己会等 /livox/lidar, 但如果
#     雷达一直没起来, 等满 --wait 秒会**照样开始录**并给出警告 (不会静默丢数据)。
#   - 结束方式只有一种是对的: Ctrl+C 或 SIGINT。rosbag2 是在退出时才写
#     metadata.yaml 的, kill -9 会让这次录制整体作废 (脚本退出时会检查这个文件)。
#   - 录 `/livox/lidar` 带上了 livox_ros_driver2 的**自定义消息类型**, 回放前要
#     source 本工作空间的 install/setup.bash, 否则 ros2 bag 认不出类型。
# =============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"

# ---------- 可配置项 ----------
BAG_MAP_NAME="${BAG_MAP_NAME:-}"
BAG_OUT_DIR="${BAG_OUT_DIR:-}"
BAG_BASE_DIR="${BAG_BASE_DIR:-$WS_DIR/log_diag}"
BAG_DERIVED="${BAG_DERIVED:-0}"
BAG_TOPICS="${BAG_TOPICS:-}"
BAG_WAIT="${BAG_WAIT:-60}"
BAG_MAX_SIZE_MB="${BAG_MAX_SIZE_MB:-4096}"
BAG_MIN_FREE_GB="${BAG_MIN_FREE_GB:-5}"
BAG_USE_SIM_TIME="${BAG_USE_SIM_TIME:-0}"
DRY_RUN="${DRY_RUN:-0}"

# 最小可重建集: 有了这些, 事后可以完整重跑 Point-LIO 重建 PCD。
RAW_TOPICS=(/livox/lidar /livox/imu /tf /tf_static)
# 派生话题: 录了就不用重跑 LIO, 但体积再翻一倍左右。
DERIVED_TOPICS=(/cloud_registered /aft_mapped_to_init /scan /map)

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

q() { printf '%q' "$1"; }

# 在 source 过工作空间环境的子 shell 里执行命令。
ros2_run() {
  (
    set +u
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash" >/dev/null 2>&1
    "$@"
  )
}

# ---------- 命令行参数 ----------
while [ $# -gt 0 ]; do
  case "$1" in
    --map-name)      BAG_MAP_NAME="${2:?--map-name 需要名字}"; shift 2 ;;
    --out-dir)       BAG_OUT_DIR="${2:?--out-dir 需要绝对路径}"; shift 2 ;;
    --base-dir)      BAG_BASE_DIR="${2:?--base-dir 需要绝对路径}"; shift 2 ;;
    --derived)       BAG_DERIVED=1; shift ;;
    --topics)        BAG_TOPICS="${2:?--topics 需要主题表}"; shift 2 ;;
    --wait)          BAG_WAIT="${2:?--wait 需要秒数}"; shift 2 ;;
    --max-size-mb)   BAG_MAX_SIZE_MB="${2:?--max-size-mb 需要 MB}"; shift 2 ;;
    --min-free-gb)   BAG_MIN_FREE_GB="${2:?--min-free-gb 需要 GB}"; shift 2 ;;
    --use-sim-time)  BAG_USE_SIM_TIME=1; shift ;;
    -n|--check)      DRY_RUN=1; shift ;;
    -h|--help)       usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 -h 查看用法)" >&2; exit 2 ;;
  esac
done

for pair in "BAG_WAIT:$BAG_WAIT" "BAG_MAX_SIZE_MB:$BAG_MAX_SIZE_MB"; do
  case "${pair#*:}" in
    ''|*[!0-9]*) echo "[错误] ${pair%%:*} 必须是非负整数, 收到: ${pair#*:}" >&2; exit 2 ;;
  esac
done
case "$BAG_MIN_FREE_GB" in
  ''|*[!0-9.]*) echo "[错误] BAG_MIN_FREE_GB 必须是非负数字, 收到: $BAG_MIN_FREE_GB" >&2; exit 2 ;;
esac

# ---------- 主题表 ----------
if [ -n "$BAG_TOPICS" ]; then
  read -r -a TOPICS <<<"$BAG_TOPICS"
else
  TOPICS=("${RAW_TOPICS[@]}")
  if is_true "$BAG_DERIVED"; then
    TOPICS+=("${DERIVED_TOPICS[@]}")
  fi
fi
if [ "${#TOPICS[@]}" = "0" ]; then
  echo "[错误] 主题表为空。" >&2
  exit 2
fi

# ---------- 输出目录 ----------
# 默认 <工作空间>/log_diag/mapping_bag_<地图名|时间戳>: 与既有的
# log_diag/real_bag_<日期>_<时间> 同一层, 且 log_diag/ 整个已被 .gitignore 忽略,
# 几 GB 的包不会污染 git 状态。
resolve_out_dir() {
  if [ -n "$BAG_OUT_DIR" ]; then
    printf '%s' "$BAG_OUT_DIR"
    return 0
  fi
  local stamp suffix
  stamp="$(date +%Y%m%d_%H%M%S)"
  suffix="$BAG_MAP_NAME"
  if [ -n "$suffix" ]; then
    printf '%s/mapping_bag_%s_%s' "$BAG_BASE_DIR" "$suffix" "$stamp"
  else
    printf '%s/mapping_bag_%s' "$BAG_BASE_DIR" "$stamp"
  fi
}

OUT_DIR="$(resolve_out_dir)"
if [ "${OUT_DIR#/}" = "$OUT_DIR" ]; then
  echo "[错误] 输出目录必须是绝对路径: $OUT_DIR" >&2
  exit 2
fi
if [ "$BAG_MAP_NAME" != "${BAG_MAP_NAME//[!A-Za-z0-9_.-]/}" ]; then
  echo "[错误] --map-name 只允许字母/数字/下划线/点/短横线, 收到: $BAG_MAP_NAME" >&2
  exit 2
fi

# ---------- 组装 ros2 bag record 命令 ----------
RECORD_ARGS=(--output "$OUT_DIR" --max-cache-size 200000000)
if [ "$BAG_MAX_SIZE_MB" != "0" ]; then
  RECORD_ARGS+=(--max-bag-size "$((BAG_MAX_SIZE_MB * 1024 * 1024))")
fi
if is_true "$BAG_USE_SIM_TIME"; then
  RECORD_ARGS+=(--use-sim-time)
fi

SETUP_FILE="$WS_DIR/install/setup.bash"

# 带引号的参数表 ("ros2 bag record" 后面的部分)。
quote_args() {
  local quoted=() arg
  for arg in "${RECORD_ARGS[@]}" "${TOPICS[@]}"; do
    quoted+=("$(q "$arg")")
  done
  printf '%s' "${quoted[*]}"
}

# --check 打印用的一整条命令。
build_record_cmd() {
  printf 'bash -c %s' \
    "$(q "set +u; source $(q "$SETUP_FILE") >/dev/null 2>&1; exec ros2 bag record $(quote_args)")"
}

RECORD_CMD="$(build_record_cmd)"

# ---------- 摘要 ----------
echo "================= SRM 建图同步录包 ================="
echo "  工作空间  : $WS_DIR"
echo "  输出目录  : $OUT_DIR"
echo "  主题 (${#TOPICS[@]})  : ${TOPICS[*]}"
echo "  分卷阈值  : $([ "$BAG_MAX_SIZE_MB" = "0" ] && echo "不分卷" || echo "${BAG_MAX_SIZE_MB} MB")"
echo "  等话题    : ${BAG_WAIT} 秒 (/livox/lidar)"
echo "  体量参考  : 约 240 MB/分钟 (Mid-360 满速, 只算 /livox/lidar)"
echo "  结束方式  : Ctrl+C (rosbag2 靠 SIGINT 退出时才写 metadata.yaml)"
echo "===================================================="

if [ "$DRY_RUN" = "1" ]; then
  echo
  echo "[--check] 仅解析, 不启动。将要执行的命令:"
  echo "  mkdir -p $(q "$OUT_DIR")"
  echo "  $RECORD_CMD"
  echo
  echo "解析完成, 未启动录制。"
  exit 0
fi

# ---------- 启动前检查 ----------
if [ ! -f "$SETUP_FILE" ]; then
  echo "[错误] 未找到 $SETUP_FILE" >&2
  echo "[提示] 先编译工作空间, 或用 SRM27_WS_DIR=/path/to/ws 指定。" >&2
  exit 1
fi

# 输出目录: rosbag2(Humble) 见到**已存在**的输出目录会直接拒绝启动
# ("Output folder ... already exists"), 空目录也一样, 所以不能自己 mkdir。
# 这里改成先把空目录收掉: rmdir 只删空目录, 里面一旦有东西(上一次录的分卷)
# 就会失败, 我们据此拒绝 —— 避免两次录制混进同一个 metadata.yaml。
if [ -e "$OUT_DIR" ]; then
  if [ ! -d "$OUT_DIR" ]; then
    echo "[错误] 输出路径已存在且不是目录: $OUT_DIR" >&2
    exit 1
  fi
  if ! rmdir "$OUT_DIR" 2>/dev/null; then
    echo "[错误] 输出目录已存在且非空, 拒绝往里混录: $OUT_DIR" >&2
    echo "[提示] 换个名字/换 --out-dir, 或先确认里面没有还要的东西再自行清空。" >&2
    exit 1
  fi
fi

# 磁盘余量: 录包是单向写, 写满会把整台机器(还有正在建的图)一起拖死, 所以这里拦住。
mkdir -p "$(dirname "$OUT_DIR")"
AVAIL_KB="$(df -Pk "$(dirname "$OUT_DIR")" | awk 'NR == 2 { print $4 }')"
case "$AVAIL_KB" in ''|*[!0-9]*) AVAIL_KB=0 ;; esac
AVAIL_GB="$((AVAIL_KB / 1024 / 1024))"
if [ "$AVAIL_GB" -lt "${BAG_MIN_FREE_GB%.*}" ]; then
  echo "[错误] 可用空间只剩 ${AVAIL_GB} GB, 低于阈值 ${BAG_MIN_FREE_GB} GB。" >&2
  echo "[错误] 按 240 MB/分钟估算, ${AVAIL_GB} GB 只够录约 $((AVAIL_GB * 1000 / 240)) 分钟。" >&2
  echo "[提示] 清理空间, 或用 --min-free-gb 显式降低阈值 (后果自负)。" >&2
  exit 1
fi
echo "[录包] 可用空间 ${AVAIL_GB} GB (按 240 MB/分钟约够 $((AVAIL_GB * 1000 / 240)) 分钟)。"

# 等关键话题出现: 录包通常和建图链路同时起, 雷达驱动要几秒才把 topic 挂上来,
# 不等的话 ros2 bag record 会明确报 "topic 不存在" 并跳过, 整趟白录。
if [ "${BAG_WAIT:-0}" -gt 0 ]; then
  waited=0
  while [ "$waited" -lt "$BAG_WAIT" ]; do
    if ros2_run timeout 15 ros2 topic list --no-daemon 2>/dev/null | grep -qx "/livox/lidar"; then
      printf '[录包] /livox/lidar 已就绪 (等待 %s 秒)。\n' "$waited"
      break
    fi
    sleep 2
    waited=$((waited + 2))
  done
  if [ "$waited" -ge "$BAG_WAIT" ]; then
    echo "[警告] 等了 ${BAG_WAIT} 秒仍未看到 /livox/lidar, 照样开始录制。" >&2
    echo "[警告] 若雷达驱动还没起, 前一段数据不会被补录; 确认雷达正常后建议重录。" >&2
  fi
fi

# ---------- 录制 ----------
# 不用 exec 顶掉本脚本: 录完之后还要检查 metadata.yaml 是否写出来了 ——
# 缺了它就说明这次录制整体作废, 必须当场告警, 不能等几天后回放才发现。
#
# 关键: $! 必须**就是 ros2 的 PID**。所以让中间那层 bash 用 exec 把自己顶成
# ros2, 否则记录到的是中间 bash 的 PID, 转发信号只能打到 bash, ros2 变孤儿,
# metadata.yaml 永远写不出来 —— 这正是"录了半小时结果整包作废"的成因。
#
# 注意这里**不能**先 mkdir "$OUT_DIR": rosbag2 拒绝已存在的输出目录 (见上)。
record_argv=(bash -c "set +u; source $(q "$SETUP_FILE") >/dev/null 2>&1; exec ros2 bag record $(quote_args)")
set +e
"${record_argv[@]}" &
record_pid=$!
set -e

# SIGTERM 是外部显式发的, 不会由终端送达 ros2, 必须自己转。
# SIGINT 不在这里转: Ctrl+C 由终端直接发给整个前台进程组, --stop 也会同时
# 命中 ros2 自己的 PID, 再补一刀就是重复 SIGINT, 反而可能打断 rosbag2 收尾。
forward_signal() { kill -TERM "$record_pid" 2>/dev/null || true; }
trap forward_signal TERM

rc=0
while kill -0 "$record_pid" 2>/dev/null; do
  wait "$record_pid" || rc=$?
done
trap - INT TERM

echo
echo "[录包] ros2 bag record 退出 (退出码 $rc)。"
if [ -f "$OUT_DIR/metadata.yaml" ]; then
  db3_count="$(find "$OUT_DIR" -maxdepth 1 -name '*.db3' | wc -l)"
  size="$(du -sh "$OUT_DIR" 2>/dev/null | cut -f1)"
  echo "[录包] 完成: $OUT_DIR ($size, $db3_count 个 db3, metadata.yaml 已写)"
  echo "[录包] 离线重建 PCD:"
  echo "  ros2 bag play $(q "$OUT_DIR") --clock --topics /livox/lidar /livox/imu"
  echo "  ros2 launch srm27_nav_bringup real_mapping_launch.py start_lidar:=False use_sim_time:=True save_pcd:=True"
else
  echo "[录包] 警告: $OUT_DIR/metadata.yaml 缺失 → 这次录制不完整 (多半是被强杀了)。" >&2
  echo "[录包] 目录里的 db3 无法作为 rosbag 回放, 需要重录。" >&2
  exit 1
fi
