#!/usr/bin/env bash
# =============================================================
# 从录好的 rosbag 离线重建三维 PCD, 落到 <工作空间>/maps/pcd/<名字>.pcd
#
# 为什么需要这一步:
#   slam_toolbox 存出来的四件套 (.pgm/.yaml/.posegraph/.data) 里**没有点云**;
#   Point-LIO 的 pcd_save 又只在节点**正常退出**时落一次盘, 而且输出路径由运行期
#   SRM27_PCD_DIR 指定 (~/srm_nav_27_data/generated_pcd/point_lio/scans.pcd)。录了原始包
#   (script/record_mapping_bag.sh / start_real_slam.sh --record-bag) 就能事后重来。
#   本脚本把"起回放链路 -> 放包 -> SIGINT 收尾 -> 取走 PCD"这套手工步骤固定下来,
#   并把结果按 <名字>.pcd 放进 maps/pcd/ —— start_real_nav.sh 的 --prior-pcd 与
#   --list-maps 会在那里找。
#
# 用法:
#   ./script/build_pcd_from_bag.sh                       # 自动用 ~/srm_nav_27_data/bags 里最新的录包
#   ./script/build_pcd_from_bag.sh --name 227_1011
#   ./script/build_pcd_from_bag.sh --bag ~/srm_nav_27_data/bags/mapping_bag_227_1011_20261011_181809
#   ./script/build_pcd_from_bag.sh --bag <目录> --name site01 --force
#   ./script/build_pcd_from_bag.sh --no-mapframe         # 只重建, 不转系
#   ./script/build_pcd_from_bag.sh --bag <目录> --no-reindex   # 缺 metadata.yaml 也别动它
#   ./script/build_pcd_from_bag.sh -n                    # 只打印将执行的命令, 不真跑
#
# 参数:
#       --bag <目录>         录包目录 (默认: ~/srm_nav_27_data/bags/mapping_bag_* 里最新的)
#       --name <名字>        输出 maps/pcd/<名字>.pcd (默认从 bag 目录名里取)
#       --out-dir <绝对路径>  输出目录, 默认 <工作空间>/maps/pcd
#       --rate <倍速>        回放倍速 (默认 1.0; **别加速**, 快了 Point-LIO 会丢帧)
#       --force              已存在同名 pcd 时允许覆盖
#       --no-mapframe        只重建出 camera_init 系点云, 不接着转成地图系
#       --map-yaml <路径>    转系用的栅格图 yaml (默认 maps/<名字>/<名字>.yaml)
#       --startup-wait <秒>  最多等 point_lio 起来的秒数 (默认 60)
#       --pcd-wait <秒>      最多等 Point-LIO 退出并写完 PCD 的秒数 (默认 180)
#       --no-reindex         缺 metadata.yaml 时不自动重建 (默认自动; 只读 db3)
#   -n, --check              只打印将执行的命令, 不启动任何东西
#   -h, --help               显示本帮助
#
# 环境变量 (同名参数均可由环境变量给默认值):
#   BAG_FILE BAG_NAME PCD_OUT_DIR PCD_BAG_RATE STARTUP_WAIT PCD_WAIT FORCE_PCD
#   TO_MAPFRAME MAP_YAML_OVERRIDE BAG_TOPICS REINDEX_BAG SRM27_WS_DIR
#
# 说明:
#   - 链路参数与实车建图**完全一致**(同一份 mapping_params.yaml), 只是不起雷达驱动
#     (start_lidar:=False) 并改用包里的时钟 (use_sim_time:=True), 所以出来的点云与
#     当时若开 --save-pcd 会得到的**同一份东西** —— 这正是录包的意义。
#   - 只回放 /livox/lidar + /livox/imu: 链路自己会发 odom->base_link 与
#     base_link->livox_frame, 回放包里的 /tf 会和它抢同一条 TF。
#   - 包缺 metadata.yaml (Ctrl+C 按两次 / 关标签页 / kill -9 收尾) 时**自动**就地跑
#     ros2 bag reindex 补出来: 它只读 db3, 产物只有目录里那个 metadata.yaml, 数据不动
#     (1.2 GB 的包实测 0.3 秒)。不想让脚本动手就加 --no-reindex。
#     正在录的包不碰: 检测到 ros2 bag record 会直接拒绝。
#   - 跑之前必须没有别的建图链路在跑 (两个 point_lio 抢同一条 TF); 脚本会检查并拒绝。
#   - 旧的 point_lio/PCD/scans.pcd 会被改名成 scans_<改动时间>.pcd 保住, 不会静默覆盖。
#   - 产出两份到 maps/pcd/:
#       <名字>.pcd           Point-LIO 的 camera_init 系 (原始件)
#       <名字>_mapframe.pcd  pgm 地图系 (成品, --reloc 用这份)
#     第二份由 script/pcd_to_mapframe.py **自动接着做** (默认开, --no-mapframe 关):
#     camera_init 与地图系差一个固定位姿 (雷达相对 base_link: yaw 63°、roll -30°), 不转就去
#     --reloc, 车会被定位到错姿态上而 GICP 不会报错 —— 所以这一步不该靠人记。
#     地图 yaml (maps/<名字>/<名字>.yaml) 不存在时只告警, 并给出事后手动转的命令。
# =============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
DATA_DIR="${SRM27_DATA_DIR:-$HOME/srm_nav_27_data}"
PCD_DATA_DIR="$DATA_DIR/generated_pcd/point_lio"
export SRM27_PCD_DIR="$PCD_DATA_DIR"
mkdir -p "$PCD_DATA_DIR"

# ---------- 环境消毒 ----------
# 海康 MVS 客户端会把 /opt/MVS/lib/64 前置进 LD_LIBRARY_PATH（老 libusb 缺符号，
# 会让链接 PCL 的程序以 exit 127 静默退出）。本脚本会跑 PCL 相关的离线处理与
# ros2 CLI，环境保持一致更安全。只影响本脚本及其子进程。
# 关掉: SRM_KEEP_MVS_LIB_PATH=1
. "$SCRIPT_DIR/../lib/srm_lib_path_guard.sh"

SETUP_FILE="$WS_DIR/install/setup.bash"
BAG_DIR_DEFAULT_ROOT="$DATA_DIR/bags"
PCD_DIR_BUILD="$PCD_DATA_DIR"
BUILT_PCD="$PCD_DIR_BUILD/scans.pcd"
LOG_DIR="$DATA_DIR/diagnostics"

# ---------- 可配置项 ----------
BAG_FILE="${BAG_FILE:-}"
BAG_NAME="${BAG_NAME:-}"
PCD_OUT_DIR="${PCD_OUT_DIR:-$WS_DIR/maps/pcd}"
PCD_BAG_RATE="${PCD_BAG_RATE:-1.0}"
STARTUP_WAIT="${STARTUP_WAIT:-60}"
PCD_WAIT="${PCD_WAIT:-180}"
FORCE_PCD="${FORCE_PCD:-0}"
# 重建完是否接着转成地图系 (默认转): 不转就 --reloc 会差一个固定位姿, 而 GICP 不会报错,
# 所以这一步不该靠人记。地图 yaml 不存在 (还没 --save) 时只告警, 并给出事后手动转的命令。
TO_MAPFRAME="${TO_MAPFRAME:-1}"
MAP_YAML_OVERRIDE="${MAP_YAML_OVERRIDE:-}"
BAG_TOPICS="${BAG_TOPICS:-/livox/lidar /livox/imu}"
# 缺 metadata.yaml 时自动补 (ros2 bag reindex): 只读 db3, 只写 metadata.yaml 一个文件。
REINDEX_BAG="${REINDEX_BAG:-1}"
DRY_RUN="${DRY_RUN:-0}"

# 回放链路的进程名 (方括号包首字符, 免得 pgrep 匹配到本脚本自己的命令行)。
LAUNCH_PATTERN='[r]eal_mapping_launch\.py'
POINTLIO_PATTERN='[p]ointlio_mapping'

# ---------- 帮助 ----------
usage() {
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
    source "$SETUP_FILE" >/dev/null 2>&1
    "$@"
  )
}

# ---------- 命令行参数 ----------
while [ $# -gt 0 ]; do
  case "$1" in
    --bag)           BAG_FILE="${2:?--bag 需要目录}"; shift 2 ;;
    --name)          BAG_NAME="${2:?--name 需要名字}"; shift 2 ;;
    --out-dir)       PCD_OUT_DIR="${2:?--out-dir 需要绝对路径}"; shift 2 ;;
    --rate)          PCD_BAG_RATE="${2:?--rate 需要倍速}"; shift 2 ;;
    --force)         FORCE_PCD=1; shift ;;
    --no-mapframe)   TO_MAPFRAME=0; shift ;;
    --map-yaml)      MAP_YAML_OVERRIDE="${2:?--map-yaml 需要 yaml 路径}"; shift 2 ;;
    --startup-wait)  STARTUP_WAIT="${2:?--startup-wait 需要秒数}"; shift 2 ;;
    --pcd-wait)      PCD_WAIT="${2:?--pcd-wait 需要秒数}"; shift 2 ;;
    --no-reindex)    REINDEX_BAG=0; shift ;;
    -n|--check)      DRY_RUN=1; shift ;;
    -h|--help)       usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 -h 查看用法)" >&2; exit 2 ;;
  esac
done

for pair in "STARTUP_WAIT:$STARTUP_WAIT" "PCD_WAIT:$PCD_WAIT"; do
  case "${pair#*:}" in
    ''|*[!0-9]*) echo "[错误] ${pair%%:*} 必须是非负整数, 收到: ${pair#*:}" >&2; exit 2 ;;
  esac
done
case "$PCD_BAG_RATE" in
  ''|*[!0-9.]*|0|0.0) echo "[错误] --rate 必须是正数, 收到: $PCD_BAG_RATE" >&2; exit 2 ;;
esac
if [ "${PCD_OUT_DIR#/}" = "$PCD_OUT_DIR" ]; then
  echo "[错误] --out-dir 必须是绝对路径: $PCD_OUT_DIR" >&2
  exit 2
fi

# ---------- 选包 ----------
if [ -z "$BAG_FILE" ]; then
  # 取最新的一个录包目录: 按 metadata.yaml / db3 的修改时间排, 避免只看目录名。
  BAG_FILE="$(ls -1dt "$BAG_DIR_DEFAULT_ROOT"/mapping_bag_* 2>/dev/null | head -n 1 || true)"
  if [ -z "$BAG_FILE" ]; then
    echo "[错误] 没找到任何录包目录: $BAG_DIR_DEFAULT_ROOT/mapping_bag_*" >&2
    echo "[提示] 建图时加 --record-bag 才会录; 或显式给 --bag <目录>。" >&2
    exit 1
  fi
fi
BAG_FILE="${BAG_FILE%/}"
if [ ! -d "$BAG_FILE" ]; then
  # 允许给相对工作空间的路径
  if [ -d "$WS_DIR/$BAG_FILE" ]; then
    BAG_FILE="$WS_DIR/$BAG_FILE"
  else
    echo "[错误] 录包目录不存在: $BAG_FILE" >&2
    exit 1
  fi
fi
BAG_FILE="$(readlink -f "$BAG_FILE")"

# ---------- 正在录的包不碰 ----------
# 录包中的 db3 还在长、metadata.yaml 要到收尾才写。这时补元数据等于给半成品盖章,
# 回放也只是读到一半的数据 —— 两头都错, 所以直接拒绝。
# pattern 带前导 /: 真进程的 cmdline 是
#   /usr/bin/python3 /opt/ros/humble/bin/ros2 bag record ...
# 而"命令行里只是提了一句 ros2 bag record"的 shell/编辑器/grep 没有那个 /,
# 用 '[r]os2 bag record' 会把它们也算成在录包 (实测踩过)。
if pgrep -u "$(id -u)" -f '/[r]os2 bag record' >/dev/null 2>&1; then
  cat >&2 <<'EOF'
[错误] 现在有 ros2 bag record 在跑, 这个包还没收尾 (数据还在长)。
[提示] 先在录包那个终端 Ctrl+C 一次 (只按一次), 或 ./script/start_real_slam.sh --stop,
[提示] 等目录里出现 metadata.yaml 之后再回来重建 PCD。
EOF
  exit 1
fi

# ---------- 读包信息: metadata.yaml, 缺了就地重建 ----------
# 输出两段: 第 1 行 "<状态>|<时长秒>", 之后每行 "<话题>|<类型>|<条数>"。
# 状态 ok = metadata.yaml 可用; missing/broken = 缺或写了半截 (此时时长/话题是从 db3
# 只读统计的, 用来报错和 -n 预演); error = 连 db3 都读不出来。
read_bag_info() {
  python3 - "$BAG_FILE" <<'PYEOF'
import glob, os, sqlite3, sys

bag = sys.argv[1]
meta = os.path.join(bag, "metadata.yaml")


def from_metadata(path):
    import yaml
    info = yaml.safe_load(open(path, encoding="utf-8"))["rosbag2_bagfile_information"]
    topics = {}
    for entry in info.get("topics_with_message_count", []):
        m = entry["topic_metadata"]
        topics[m["name"]] = (m["type"], entry["message_count"])
    return info.get("duration", {}).get("nanoseconds", 0) / 1e9, topics


def from_db3():
    """只读扫 db3: metadata.yaml 没了也要能说清包里到底有什么。"""
    files = sorted(glob.glob(os.path.join(bag, "*.db3")))
    if not files:
        return None
    topics = {}
    lo = hi = None
    for path in files:
        con = sqlite3.connect("file:%s?mode=ro" % path, uri=True)
        try:
            for name, typ, count in con.execute(
                    "SELECT t.name, t.type, COUNT(*) FROM messages m "
                    "JOIN topics t ON t.id = m.topic_id GROUP BY m.topic_id"):
                prev = topics.get(name)
                topics[name] = (typ, (prev[1] if prev else 0) + count)
            first, last = con.execute(
                "SELECT MIN(timestamp), MAX(timestamp) FROM messages").fetchone()
        finally:
            con.close()
        if first is not None:
            lo = first if lo is None else min(lo, first)
            hi = last if hi is None else max(hi, last)
    if lo is None:
        return None
    return (hi - lo) / 1e9, topics


status, duration, topics = "ok", 0.0, {}
if not os.path.isfile(meta) or os.path.getsize(meta) == 0:
    status = "missing"
else:
    try:
        duration, topics = from_metadata(meta)
    except Exception as exc:
        status = "broken"
        sys.stderr.write("metadata.yaml 解析失败: %s\n" % exc)

if status != "ok":
    try:
        fallback = from_db3()
    except Exception:
        fallback = None
    if fallback is None:
        print("error|0")
        sys.exit(0)
    duration, topics = fallback

print("%s|%.1f" % (status, duration))
for name, (typ, count) in sorted(topics.items()):
    print("%s|%s|%d" % (name, typ, count))
PYEOF
}

BAG_INFO="$(read_bag_info)" || { echo "[错误] 读 $BAG_FILE 失败。" >&2; exit 1; }
META_STATUS="$(printf '%s\n' "$BAG_INFO" | head -n 1 | cut -d'|' -f1)"
BAG_DURATION="$(printf '%s\n' "$BAG_INFO" | head -n 1 | cut -d'|' -f2)"
BAG_TOPIC_LINES="$(printf '%s\n' "$BAG_INFO" | tail -n +2)"

if [ "$META_STATUS" = "error" ]; then
  echo "[错误] $BAG_FILE 里没有可读的 .db3: 它不像一个录包目录, 或者数据已经坏了。" >&2
  exit 1
fi

REINDEXED=0
if [ "$META_STATUS" != "ok" ]; then
  if [ "$META_STATUS" = "missing" ]; then
    WHY="没有 metadata.yaml"
  else
    WHY="metadata.yaml 读不出来 (多半是收尾时只写了一半)"
  fi
  echo "[预处理] $BAG_FILE: $WHY" >&2
  echo "[预处理] rosbag2 只在收到 SIGINT 收尾时才写它: Ctrl+C 按两次、关标签页、kill -9 都会缺。" >&2
  echo "[预处理] 数据本身还在 (上面已只读扫过 db3), 补元数据即可, 不用重录。" >&2

  if [ "$DRY_RUN" = "1" ]; then
    echo "[--check] 真跑时会先执行 (只读 db3, 产物只有 metadata.yaml):" >&2
    echo "[--check]   ros2 bag reindex $(q "$BAG_FILE")" >&2
    echo "[--check] 下面的时长/话题是从 db3 只读统计的, 可用于预演。" >&2
  elif ! is_true "$REINDEX_BAG"; then
    echo "[错误] --no-reindex: 没有 metadata.yaml 就没法 ros2 bag play。" >&2
    echo "[提示] 去掉 --no-reindex 让脚本补, 或手动: ros2 bag reindex $(q "$BAG_FILE")" >&2
    exit 1
  else
    # 先把半截的 metadata.yaml 挪走: 留着它, reindex 会照着坏文件去选存储插件。
    if [ -e "$BAG_FILE/metadata.yaml" ]; then
      BROKEN_META="$BAG_FILE/metadata.yaml.broken_$(date +%Y%m%d_%H%M%S)"
      mv "$BAG_FILE/metadata.yaml" "$BROKEN_META"
      echo "[预处理] 半截的 metadata.yaml 已改名保留: $(basename "$BROKEN_META")" >&2
    fi
    if [ ! -w "$BAG_FILE" ]; then
      echo "[错误] $BAG_FILE 不可写, reindex 没地方落 metadata.yaml。" >&2
      exit 1
    fi
    echo "[预处理] ros2 bag reindex 重建 metadata.yaml (只读 db3, 数据不动)..."
    if ! ros2_run timeout 600 ros2 bag reindex "$BAG_FILE"; then
      echo "[错误] ros2 bag reindex 失败, 这个包补不回来。" >&2
      echo "[错误] 常见原因: 录制被打断时 db3 正写到一半 —— 那是数据本身坏了, 不只是缺元数据。" >&2
      echo "[提示] 这种只能重录; 这一趟没有可用输入, 不往下跑。" >&2
      exit 1
    fi
    BAG_INFO="$(read_bag_info)"
    META_STATUS="$(printf '%s\n' "$BAG_INFO" | head -n 1 | cut -d'|' -f1)"
    BAG_DURATION="$(printf '%s\n' "$BAG_INFO" | head -n 1 | cut -d'|' -f2)"
    BAG_TOPIC_LINES="$(printf '%s\n' "$BAG_INFO" | tail -n +2)"
    if [ "$META_STATUS" != "ok" ]; then
      echo "[错误] reindex 跑完了, metadata.yaml 仍不可用 ($META_STATUS), 不往下跑。" >&2
      exit 1
    fi
    # Humble 的 reindex 会把 storage_identifier 留成空串; 空值只是"回退到默认插件"
    # (这里恰好是 sqlite3 才放得动)。补上实际值, 免得别的工具按空值选错插件。
    if find "$BAG_FILE" -maxdepth 1 -name '*.db3' -print -quit | grep -q .; then
      sed -i 's/^\( *\)storage_identifier: *""/\1storage_identifier: sqlite3/' "$BAG_FILE/metadata.yaml"
    fi
    REINDEXED=1
    echo "[预处理] 已补出 metadata.yaml: ${BAG_DURATION} 秒, $(printf '%s\n' "$BAG_TOPIC_LINES" | grep -c '|') 个话题, 可以回放。"
    echo "[预处理] 强杀时丢的是内存里最后几秒缓存; 剩下这些点云仍然能用, 介意就重录。"
  fi
fi

bag_has_topic() { printf '%s\n' "$BAG_TOPIC_LINES" | cut -d'|' -f1 | grep -qx "$1"; }

REPLAY_TOPICS=()
for t in $BAG_TOPICS; do
  REPLAY_TOPICS+=("$t")
done

MISSING=""
for t in "${REPLAY_TOPICS[@]}"; do
  if ! bag_has_topic "$t"; then
    MISSING="$MISSING $t"
  fi
done
if [ -n "$MISSING" ]; then
  echo "[错误] 这个包里缺少回放必需的话题:$MISSING" >&2
  echo "[错误] 包里实际有:" >&2
  printf '  %s\n' "$BAG_TOPIC_LINES" >&2
  echo "[提示] Point-LIO 是激光惯性里程计, 没有 IMU 就跑不起来; 建图录包请用默认主题集。" >&2
  exit 1
fi

# ---------- 定名字与输出路径 ----------
if [ -z "$BAG_NAME" ]; then
  # mapping_bag_<名字>_<YYYYmmdd>_<HHMMSS> -> <名字>
  BAG_NAME="$(basename "$BAG_FILE" | sed -n 's/^mapping_bag_\(.*\)_[0-9]\{8\}_[0-9]\{6\}$/\1/p')"
fi
if [ -z "$BAG_NAME" ]; then
  echo "[错误] 无法从包目录名推出地图名: $(basename "$BAG_FILE")" >&2
  echo "[提示] 用 --name <名字> 显式指定。" >&2
  exit 2
fi
if [ "$BAG_NAME" != "${BAG_NAME//[!A-Za-z0-9_.-]/}" ]; then
  echo "[错误] --name 只允许字母/数字/下划线/点/短横线, 收到: $BAG_NAME" >&2
  exit 2
fi

# --name 给的是"地图名"; _mapframe 是脚本自己加的后缀。传 227_1113_mapframe 会得到
# 227_1113_mapframe.pcd + 227_1113_mapframe_mapframe.pcd, 而且 maps/227_1113_mapframe/
# 不存在, 转系只能告警跳过。名字里的意图一眼可辨, 直接按意图纠正 —— 别让人跑完
# 6 分钟才发现产物名字不对。
if is_true "$TO_MAPFRAME" && [ "$BAG_NAME" != "${BAG_NAME%_mapframe}" ] \
   && [ -n "${BAG_NAME%_mapframe}" ]; then
  echo "[提示] --name $BAG_NAME 结尾的 _mapframe 是多余的: 脚本会自己产出 <名字>_mapframe.pcd"
  echo "[提示] 按 $BAG_NAME 去掉后缀处理 -> ${BAG_NAME%_mapframe}"
  BAG_NAME="${BAG_NAME%_mapframe}"
fi

OUT_PCD="$PCD_OUT_DIR/$BAG_NAME.pcd"

# ---------- 组装命令 ----------
# ⚠ 必须显式指定 point_lio：real_mapping_launch.py 的 lio_frontend 默认已改成 zlio
# （与实车建图一致），而**累积点云落盘 (save_pcd -> scans.pcd) 是 Point-LIO 的功能**，
# zlio_bridge 没有这条路。少这一项会起 zlio、等不到 /point_lio，白跑一趟。
# 2026-11-11 实测：漏写时日志里只有 zlio_node，脚本卡在"等 point_lio 订阅"。
LIO_FRONTEND_FOR_PCD="${LIO_FRONTEND_FOR_PCD:-point_lio}"
LAUNCH_ARGS="start_lidar:=False use_sim_time:=True save_pcd:=True use_rviz:=False"
LAUNCH_ARGS="$LAUNCH_ARGS lio_frontend:=$LIO_FRONTEND_FOR_PCD"
LAUNCH_CMD="ros2 launch srm27_nav_bringup real_mapping_launch.py $LAUNCH_ARGS"

TOPIC_ARGS="$(printf '%s ' "${REPLAY_TOPICS[@]}")"
PLAY_CMD="ros2 bag play $(q "$BAG_FILE") --clock --topics ${TOPIC_ARGS% } --rate $PCD_BAG_RATE"

RUN_TAG="$(date +%Y%m%d_%H%M%S)"
RUN_LOG="$LOG_DIR/build_pcd_${BAG_NAME}_${RUN_TAG}.log"

# ---------- 摘要 ----------
echo "================= 从 rosbag 重建 PCD ================="
echo "  录包目录  : $BAG_FILE"
if [ "$REINDEXED" = "1" ]; then
  echo "  元数据    : 刚自动补出 (原包缺 metadata.yaml, 被强杀收尾; 尾部几秒可能缺)"
fi
echo "  时长      : ${BAG_DURATION} 秒 (按 ${PCD_BAG_RATE} 倍速回放)"
echo "  回放话题  : ${REPLAY_TOPICS[*]}"
echo "  输出      : $OUT_PCD"
echo "  链路日志  : $RUN_LOG"
echo "  前端      : $LIO_FRONTEND_FOR_PCD (PCD 累积只有 Point-LIO 支持)
  坐标系    : camera_init (Point-LIO 世界系, 不是 map 系)"
echo "======================================================"

if [ "$DRY_RUN" = "1" ]; then
  echo
  echo "[--check] 仅解析, 不启动、不改任何文件。将要执行的步骤:"
  echo "  0) (仅当缺 metadata.yaml) ros2 bag reindex $(q "$BAG_FILE")   # 只读 db3, 只补 metadata.yaml"
  echo "  1) 备份已存在的 $BUILT_PCD -> $PCD_DIR_BUILD/scans_<改动时间>.pcd"
  echo "  2) (source $SETUP_FILE) $LAUNCH_CMD"
  echo "     日志 -> $RUN_LOG   等 /point_lio 订阅上 /livox/lidar + /livox/imu (最多 ${STARTUP_WAIT} 秒)"
  echo "  3) (source $SETUP_FILE) $PLAY_CMD"
  echo "  4) SIGINT 给 point_lio, 收掉回放链路"
  echo "  5) 校验 $BUILT_PCD 完整性"
  echo "  6) 移动到 $OUT_PCD"
  if is_true "$TO_MAPFRAME"; then
    echo "  7) 转成地图系 -> ${OUT_PCD%.pcd}_mapframe.pcd"
    echo "     python3 script/pcd_to_mapframe.py --pcd $OUT_PCD --map ${MAP_YAML_OVERRIDE:-$WS_DIR/maps/$BAG_NAME/$BAG_NAME.yaml}"
  else
    echo "  7) [--no-mapframe] 跳过转系, 只留 camera_init 系原始件"
  fi
  echo
  echo "解析完成, 未启动任何进程。"
  exit 0
fi

# ---------- 启动前检查 ----------
if [ ! -f "$SETUP_FILE" ]; then
  echo "[错误] 未找到 $SETUP_FILE" >&2
  echo "[提示] 先编译工作空间, 或用 SRM27_WS_DIR=/path/to/ws 指定。" >&2
  exit 1
fi
if [ ! -d "$PCD_DIR_BUILD" ]; then
  echo "[错误] Point-LIO 的 PCD 目录不存在: $PCD_DIR_BUILD" >&2
  echo "[提示] 这个目录是 Point-LIO 编译期 ROOT_DIR 的一部分, 删了就没地方落盘。" >&2
  exit 1
fi
if pgrep -u "$(id -u)" -f "$POINTLIO_PATTERN" >/dev/null 2>&1 \
   || pgrep -u "$(id -u)" -f "$LAUNCH_PATTERN" >/dev/null 2>&1; then
  cat >&2 <<'EOF'
[错误] 已经有建图链路在跑 (pointlio_mapping / real_mapping_launch)。
[错误] 两个 point_lio 会抢同一条 TF, 出来的点云必然是错的。
[错误] 先停掉: ./script/start_real_slam.sh --stop
EOF
  exit 1
fi
if [ -e "$OUT_PCD" ] && ! is_true "$FORCE_PCD"; then
  echo "[错误] 输出已存在: $OUT_PCD" >&2
  echo "[提示] 换个名字: --name ${BAG_NAME}_b" >&2
  echo "[提示] 或确认要覆盖后加 --force。" >&2
  exit 1
fi
mkdir -p "$PCD_OUT_DIR" "$LOG_DIR"

# ---------- 收尾: 任何退出路径都不能留下跑着的链路 ----------
LAUNCH_PID=""
cleanup() {
  local rc=$?
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[收尾] 结束回放链路..." >&2
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 3
    kill -KILL "$LAUNCH_PID" 2>/dev/null || true
  fi
  # launch 被强杀时子节点可能残留, 按名字兜底一遍 (含 RViz/驱动, 这里只清重建链路那几个)。
  pkill -KILL -u "$(id -u)" -f "$POINTLIO_PATTERN" 2>/dev/null || true
  pkill -KILL -u "$(id -u)" -f '[l]oam_interface_node' 2>/dev/null || true
  pkill -KILL -u "$(id -u)" -f '[s]ensor_scan_generation_node' 2>/dev/null || true
  pkill -KILL -u "$(id -u)" -f '[p]ointcloud_to_laserscan_node' 2>/dev/null || true
  pkill -KILL -u "$(id -u)" -f '[r]obot_state_publisher' 2>/dev/null || true
  pkill -KILL -u "$(id -u)" -f '[s]ync_slam_toolbox_node' 2>/dev/null || true
  exit "$rc"
}
trap cleanup EXIT

# ---------- 1) 保住旧的 scans.pcd ----------
if [ -f "$BUILT_PCD" ]; then
  # 用它的改动时间命名: 这样"被覆盖掉的是哪一份"永远查得到 (上一次就是这么丢的)。
  OLD_TS="$(date -r "$BUILT_PCD" +%Y%m%d_%H%M%S)"
  OLD_KEEP="$PCD_DIR_BUILD/scans_${OLD_TS}.pcd"
  n=1
  while [ -e "$OLD_KEEP" ]; do
    OLD_KEEP="$PCD_DIR_BUILD/scans_${OLD_TS}_${n}.pcd"
    n=$((n + 1))
  done
  mv "$BUILT_PCD" "$OLD_KEEP"
  echo "[1/7] 旧的 scans.pcd 已保到 $(basename "$OLD_KEEP")"
else
  echo "[1/7] Point-LIO 落盘目录是空的, 不需要备份。"
fi

# ---------- 2) 起回放链路 ----------
echo "[2/7] 起回放链路 (start_lidar:=False use_sim_time:=True save_pcd:=True)..."
# 用 subshell 里 source 再 exec, 这样 $! 就是 ros2 launch 本身, 信号能直达。
set +e
( set +u; source "$SETUP_FILE" >/dev/null 2>&1; exec ros2 launch srm27_nav_bringup real_mapping_launch.py \
    start_lidar:=False use_sim_time:=True save_pcd:=True use_rviz:=False \
    lio_frontend:="$LIO_FRONTEND_FOR_PCD" ) > "$RUN_LOG" 2>&1 &
LAUNCH_PID=$!
set -e

node_up=0
waited=0
while [ "$waited" -lt "$STARTUP_WAIT" ]; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[错误] 回放链路启动就退出了, 日志尾部:" >&2
    tail -20 "$RUN_LOG" >&2
    exit 1
  fi
  # 光看到 /point_lio 节点还不够: DDS 的 reader-writer 匹配比节点上线晚,
  # 这时放包会静默丢掉开头几秒的点云 —— 后果是 LIO 世界原点落在轨迹中途,
  # 点云覆盖面积变小 (实测两趟 16.3 m vs 12.8 m), 而且对齐变换也跟着变。
  # 所以要等到它**真的订阅上了** /livox/lidar 与 /livox/imu 再放。
  if ros2_run timeout 15 ros2 node list --no-daemon 2>/dev/null | grep -qx "/point_lio" \
     && ros2_run timeout 15 ros2 topic info /livox/lidar 2>/dev/null | grep -q "Subscription count: [1-9]" \
     && ros2_run timeout 15 ros2 topic info /livox/imu 2>/dev/null | grep -q "Subscription count: [1-9]"; then
    node_up=1
    break
  fi
  sleep 2
  waited=$((waited + 2))
done
if [ "$node_up" != "1" ]; then
  echo "[错误] 等了 ${STARTUP_WAIT} 秒, /point_lio 仍没有订阅上 /livox/lidar + /livox/imu, 日志尾部:" >&2
  tail -20 "$RUN_LOG" >&2
  exit 1
fi
echo "      /point_lio 已就绪且已订阅点云/IMU (等待 ${waited} 秒)。"
# 订阅在图上出现后, 传输层再给一点匹配余量, 免得第一帧又漏掉。
sleep 2

# ---------- 3) 放包 ----------
echo "[3/7] 放包 (约 ${BAG_DURATION} 秒, ${PCD_BAG_RATE} 倍速)..."
(
  set +u
  # shellcheck disable=SC1091
  source "$SETUP_FILE" >/dev/null 2>&1
  ros2 bag play "$BAG_FILE" --clock --topics "${REPLAY_TOPICS[@]}" --rate "$PCD_BAG_RATE"
) || { echo "[错误] ros2 bag play 失败。" >&2; exit 1; }

# ---------- 4) 收尾 ----------
# 先直接给 point_lio SIGINT: 只有它走正常退出流程才会写 PCD (和实车 --stop 同一招)。
echo "[4/7] 收尾: 通知 point_lio 退出并落盘..."
sleep 2
pkill -INT -u "$(id -u)" -f "$POINTLIO_PATTERN" 2>/dev/null || true
kill -INT "$LAUNCH_PID" 2>/dev/null || true

waited=0
while [ "$waited" -lt "$PCD_WAIT" ]; do
  if ! pgrep -u "$(id -u)" -f "$POINTLIO_PATTERN" >/dev/null 2>&1; then
    break
  fi
  sleep 2
  waited=$((waited + 2))
done
if pgrep -u "$(id -u)" -f "$POINTLIO_PATTERN" >/dev/null 2>&1; then
  echo "[错误] point_lio 在 ${PCD_WAIT} 秒内没有退出, PCD 可能没写出来。" >&2
  echo "[提示] 看日志: $RUN_LOG" >&2
  exit 1
fi
# 进程已退出 = 退出前的 writeBinary 已完成, 这里再给文件系统一点余量。
sleep 2
if [ ! -s "$BUILT_PCD" ]; then
  echo "[错误] point_lio 退出了, 但没有写出 $BUILT_PCD" >&2
  echo "[错误] 最常见的原因: 这一趟没有真正收到点云 (包里话题对不上 / 时间戳问题)。" >&2
  echo "[提示] 看日志: $RUN_LOG" >&2
  exit 1
fi
echo "      point_lio 已退出, PCD 已写出 (等待 ${waited} 秒)。"

# ---------- 5) 校验 ----------
echo "[5/7] 校验 PCD 完整性..."
if ! python3 - "$BUILT_PCD" <<'PYEOF'
import re, sys
path = sys.argv[1]
data = open(path, "rb").read()
marker = b"DATA binary\n"
end = data.index(marker) + len(marker)
header = data[:end].decode("ascii", "replace")
points = int(re.search(r"POINTS (\d+)", header).group(1))
step = sum(int(x) for x in re.search(r"SIZE (.*)", header).group(1).split())
expect = points * step
actual = len(data) - end
print("      点数 %d, 每点 %d 字节" % (points, step))
if expect != actual:
    print("      数据区应有 %d 字节, 实际 %d 字节 —— 文件不完整!" % (expect, actual), file=sys.stderr)
    sys.exit(1)
# 顺带报一下范围, 一眼能看出是不是真地图
try:
    import numpy as np
    arr = np.frombuffer(data[end:], dtype=np.float32).reshape(points, -1)
    xyz = arr[:, :3]
    print("      x %.2f~%.2f  y %.2f~%.2f  z %.2f~%.2f (m)" % (
        xyz[:, 0].min(), xyz[:, 0].max(),
        xyz[:, 1].min(), xyz[:, 1].max(),
        xyz[:, 2].min(), xyz[:, 2].max()))
except Exception:
    pass
PYEOF
then
  echo "[错误] PCD 校验失败, 不移动它: $BUILT_PCD" >&2
  exit 1
fi

# ---------- 6) 落地 ----------
echo "[6/7] 移动到 $OUT_PCD"
mv "$BUILT_PCD" "$OUT_PCD"

# ---------- 7) 接着转成地图系 ----------
# 不转就没法用: camera_init 系与 pgm 地图系差一个固定位姿 (雷达 yaw 63°、roll -30°),
# 直接 --reloc 车会被定位到错姿态而 GICP 不报错。这一步默认做, 别让人靠记忆。
# 装歪的 roll 由 pcd_to_mapframe.py 按 --level-mount 自动摆正 (只按平面拟合会差 0.15 m)。
MAP_YAML="${MAP_YAML_OVERRIDE:-$WS_DIR/maps/$BAG_NAME/$BAG_NAME.yaml}"
MAPFRAME_PCD="${OUT_PCD%.pcd}_mapframe.pcd"
MAPFRAME_OK=0
if is_true "$TO_MAPFRAME"; then
  if [ -f "$MAP_YAML" ]; then
    echo
    echo "[7/7] 转到地图系 (--no-mapframe 可跳过)..."
    if python3 "$SCRIPT_DIR/pcd_to_mapframe.py" --pcd "$OUT_PCD" --map "$MAP_YAML"; then
      MAPFRAME_OK=1
    else
      echo "[警告] 转系失败 (见上面的错误)。原始件已留在 $OUT_PCD。" >&2
    fi
  else
    echo "[警告] 没找到地图 yaml: $MAP_YAML" >&2
    echo "[警告] 这张图还没 --save 的话, 先 ./script/start_real_slam.sh --save $BAG_NAME (需要链路在跑)," >&2
    echo "[警告] 或之后用 --map-yaml 指定别的地图再转。" >&2
  fi
fi

if [ "$MAPFRAME_OK" = "1" ]; then
  cat <<EOF

完成 (可直接用于重定位):
  $MAPFRAME_PCD  ($(du -h "$MAPFRAME_PCD" | cut -f1))   <- 地图系, --reloc 用这份
  $OUT_PCD  ($(du -h "$OUT_PCD" | cut -f1))   <- camera_init 系原始件
  链路日志: $RUN_LOG

  ./script/start_real_nav.sh --list-maps                        # 应显示 先验PCD: 有
  ./script/start_real_nav.sh --reloc -m $BAG_NAME --prior-pcd $BAG_NAME
  # --prior-pcd $BAG_NAME 会优先命中 _mapframe 那份 (见 start_real_nav.sh 的搜索顺序)

⚠ 每次重建出来的点云不是逐点可复现的, 重建之后必须重新转系 (本脚本默认已经做了);
  不要把旧的 $(basename "$MAPFRAME_PCD") 配新的原始件用。
EOF
else
  cat <<EOF

重建完成 (仍是 camera_init 系):
  $OUT_PCD  ($(du -h "$OUT_PCD" | cut -f1))
  链路日志: $RUN_LOG

下一步 — 转成地图系 (不转直接 --reloc 车会被定位到错姿态上, 而 GICP 不会报错):
  python3 script/pcd_to_mapframe.py \\
      --pcd $OUT_PCD \\
      --map $MAP_YAML

  它会在同目录产出 $(basename "$MAPFRAME_PCD") (原始件不动), 头部标注坐标系与所用变换。
  转好之后:
    ./script/start_real_nav.sh --list-maps                       # 应显示 先验PCD: 有
    ./script/start_real_nav.sh --reloc -m $BAG_NAME --prior-pcd $BAG_NAME

  - 想先看一眼点云 (camera_init 系那份):
      ros2 run pcl_ros pcd_to_pointcloud --ros-args -p file_name:=$OUT_PCD -p tf_frame:=camera_init
      RViz: Fixed Frame 填 camera_init, Add -> By topic -> /cloud_pcd
EOF
fi
