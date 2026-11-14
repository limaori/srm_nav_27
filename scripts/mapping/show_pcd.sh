#!/usr/bin/env bash
# =============================================================
# 在 RViz 里看一份 PCD（先验点云 / 重建点云）
#
# 为什么要这个脚本:
#   1) 手敲 `ros2 run pcl_ros pcd_to_pointcloud` 会撞上 MVS 的 LD_LIBRARY_PATH
#      （/opt/MVS/lib/64 里那份 2023 年的 libusb-1.0 缺 libusb_set_option，
#       导致 libpcl_io 加载失败、进程 Aborted）。本脚本自己剔除这两个路径。
#   2) 点云只有 `map` 一个 frame，而此刻通常没有任何节点发 TF，RViz 会一路
#      "Message Filter dropping message ... queue is full"，什么都不显示。
#      本脚本补一个 `world -> map` 静态变换，RViz 的 Fixed Frame 用 world。
#   3) 建图重建出来的 PCD 动辄 700 万点 / 200 MB，单条消息太大 RViz 吃不下。
#      默认先做 5 cm 体素降采样（缓存成 <名字>_view.pcd，秒开）；要原样看加 --raw。
#
# 用法:
#   ./script/show_pcd.sh 227_1111                 # 自动找 maps/pcd/227_1111_mapframe.pcd
#   ./script/show_pcd.sh maps/pcd/foo.pcd         # 直接给路径
#   ./script/show_pcd.sh 227_1111 --rviz          # 连 rviz2 一起起（自带配置）
#   ./script/show_pcd.sh 227_1111 --raw           # 不降采样
#   ./script/show_pcd.sh 227_1111 --downsample 0.03
#   ./script/show_pcd.sh 227_1111 --map 227_1111  # 同时把 2D 栅格图（/map）发出来叠加看
#   ./script/show_pcd.sh 227_1111 --no-tf         # 已有 TF（比如导航在跑）时不发静态变换
#   ./script/show_pcd.sh 227_1111 --frame camera_init --world camera_init
#   ./script/show_pcd.sh 227_1111 -n              # 只打印将执行的命令
#
# 起来之后（RViz 里）:
#   Global Options → Fixed Frame = world
#   Add → By topic → /cloud_pcd
# 已经用 --rviz 起好 RViz 的，这一步不用做。
# =============================================================

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
DATA_DIR="${SRM27_DATA_DIR:-$HOME/srm_nav_27_data}"
PCD_DIR="$WS_DIR/maps/pcd"
SETUP_FILE="$WS_DIR/install/setup.bash"

MAP_DIR="$WS_DIR/maps"
RVZ_CFG_SRC="$WS_DIR/src/srm27_navigation/srm27_nav_bringup/rviz/pcd_view.rviz"
RVZ_CFG_INST="$WS_DIR/install/srm27_nav_bringup/share/srm27_nav_bringup/rviz/pcd_view.rviz"

# ---------- 参数 ----------
TARGET=""
FRAME=""
WORLD="world"
DOWNSAMPLE="0.05"
RAW=0
START_RVIZ=0
DO_TF=1
DRY_RUN=0
MAP_ARG=""
PERIOD_MS="1000"

usage() { sed -n '2,32p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    --frame)       FRAME="${2:?--frame 需要坐标系名}"; shift 2 ;;
    --world)       WORLD="${2:?--world 需要坐标系名}"; shift 2 ;;
    --downsample)  DOWNSAMPLE="${2:?--downsample 需要体素边长}"; shift 2 ;;
    --period-ms)   PERIOD_MS="${2:?--period-ms 需要毫秒}"; shift 2 ;;
    --raw)         RAW=1; shift ;;
    --rviz)        START_RVIZ=1; shift ;;
    --map)         MAP_ARG="${2:?--map 需要地图名或 yaml 路径}"; shift 2 ;;
    --no-tf)       DO_TF=0; shift ;;
    -n|--dry-run)  DRY_RUN=1; shift ;;
    -h|--help)     usage; exit 0 ;;
    -*)            echo "[错误] 未知参数: $1" >&2; usage >&2; exit 2 ;;
    *)             if [ -n "$TARGET" ]; then echo "[错误] 只能给一个点云" >&2; exit 2; fi
                   TARGET="$1"; shift ;;
  esac
done

[ -n "$TARGET" ] || { echo "[错误] 需要给点名或点云路径（-h 看用法）" >&2; exit 2; }

# ---------- 定位文件 ----------
if [ -f "$TARGET" ]; then
  PCD_FILE="$(cd "$(dirname "$TARGET")" && pwd)/$(basename "$TARGET")"
else
  PCD_FILE=""
  for cand in "$PCD_DIR/${TARGET}_mapframe.pcd" "$PCD_DIR/${TARGET}.pcd" \
              "$PCD_DIR/${TARGET}_mapframe_view.pcd"; do
    if [ -f "$cand" ]; then PCD_FILE="$cand"; break; fi
  done
fi
if [ -z "$PCD_FILE" ] || [ ! -f "$PCD_FILE" ]; then
  echo "[错误] 找不到点云: $TARGET" >&2
  echo "       可用的（$PCD_DIR）:" >&2
  ls "$PCD_DIR" 2>/dev/null | grep '\.pcd$' | sed 's/^/         /' >&2
  exit 1
fi

# ---------- 2D 地图（可选，用来和点云叠加） ----------
MAP_YAML=""
if [ -n "$MAP_ARG" ]; then
  if [ -f "$MAP_ARG" ]; then
    MAP_YAML="$(cd "$(dirname "$MAP_ARG")" && pwd)/$(basename "$MAP_ARG")"
  elif [ -f "$MAP_DIR/$MAP_ARG/$MAP_ARG.yaml" ]; then
    MAP_YAML="$MAP_DIR/$MAP_ARG/$MAP_ARG.yaml"
  fi
  if [ -z "$MAP_YAML" ]; then
    echo "[错误] 找不到 2D 地图: $MAP_ARG" >&2
    echo "       可用的（$MAP_DIR）: $(ls "$MAP_DIR" 2>/dev/null | tr '\n' ' ')" >&2
    exit 1
  fi
fi

# ---------- 点云坐标系 ----------
# 头部带 "# frame: ... map frame of maps/<名字>/<名字>.yaml" 的是 pcd_to_mapframe.py
# 转过的成品；其余（没有该标注）按 camera_init（Point-LIO 世界系）处理。
if [ -z "$FRAME" ]; then
  if grep -q -m1 'map frame of' "$PCD_FILE" 2>/dev/null; then
    FRAME="map"
  elif grep -q -m1 'camera_init' "$PCD_FILE" 2>/dev/null; then
    FRAME="camera_init"
  else
    FRAME="map"
  fi
fi

# ---------- 点云点数 ----------
POINTS="$(LC_ALL=C grep -m1 -a '^POINTS ' "$PCD_FILE" | awk '{print $2}')"
[ -n "$POINTS" ] || POINTS="?"

# ---------- RViz 配置 ----------
if [ -f "$RVZ_CFG_INST" ]; then RVZ_CFG="$RVZ_CFG_INST"
elif [ -f "$RVZ_CFG_SRC" ]; then RVZ_CFG="$RVZ_CFG_SRC"
else RVZ_CFG=""; fi

echo "==================== 看 PCD ===================="
echo "  点云文件 : $PCD_FILE"
echo "  点数     : $POINTS"
echo "  坐标系   : $FRAME   （RViz 里 Fixed Frame 用 $WORLD）"
echo "  静态 TF  : $([ "$DO_TF" = 1 ] && echo "$WORLD -> $FRAME" || echo "不发（--no-tf）")"
echo "  降采样   : $([ "$RAW" = 1 ] && echo "关（--raw，发原文件）" || echo "体素 ${DOWNSAMPLE} m")"
echo "  RViz     : $([ "$START_RVIZ" = 1 ] && echo "自动起 $RVZ_CFG" || echo "不自动起")"
echo "  2D 地图  : $([ -n "$MAP_YAML" ] && echo "$MAP_YAML -> /map" || echo "不发（加 --map <名字> 可叠加栅格图）")"
echo "================================================"

# ---------- 降采样（缓存） ----------
PUB_FILE="$PCD_FILE"
if [ "$RAW" != 1 ]; then
  STEM="${PCD_FILE%.pcd}"
  VIEW_FILE="${STEM}_view.pcd"
  if [ -f "$VIEW_FILE" ] && [ "$VIEW_FILE" -nt "$PCD_FILE" ]; then
    PUB_FILE="$VIEW_FILE"
    echo "[降采样] 复用缓存 $VIEW_FILE"
  else
    echo "[降采样] ${DOWNSAMPLE} m 体素 -> $VIEW_FILE ..."
    if [ "$DRY_RUN" = 1 ]; then
      echo "         (dry-run，跳过)"
      PUB_FILE="$VIEW_FILE"
    else
      python3 - "$PCD_FILE" "$VIEW_FILE" "$DOWNSAMPLE" <<'PYEOF'
import sys
import numpy as np

src, dst, voxel = sys.argv[1], sys.argv[2], float(sys.argv[3])
with open(src, "rb") as handle:
    header = []
    while True:
        line = handle.readline()
        if not line:
            raise SystemExit("[错误] PCD 头部不完整")
        text = line.decode("ascii", "replace").strip()
        header.append(text)
        if text.startswith("DATA"):
            break
    payload = handle.read()

info = {}
for line in header:
    if line.startswith("#") or " " not in line:
        continue
    key, _, value = line.partition(" ")
    info[key] = value

if info.get("DATA", "").strip() != "binary":
    raise SystemExit("[错误] 只支持 DATA binary 的 PCD（这份是 %r）" % info.get("DATA"))

fields = info["FIELDS"].split()
count = int(info["POINTS"])
dtype = np.dtype([(name, "<f4") for name in fields])
points = np.frombuffer(payload, dtype=dtype, count=count)
xyz = np.c_[points["x"], points["y"], points["z"]].astype(np.float32)
xyz = xyz[np.isfinite(xyz).all(axis=1)]

key = np.floor(xyz / voxel).astype(np.int64)
_, idx = np.unique(key, axis=0, return_index=True)
kept = xyz[np.sort(idx)]
if len(kept) > 800000:                        # RViz 友好的上限
    picked = np.random.default_rng(0).choice(len(kept), 800000, replace=False)
    kept = kept[np.sort(picked)]

with open(dst, "wb") as out:
    out.write((
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
        % (len(kept), len(kept))
    ).encode())
    out.write(kept.tobytes())
print("          %d 点 -> %d 点" % (count, len(kept)))
PYEOF
      [ $? -eq 0 ] || { echo "[错误] 降采样失败，可用 --raw 直接发原文件" >&2; exit 1; }
      PUB_FILE="$VIEW_FILE"
    fi
  fi
fi

PUB_POINTS="$(LC_ALL=C grep -m1 -a '^POINTS ' "$PUB_FILE" | awk '{print $2}')"

# ---------- 环境 ----------
# MVS 自带的老 libusb-1.0 缺 libusb_set_option()，会让 libpcl_io 加载失败（Aborted）。
STRIP_MVS='export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ":" "\n" | grep -v "^/opt/MVS/" | paste -sd:)'

TF_CMD="ros2 run tf2_ros static_transform_publisher --frame-id $WORLD --child-frame-id $FRAME"
PUB_CMD="ros2 run pcl_ros pcd_to_pointcloud --ros-args -p file_name:=$PUB_FILE -p tf_frame:=$FRAME -p publishing_period_ms:=$PERIOD_MS"
RVZ_CMD="rviz2${RVZ_CFG:+ -d $RVZ_CFG}"

if [ "$DRY_RUN" = 1 ]; then
  echo
  echo "[--dry-run] 将要执行:"
  echo "  1) $STRIP_MVS"
  echo "  2) source $SETUP_FILE"
  [ "$DO_TF" = 1 ] && echo "  3) $TF_CMD &"
  echo "  4) $PUB_CMD &"
  [ -n "$MAP_YAML" ] && echo "  5) ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_YAML -p frame_id:=$FRAME -p topic_name:=map &（再 lifecycle configure+activate）"
  [ "$START_RVIZ" = 1 ] && echo "  6) $RVZ_CMD   （Fixed Frame 已是 $WORLD）"
  exit 0
fi

# ---------- 启动 ----------
set +u
# shellcheck disable=SC1090
source "$SETUP_FILE" >/dev/null 2>&1 || { echo "[错误] source 失败: $SETUP_FILE" >&2; exit 1; }
eval "$STRIP_MVS"
set -u

kill_tree() {
  local pid="$1" child
  [ -n "$pid" ] || return 0
  for child in $(pgrep -P "$pid" 2>/dev/null); do
    kill_tree "$child"
  done
  kill -TERM "$pid" 2>/dev/null
}

TF_PID=""
PUB_PID=""
RVZ_PID=""
cleanup() {
  trap - INT TERM EXIT
  # ros2 run 会 fork 出真正的节点（ros2 run 进程 -> 节点进程），只杀 wrapper 会留孤儿，
  # 所以递归杀整棵子树。
  for pid in "$RVZ_PID" "$MAP_PID" "$PUB_PID" "$TF_PID"; do
    kill_tree "$pid"
  done
  wait 2>/dev/null
  echo "[退出] 已停掉发布节点"
}
trap cleanup INT TERM EXIT

PUB_LOG="/tmp/show_pcd_$$.log"

if [ "$DO_TF" = 1 ]; then
  $TF_CMD >/dev/null 2>&1 &
  TF_PID=$!
fi
MAP_PID=""
if [ -n "$MAP_YAML" ]; then
  ros2 run nav2_map_server map_server --ros-args \
      -p yaml_filename:="$MAP_YAML" -p frame_id:="$FRAME" -p topic_name:=map \
      >"$PUB_LOG.map" 2>&1 &
  MAP_PID=$!
  # map_server 是 lifecycle 节点：必须 configure + activate 才会发 /map
  for _ in $(seq 1 20); do
    if ros2 lifecycle set /map_server configure >/dev/null 2>&1; then break; fi
    sleep 0.5
  done
  ros2 lifecycle set /map_server activate >/dev/null 2>&1
  echo "[2D 地图] /map <- $MAP_YAML"
fi
$PUB_CMD >"$PUB_LOG" 2>&1 &
PUB_PID=$!
sleep 2
if ! kill -0 "$PUB_PID" 2>/dev/null; then
  echo "[错误] pcd_to_pointcloud 起不来，日志:" >&2
  tail -5 "$PUB_LOG" >&2
  exit 1
fi

echo
echo "[发布中] /cloud_pcd  frame=$FRAME  ($PUB_POINTS 点)"
if [ "$START_RVIZ" = 1 ]; then
  echo "[RViz] 已用配置启动：Fixed Frame=$WORLD，显示 /cloud_pcd"
  $RVZ_CMD >/dev/null 2>&1 &
  RVZ_PID=$!
fi
echo "  RViz 里: Global Options → Fixed Frame = $WORLD"
echo "           Add → By topic → /cloud_pcd"
echo "  Ctrl-C 结束"
wait
