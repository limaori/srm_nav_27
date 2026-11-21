#!/usr/bin/env bash
# =============================================================
# 从地图系点云 maps/pcd/<名字>_mapframe.pcd 生成本仓库 zloc backend 的**先验地图包**
#
#   maps/pcd/map_bundles/<名字>/
#     keyframes.txt        逐帧位姿 (index timestamp tx ty tz qx qy qz qw)
#     clouds/NNNNNN.pcd    逐帧点云 (传感器系)
#     sc_descriptors.bin   Scan Context 描述子 (全局检索/回环)
#     map.pcd              全局显示云 (只给 RViz 看)
#
# 为什么需要这一步:
#   start_real_nav.sh --zloc-backend --zloc-prior 的 backend **不吃单个 pcd**: 它要的是
#   "关键帧地图包"(逐帧位姿 + 逐帧点云 + 描述子), 由离线工具
#   srm27_zlocalization_backend_build_pcd_map_bundle 从**地图系**点云生成。
#   输入必须是 <名字>_mapframe.pcd —— camera_init 系的 <名字>.pcd 不能用: 两者差一个
#   固定的雷达安装位姿, 用它建包会让车被定位到错姿态, 而且 GICP/backend 都不会报错。
#
# 用法:
#   ./scripts/mapping/build_zloc_map_bundle.sh --name 227_1113pro
#   ./scripts/mapping/build_zloc_map_bundle.sh --name 227_1113pro --crop-inset 0.15
#   ./scripts/mapping/build_zloc_map_bundle.sh --name 227_1113pro \
#       --x-min -8 --x-max 4.5 --y-min -7.6 --y-max 10.1
#   ./scripts/mapping/build_zloc_map_bundle.sh --name 227_1113pro --force
#   ./scripts/mapping/build_zloc_map_bundle.sh --name 227_1113pro -n
#
# 参数:
#       --name <名字>            地图名; 输入 maps/pcd/<名字>_mapframe.pcd,
#                                输出 maps/pcd/map_bundles/<名字>
#       --pcd <路径>             输入地图系点云 (默认 maps/pcd/<名字>_mapframe.pcd)
#       --out-bundle <绝对路径>  输出地图包目录 (默认 <工作空间>/maps/pcd/map_bundles/<名字>)
#       --config <路径>          backend 配置; --sc-* 几何参数从这里读, 保证与运行期一致
#                                (默认 config/backend.yaml)
#       --crop-inset <米>        裁掉点云边缘的宽度 (默认 0.1, 给 0 用原始包围盒)
#       --x-min/--x-max/--y-min/--y-max <米>
#                                显式指定裁剪范围 (覆盖自动推导; 给了就完全按你给的)
#       --sensor-z <米>          传感器离地高度 (默认 = config 的 scan_context.lidar_height)
#       --query-step <米>        位姿网格步长 (默认 2.0)
#       --bundle-step <米>       关键帧抽稀步长 (默认 1.5, 不能大于 query-step)
#       --map-voxel <米>         全局显示云体素 (默认 0.15)
#       --scan-voxel <米>        配准/描述子输入体素 (默认 0.15)
#       --force                  已存在同名地图包时覆盖 (先建到临时目录再替换, 不会半残)
#   -n, --check                  只打印将执行的命令, 不建任何东西
#   -h, --help                   显示本帮助
#
# 环境变量 (同名参数均可由环境变量给默认值):
#   MAP_NAME ZLOC_PCD_IN ZLOC_BUNDLE_OUT ZLOC_BACKEND_CONFIG CROP_INSET
#   ZLOC_SENSOR_Z QUERY_STEP BUNDLE_STEP MAP_VOXEL SCAN_VOXEL FORCE_BUNDLE
#   ZLOC_BUNDLE_ROOT SRM27_WS_DIR DRY_RUN
#
# 说明:
#   - 建包参数**必须与 config/backend.yaml 的 common.scan_context.* 逐位一致**
#     (num_rings/num_sectors/max_radius/downsample/lidar_height/min_bin_points/augment_count)。
#     storage.cpp 按 1e-6 比较几何配置, 不一致时 backend 启动直接 FATAL
#     "failed to load Scan Context descriptors"。本脚本自动从配置里读这几个值再传,
#     所以换配置、换车都不用改命令行。
#   - 裁剪范围默认取点云包围盒向内缩 --crop-inset: 位姿网格只铺在有点的地方, 边上
#     没点的区域铺出来的关键帧只会往包里塞空帧。
#   - 地图包是"按场地生成的大二进制", 已在 .gitignore 里 (/maps/pcd/map_bundles/),
#     默认不入库; 要版本化请走 Release 附件或 Git LFS。
#   - 这份包只服务 backend 先验定位 (--zloc-backend --zloc-prior)。
#     small_gicp 重定位 (--reloc) 用的是 maps/pcd/<名字>_mapframe.pcd, 不需要地图包。
# =============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS_DIR="${SRM27_WS_DIR:-$(cd "$SCRIPT_DIR/../.." && pwd)}"

# ---------- 环境消毒 ----------
# 海康 MVS 客户端前置进 LD_LIBRARY_PATH 的 /opt/MVS/lib/64 带老 libusb, 缺
# libusb_set_option, 会让链接 PCL 的程序以 exit 127 静默退出。只影响本脚本及其子进程。
. "$SCRIPT_DIR/../lib/srm_lib_path_guard.sh"

SETUP_FILE="$WS_DIR/install/setup.bash"
BUNDLE_ROOT="${ZLOC_BUNDLE_ROOT:-$WS_DIR/maps/pcd/map_bundles}"

# 配置: 源码包优先, 退到 install/share (两者通常是软链接, 内容同一份)。
CONFIG_DEFAULT="$WS_DIR/src/localization/backend/srm27_zlocalization_backend/config/backend.yaml"
[ -f "$CONFIG_DEFAULT" ] || \
  CONFIG_DEFAULT="$WS_DIR/install/srm27_zlocalization_backend/share/srm27_zlocalization_backend/config/backend.yaml"

# ---------- 可配置项 ----------
MAP_NAME="${MAP_NAME:-}"
ZLOC_PCD_IN="${ZLOC_PCD_IN:-}"
ZLOC_BUNDLE_OUT="${ZLOC_BUNDLE_OUT:-}"
ZLOC_BACKEND_CONFIG="${ZLOC_BACKEND_CONFIG:-$CONFIG_DEFAULT}"
CROP_INSET="${CROP_INSET:-0.1}"
# 空 = 自动: 先按配置里的 lidar_height, 读不到再退回 0.3464 (URDF 里 livox_frame 离地高度)
ZLOC_SENSOR_Z="${ZLOC_SENSOR_Z:-}"
QUERY_STEP="${QUERY_STEP:-2.0}"
BUNDLE_STEP="${BUNDLE_STEP:-1.5}"
MAP_VOXEL="${MAP_VOXEL:-0.15}"
SCAN_VOXEL="${SCAN_VOXEL:-0.15}"
FORCE_BUNDLE="${FORCE_BUNDLE:-0}"
DRY_RUN="${DRY_RUN:-0}"
# 裁剪范围: 空 = 按点云包围盒自动推导; 给了任意一个就四个都要给。
X_MIN=""; X_MAX=""; Y_MIN=""; Y_MAX=""

TOOL="srm27_zlocalization_backend_build_pcd_map_bundle"

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

die() { echo "[错误] $*" >&2; exit 1; }

# ---------- 命令行参数 ----------
while [ $# -gt 0 ]; do
  case "$1" in
    --name)          MAP_NAME="${2:?--name 需要地图名}"; shift 2 ;;
    --pcd)           ZLOC_PCD_IN="${2:?--pcd 需要路径}"; shift 2 ;;
    --out-bundle)    ZLOC_BUNDLE_OUT="${2:?--out-bundle 需要路径}"; shift 2 ;;
    --config)        ZLOC_BACKEND_CONFIG="${2:?--config 需要路径}"; shift 2 ;;
    --crop-inset)    CROP_INSET="${2:?--crop-inset 需要米数}"; shift 2 ;;
    --x-min)         X_MIN="${2:?--x-min 需要米数}"; shift 2 ;;
    --x-max)         X_MAX="${2:?--x-max 需要米数}"; shift 2 ;;
    --y-min)         Y_MIN="${2:?--y-min 需要米数}"; shift 2 ;;
    --y-max)         Y_MAX="${2:?--y-max 需要米数}"; shift 2 ;;
    --sensor-z)      ZLOC_SENSOR_Z="${2:?--sensor-z 需要米数}"; shift 2 ;;
    --query-step)    QUERY_STEP="${2:?--query-step 需要米数}"; shift 2 ;;
    --bundle-step)   BUNDLE_STEP="${2:?--bundle-step 需要米数}"; shift 2 ;;
    --map-voxel)     MAP_VOXEL="${2:?--map-voxel 需要米数}"; shift 2 ;;
    --scan-voxel)    SCAN_VOXEL="${2:?--scan-voxel 需要米数}"; shift 2 ;;
    --force)         FORCE_BUNDLE=1; shift ;;
    -n|--check)      DRY_RUN=1; shift ;;
    -h|--help)       usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 -h 查看用法)" >&2; exit 2 ;;
  esac
done

# ---------- 校验 ----------
if [ -z "$MAP_NAME" ] && [ -z "$ZLOC_PCD_IN" ]; then
  die "需要 --name <地图名> (或用 --pcd 直接给点云, 名字从文件名推)。"
fi

# --name 结尾的 _mapframe 是多余的: 脚本自己加。
if [ -n "$MAP_NAME" ] && [ "$MAP_NAME" != "${MAP_NAME%_mapframe}" ]; then
  echo "[提示] --name 结尾的 _mapframe 是多余的, 按 ${MAP_NAME%_mapframe} 处理。" >&2
  MAP_NAME="${MAP_NAME%_mapframe}"
fi

if [ -z "$ZLOC_PCD_IN" ]; then
  ZLOC_PCD_IN="$WS_DIR/maps/pcd/${MAP_NAME}_mapframe.pcd"
fi
if [ -z "$MAP_NAME" ]; then
  base="$(basename "$ZLOC_PCD_IN" .pcd)"
  MAP_NAME="${base%_mapframe}"
fi
if [ -z "$ZLOC_BUNDLE_OUT" ]; then
  ZLOC_BUNDLE_OUT="$BUNDLE_ROOT/$MAP_NAME"
fi
case "$ZLOC_BUNDLE_OUT" in
  /*) ;;
  *) die "--out-bundle 必须是绝对路径: $ZLOC_BUNDLE_OUT" ;;
esac

[ -f "$ZLOC_PCD_IN" ] || {
  echo "[错误] 输入点云不存在: $ZLOC_PCD_IN" >&2
  echo "[提示] 先跑: ./scripts/mapping/build_pcd_from_bag.sh --name $MAP_NAME" >&2
  exit 1
}
# camera_init 系的原始件混进来是最常见的错: 建出来的包看着正常, 车却定位到错姿态。
case "$(basename "$ZLOC_PCD_IN")" in
  *_mapframe.pcd) ;;
  *)
    echo "[警告] 输入不是 *_mapframe.pcd: $ZLOC_PCD_IN" >&2
    echo "[警告] camera_init 系的点云建包会让车被定位到错姿态, 而且不会报错。" >&2
    echo "[警告] 确认它已经在地图系下再继续 (--no-mapframe 生成的原始件不能直接用)。" >&2
    ;;
esac

if [ ! -f "$ZLOC_BACKEND_CONFIG" ]; then
  die "backend 配置不存在: $ZLOC_BACKEND_CONFIG"
fi

# 地图 yaml 不在只告警: 建包不读它, 但两者必须是同一张图, 否则定位对不上。
if [ ! -f "$WS_DIR/maps/$MAP_NAME/$MAP_NAME.yaml" ]; then
  echo "[警告] 没找到 2D 地图 maps/$MAP_NAME/$MAP_NAME.yaml。" >&2
  echo "[警告] 地图包要与导航加载的栅格图同一坐标系, 请确认这对名字是一致的。" >&2
fi

case "$CROP_INSET" in
  ''|*[!0-9.-]*) die "--crop-inset 必须是数字, 收到: $CROP_INSET" ;;
esac

if [ -e "$ZLOC_BUNDLE_OUT" ] && ! is_true "$FORCE_BUNDLE"; then
  echo "[错误] 地图包已存在: $ZLOC_BUNDLE_OUT" >&2
  echo "[提示] 要重建成另一份就用 --out-bundle 换个目录; 确认覆盖加 --force。" >&2
  exit 1
fi

# ---------- 从配置里取 sc_* 几何参数 ----------
# 这几个值必须与运行期 backend 读到的**逐位一致**, 否则启动即 FATAL。与其让调用方手抄,
# 不如每次从同一份配置里读。
SC_FIELDS="$(python3 - "$ZLOC_BACKEND_CONFIG" <<'PY' 2>/dev/null || true
import sys
try:
    import yaml
except Exception:
    sys.exit(1)
try:
    with open(sys.argv[1], encoding="utf-8") as stream:
        doc = yaml.safe_load(stream) or {}
except Exception:
    sys.exit(1)
sc = ((doc.get("common") or {}).get("scan_context") or {})

def num(key, default):
    value = sc.get(key, default)
    if isinstance(value, bool):
        return default
    try:
        return float(value)
    except Exception:
        return float(default)

# rings/sectors/min_bin_points/augment_count 必须是整数, 其余是浮点
print(int(num("num_rings", 30)), int(num("num_sectors", 90)),
      num("max_radius", 80.0), num("downsample", 0.6), num("lidar_height", 0.3464),
      int(num("min_bin_points", 1)), int(num("augment_count", 3)))
PY
)"

if [ -n "$SC_FIELDS" ]; then
  read -r SC_RINGS SC_SECTORS SC_MAX_RADIUS SC_DOWNSAMPLE SC_LIDAR_HEIGHT SC_MIN_BIN SC_AUGMENT <<<"$SC_FIELDS"
  SC_SOURCE="$ZLOC_BACKEND_CONFIG"
else
  # 配置读不出来 (没装 pyyaml / 格式变了) 时退回文档里的基线值, 并明确告警。
  SC_RINGS=30; SC_SECTORS=90; SC_MAX_RADIUS=80.0; SC_DOWNSAMPLE=0.6
  SC_LIDAR_HEIGHT=0.3464; SC_MIN_BIN=1; SC_AUGMENT=3
  SC_SOURCE="内置基线 (配置解析失败)"
  echo "[警告] 未能从 $ZLOC_BACKEND_CONFIG 读出 scan_context 参数, 用内置基线值。" >&2
  echo "[警告] 若与实际配置不一致, backend 启动会 FATAL: failed to load Scan Context descriptors。" >&2
fi

if [ -z "$ZLOC_SENSOR_Z" ]; then
  ZLOC_SENSOR_Z="$SC_LIDAR_HEIGHT"
fi

# ---------- 裁剪范围: 没给就按点云包围盒推导 ----------
BOUNDS_AUTO=0
if [ -z "$X_MIN" ] || [ -z "$X_MAX" ] || [ -z "$Y_MIN" ] || [ -z "$Y_MAX" ]; then
  if [ -n "$X_MIN$X_MAX$Y_MIN$Y_MAX" ]; then
    die "--x-min/--x-max/--y-min/--y-max 必须四个一起给 (只给了一部分)。"
  fi
  BOUNDS="$(python3 - "$ZLOC_PCD_IN" <<'PY'
import sys

path = sys.argv[1]
with open(path, "rb") as stream:
    header = {}
    while True:
        raw = stream.readline()
        if not raw:
            raise SystemExit("bad pcd: no DATA line")
        line = raw.decode("ascii", "replace").strip()
        if not line or line.startswith("#"):
            continue
        key, _, value = line.partition(" ")
        header[key.upper()] = value.strip()
        if key.upper() == "DATA":
            break
    payload = stream.read()

fields = header["FIELDS"].split()
count = int(header["POINTS"])
mode = header["DATA"].lower()
try:
    ix, iy = fields.index("x"), fields.index("y")
except ValueError:
    raise SystemExit("bad pcd: no x/y fields")
width = len(fields)

xs, ys = [], []
if mode == "ascii":
    for raw in payload.splitlines():
        parts = raw.split()
        if len(parts) < width:
            continue
        xs.append(float(parts[ix])); ys.append(float(parts[iy]))
elif mode == "binary":
    import struct
    if "SIZE" not in header:
        raise SystemExit("bad pcd: no SIZE")
    sizes = [int(v) for v in header["SIZE"].split()]
    types = header["TYPE"].split()
    if len(sizes) != width or len(types) != width:
        raise SystemExit("bad pcd: field layout mismatch")
    stride = sum(sizes)
    fmt = {("F", 4): "f", ("F", 8): "d", ("U", 1): "B", ("U", 2): "H",
           ("U", 4): "I", ("I", 1): "b", ("I", 2): "h", ("I", 4): "i"}
    for index in (ix, iy):
        key = (types[index], sizes[index])
        if key not in fmt:
            raise SystemExit("unsupported field type")
    import array
    for point in range(count):
        base = point * stride
        for index, bucket in ((ix, xs), (iy, ys)):
            off = base + sum(sizes[:index])
            key = (types[index], sizes[index])
            bucket.append(struct.unpack_from("<" + fmt[key], payload, off)[0])
else:
    raise SystemExit("unsupported pcd DATA mode: %s" % mode)

if not xs:
    raise SystemExit("empty cloud")
print("%.4f %.4f %.4f %.4f" % (min(xs), max(xs), min(ys), max(ys)))
PY
)" || {
    echo "[错误] 读取点云范围失败: $ZLOC_PCD_IN" >&2
    echo "[提示] 用 --x-min/--x-max/--y-min/--y-max 显式给裁剪范围。" >&2
    exit 1
  }
  read -r PX_MIN PX_MAX PY_MIN PY_MAX <<<"$BOUNDS"
  RAW_BOUNDS="$BOUNDS"
  X_MIN="$(awk -v v="$PX_MIN" -v i="$CROP_INSET" 'BEGIN{printf "%.2f", v + i}')"
  X_MAX="$(awk -v v="$PX_MAX" -v i="$CROP_INSET" 'BEGIN{printf "%.2f", v - i}')"
  Y_MIN="$(awk -v v="$PY_MIN" -v i="$CROP_INSET" 'BEGIN{printf "%.2f", v + i}')"
  Y_MAX="$(awk -v v="$PY_MAX" -v i="$CROP_INSET" 'BEGIN{printf "%.2f", v - i}')"
  BOUNDS_AUTO=1
  # 缩过头 (场地比 inset 还小) 时退回原始包围盒, 别把整个场地裁没。
  if awk -v a="$X_MIN" -v b="$X_MAX" -v c="$Y_MIN" -v d="$Y_MAX" 'BEGIN{exit !(a<b && c<d)}'; then :; else
    echo "[警告] --crop-inset $CROP_INSET 把裁剪范围缩没了, 改用原始包围盒。" >&2
    read -r X_MIN X_MAX Y_MIN Y_MAX <<<"$PX_MIN $PX_MAX $PY_MIN $PY_MAX"
  fi
fi

[ -z "$X_MIN" ] && die "裁剪范围解析失败。"

# ---------- 组装命令 ----------
STAGE="$ZLOC_BUNDLE_OUT.staging.$$"
CMD=( "$TOOL"
  --input "$ZLOC_PCD_IN"
  --out-bundle "$STAGE"
  --x-min "$X_MIN" --x-max "$X_MAX" --y-min "$Y_MIN" --y-max "$Y_MAX"
  --sensor-z "$ZLOC_SENSOR_Z"
  --query-step "$QUERY_STEP" --bundle-step "$BUNDLE_STEP"
  --map-voxel "$MAP_VOXEL" --scan-voxel "$SCAN_VOXEL"
  --descriptor-type scan_context
  --sc-rings "$SC_RINGS" --sc-sectors "$SC_SECTORS"
  --sc-max-radius "$SC_MAX_RADIUS" --sc-downsample "$SC_DOWNSAMPLE"
  --sc-lidar-height "$SC_LIDAR_HEIGHT"
  --sc-min-bin-points "$SC_MIN_BIN" --sc-augment-count "$SC_AUGMENT"
)

# ---------- 启动摘要 ----------
echo "================= zloc backend 先验地图包 ================="
echo "  地图名      : $MAP_NAME"
echo "  输入点云    : $ZLOC_PCD_IN ($(du -h "$ZLOC_PCD_IN" 2>/dev/null | cut -f1))"
echo "  输出地图包  : $ZLOC_BUNDLE_OUT"
echo "  裁剪范围    : x[$X_MIN, $X_MAX]  y[$Y_MIN, $Y_MAX]$( [ "$BOUNDS_AUTO" = "1" ] && echo "  (由包围盒 x[$PX_MIN,$PX_MAX] y[$PY_MIN,$PY_MAX] 内缩 $CROP_INSET m)" || echo "  (命令行指定)")"
echo "  传感器高度  : $ZLOC_SENSOR_Z m"
echo "  位姿网格    : query-step $QUERY_STEP / bundle-step $BUNDLE_STEP"
echo "  体素        : map $MAP_VOXEL / scan $SCAN_VOXEL"
echo "  ScanContext : rings $SC_RINGS, sectors $SC_SECTORS, max_radius ${SC_MAX_RADIUS} m, downsample $SC_DOWNSAMPLE m"
echo "                来源: $SC_SOURCE"
echo "=========================================================="

if [ "$DRY_RUN" = "1" ]; then
  echo
  echo "[--check] 仅解析, 不建包。将要执行的命令:"
  printf '  %s\n' "$(printf '%q ' "${CMD[@]}")"
  echo
  echo "解析完成, 未生成任何文件。"
  exit 0
fi

# ---------- 建包 ----------
# 工具拒绝非空输出目录; 先建到临时目录, 成功后再替换, 中途失败不会留下半份包。
cleanup() { rm -rf "$STAGE"; }
trap cleanup EXIT

mkdir -p "$(dirname "$ZLOC_BUNDLE_OUT")"
rm -rf "$STAGE"

echo "[1/3] 生成地图包 (临时目录 $STAGE)..."
(
  set +u
  # shellcheck disable=SC1091
  source "$SETUP_FILE" >/dev/null 2>&1
  ros2 run srm27_zlocalization_backend "$TOOL" "${CMD[@]:1}"
)

if [ ! -f "$STAGE/keyframes.txt" ]; then
  echo "[错误] 建包失败: 没有生成 keyframes.txt" >&2
  exit 1
fi

# ---------- 自检 ----------
echo "[2/3] 自检..."
KF_COUNT="$(awk '!/^[[:space:]]*(#|$)/ { n++ } END { print n + 0 }' "$STAGE/keyframes.txt")"
if [ "$KF_COUNT" -le 0 ]; then
  echo "[错误] keyframes.txt 里没有关键帧。" >&2
  exit 1
fi
if [ ! -f "$STAGE/sc_descriptors.bin" ]; then
  echo "[错误] 没有生成 sc_descriptors.bin (Scan Context 关闭了?)" >&2
  exit 1
fi
CLOUD_COUNT="$(find "$STAGE/clouds" -name '*.pcd' -type f 2>/dev/null | wc -l)"
if [ "$CLOUD_COUNT" -ne "$KF_COUNT" ]; then
  echo "[警告] 关键帧 $KF_COUNT 个, 但点云 $CLOUD_COUNT 个 —— backend 会按索引找 clouds/, 请核对。" >&2
fi

echo "[3/3] 替换 $ZLOC_BUNDLE_OUT ..."
rm -rf "$ZLOC_BUNDLE_OUT"
mv "$STAGE" "$ZLOC_BUNDLE_OUT"
trap - EXIT

cat <<EOF

建包完成。
  $ZLOC_BUNDLE_OUT
    keyframes.txt        $KF_COUNT 帧
    clouds/              $CLOUD_COUNT 个点云 ($(du -sh "$ZLOC_BUNDLE_OUT/clouds" 2>/dev/null | cut -f1))
    sc_descriptors.bin   $(du -h "$ZLOC_BUNDLE_OUT/sc_descriptors.bin" 2>/dev/null | cut -f1)
    map.pcd              $(du -h "$ZLOC_BUNDLE_OUT/map.pcd" 2>/dev/null | cut -f1)
  总计 $(du -sh "$ZLOC_BUNDLE_OUT" 2>/dev/null | cut -f1)

用它跑先验定位 (地图包按名字自动解析, 不用再给路径):
  ./scripts/bringup/start_real_nav.sh -m $MAP_NAME \\
    --lio --lio-frontend zlio --zloc-backend --zloc-prior

只想先确认 backend 能加载 (约 10 s, 之后 Ctrl-C):
  source scripts/setup/env_srm27.sh
  ros2 launch srm27_zlocalization_backend backend.launch.py mode:=localization \\
    prior_map:=$ZLOC_BUNDLE_OUT use_rviz:=false
  看到 "loaded immutable prior: keyframes=$KF_COUNT, descriptor=Scan Context" 即正常。
EOF
