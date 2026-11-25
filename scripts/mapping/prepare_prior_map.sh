#!/usr/bin/env bash
# =============================================================
# 一个名字走完「点云 → 地图系 → 摆正 → 先验地图包」——只写一次地图名。
#
# 之前要敲三遍名字（--pcd maps/pcd/<名字>.pcd、--map maps/<名字>/<名字>.yaml、--name <名字>），
# 本脚本按仓库约定从名字推出全部路径:
#   maps/pcd/<名字>.pcd                 ← 输入（实车 --save-pcd 或 rebuild_map_from_bag.sh 的产物）
#   maps/<名字>/<名字>.yaml+pgm         ← 输入（--save 存的 2D 图）
#   maps/pcd/<名字>_mapframe.pcd        ← 产物 1
#   maps/pcd/<名字>_mapframe_leveled.pcd← 产物 2
#   maps/pcd/map_bundles/<名字>/        ← 产物 3（先验地图包）
#
# 用法
# ====
#   ./scripts/mapping/prepare_prior_map.sh --name 227to219_1125
#   ./scripts/mapping/prepare_prior_map.sh --name 227to219_1125 --pcd maps/pcd/别处.pcd
#   ./scripts/mapping/prepare_prior_map.sh --name 227to219_1125 --no-bundle   # 只做到 leveled
#   ./scripts/mapping/prepare_prior_map.sh --name 227to219_1125 -n            # 只打印
#
# 参数
# ====
#       --name <名字>          地图名（唯一必填）
#       --pcd <路径>           覆盖输入点云（默认 maps/pcd/<名字>.pcd）
#       --map <路径>           覆盖地图 yaml（默认 maps/<名字>/<名字>.yaml）
#       --max-residual <米>    传给 pcd_to_mapframe 的对齐门限（默认让它按点距自动定量）
#       --sensor-z <米>        雷达离地高度（默认 0.2966，与实车 profile 的
#                              scan_context.lidar_height 一致；改了必须同步改那张 profile）
#       --skip-level           跳过地面摆正（zlio 落盘的云地面已在 z≈0 时用）
#       --no-bundle            只做 mapframe + leveled，不建先验包
#       --force                覆盖已存在的产物
#   -n, --check                只打印将执行的命令
#   -h, --help                 显示本帮助
#
# 说明
# ====
#   - 四步都沿用仓库原有脚本，本文件只负责「按名字推路径 + 串起来 + 少敲字」；
#     不改它们的行为。
#   - 第 2 步 level_map_pcd 把地面摆到 map z=0，第 3 步建包用的 --sensor-z 就是按这个口径；
#     跳过第 2 步会让描述子的 lidar_height 与运行期对不上（backend 启动会 FATAL）。
#   - 对齐失败（中位残差超门限）时脚本直接停，不会产出"看着像成功"的错文件。
# =============================================================

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
BACKEND_CONFIG="$WS_DIR/src/localization/backend/srm27_zlocalization_backend/config/srm27_localization_real.yaml"

NAME=""
PCD_IN=""
MAP_YAML=""
MAX_RESIDUAL=""
SENSOR_Z="0.2966"
DO_BUNDLE="1"
SKIP_LEVEL="0"
FORCE="0"
DRY_RUN="0"

usage() {
  awk 'NR == 1 { next }
       /^# =+[[:space:]]*$/ { sep++; if (sep == 2) exit; next }
       { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}
die() { echo "[错误] $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    --name)         NAME="${2:?--name 需要名字}"; shift 2 ;;
    --pcd)          PCD_IN="${2:?--pcd 需要路径}"; shift 2 ;;
    --map)          MAP_YAML="${2:?--map 需要路径}"; shift 2 ;;
    --max-residual) MAX_RESIDUAL="${2:?}"; shift 2 ;;
    --sensor-z)     SENSOR_Z="${2:?}"; shift 2 ;;
    --skip-level)   SKIP_LEVEL="1"; shift ;;
    --no-bundle)    DO_BUNDLE="0"; shift ;;
    --force)        FORCE="1"; shift ;;
    -n|--check)     DRY_RUN="1"; shift ;;
    -h|--help)      usage; exit 0 ;;
    *) echo "[错误] 未知参数: $1 (用 -h 查看用法)" >&2; exit 2 ;;
  esac
done

[ -n "$NAME" ] || die "需要 --name <地图名>（别的路径都从它推）"
case "$NAME" in
  *[!A-Za-z0-9_.-]*|'') die "--name 只允许字母/数字/下划线/点/短横线: $NAME" ;;
esac

[ -n "$PCD_IN" ] || PCD_IN="$WS_DIR/maps/pcd/$NAME.pcd"
[ -n "$MAP_YAML" ] || MAP_YAML="$WS_DIR/maps/$NAME/$NAME.yaml"
PCD_MAPFRAME="$WS_DIR/maps/pcd/${NAME}_mapframe.pcd"
PCD_LEVELED="$WS_DIR/maps/pcd/${NAME}_mapframe_leveled.pcd"
BUNDLE_DIR="$WS_DIR/maps/pcd/map_bundles/$NAME"

[ -f "$PCD_IN" ] || die "输入点云不存在: $PCD_IN（先用 --save-pcd 或 rebuild_map_from_bag.sh 产出）"
[ -f "$MAP_YAML" ] || die "地图 yaml 不存在: $MAP_YAML（先 --save $NAME）"
[ -f "$BACKEND_CONFIG" ] || die "实车 profile 不存在: $BACKEND_CONFIG"

if [ "$FORCE" != "1" ]; then
  for f in "$PCD_MAPFRAME" "$PCD_LEVELED"; do
    [ -e "$f" ] && die "已存在: $f（确认覆盖加 --force）"
  done
  [ "$DO_BUNDLE" = "1" ] && [ -d "$BUNDLE_DIR" ] && die "先验包已存在: $BUNDLE_DIR（确认覆盖加 --force）"
fi

MF_FLAG=()
[ -n "$MAX_RESIDUAL" ] && MF_FLAG=(--max-residual "$MAX_RESIDUAL")

echo "================ 一个名字建先验图 ================"
echo "  地图名    : $NAME"
echo "  输入点云  : $PCD_IN"
echo "  输入栅格  : $MAP_YAML"
echo "  产物 1    : $PCD_MAPFRAME"
echo "  产物 2    : $PCD_LEVELED"
echo "  先验包    : $([ "$DO_BUNDLE" = 1 ] && echo "$BUNDLE_DIR" || echo "（--no-bundle，跳过）")"
echo "  雷达高度  : ${SENSOR_Z} m（与实车 profile 的 scan_context.lidar_height 对齐）"
echo "================================================="

CMD1=(python3 "$SCRIPT_DIR/pcd_to_mapframe.py" --pcd "$PCD_IN" --map "$MAP_YAML")
[ ${#MF_FLAG[@]} -gt 0 ] && CMD1+=("${MF_FLAG[@]}")
CMD2=(python3 "$SCRIPT_DIR/level_map_pcd.py" --pcd "$PCD_MAPFRAME")
# CMD3 要等第 2 步确定用哪份点云 (摆正成功用 leveled, 否则退回 mapframe) 之后再拼

if [ "$DRY_RUN" = "1" ]; then
  echo
  echo "[--check] 将执行："
  printf '  1) %s\n' "$(printf '%q ' "${CMD1[@]}")"
  printf '  2) %s\n' "$(printf '%q ' "${CMD2[@]}")"
  if [ "$DO_BUNDLE" = "1" ]; then
    printf '  3) %s --name %q --pcd %q --config %q --sensor-z %q%s\n' \
      "$SCRIPT_DIR/build_zloc_map_bundle.sh" "$NAME" "$PCD_LEVELED" "$BACKEND_CONFIG" "$SENSOR_Z" \
      "$([ "$FORCE" = "1" ] && echo ' --force')"
  fi
  exit 0
fi

echo
echo "[1/3] 转地图系（自动判断点云是 camera_init 还是已重力对齐）"
"${CMD1[@]}" || die "第 1 步失败：pcd_to_mapframe 拒绝输出（先看它打印的残差与提示）"

echo
echo "[2/3] 地面摆到 z=0"
LEVEL_FLAG=()
[ "$FORCE" = "1" ] && LEVEL_FLAG=(--force)
if [ "$SKIP_LEVEL" = "1" ]; then
  echo "      --skip-level: 跳过（假定地面已在 z≈0）"
else
  if ! "${CMD2[@]}" "${LEVEL_FLAG[@]}"; then
    echo "[提示] level_map_pcd 失败（多半是点云里地面点太少: zlio 的 0.1 m 体素云常见）"
    echo "[提示] 用 RANSAC 复核地面到底平不平、在不在 z≈0 ..."
    if python3 - "$PCD_MAPFRAME" <<'PY'
import sys
import numpy as np

path = sys.argv[1]
with open(path, 'rb') as f:
    hdr = {}
    while True:
        line = f.readline().decode('ascii', 'replace').strip()
        if not line or line.startswith('#'):
            continue
        k, _, v = line.partition(' ')
        hdr[k.upper()] = v.strip()
        if k.upper() == 'DATA':
            break
    payload = f.read()
fields = hdr['FIELDS'].split()
sizes = [int(x) for x in hdr['SIZE'].split()]
types = hdr['TYPE'].split()
dt = {'F': {4: 'f4', 8: 'f8'}, 'U': {1: 'u1'}, 'I': {1: 'i1', 2: 'i2', 4: 'i4'}}
arr = np.frombuffer(payload,
                    dtype=np.dtype([(f, '<' + dt[t][s]) for f, t, s in zip(fields, types, sizes)]),
                    count=int(hdr['POINTS']))
P = np.stack([arr['x'], arr['y'], arr['z']], 1).astype(np.float64)
rng = np.random.default_rng(0)
S = P[rng.choice(len(P), min(20000, len(P)), replace=False)]
best = None
for _ in range(400):
    p0, p1, p2 = S[rng.choice(len(S), 3, replace=False)]
    n = np.cross(p1 - p0, p2 - p0)
    norm = np.linalg.norm(n)
    if norm < 1e-9:
        continue
    n /= norm
    if n[2] < 0:
        n = -n
    if n[2] < 0.9:
        continue
    d = -n @ p0
    dist = S @ n + d
    inl = np.abs(dist) < 0.06
    c = int(inl.sum())
    if c < 300 or (dist >= 0).mean() < 0.8:
        continue
    if best is None or c > best[0]:
        best = (c, n, d)
if best is None:
    print('  没找到地面平面')
    raise SystemExit(1)
c, n, d = best
inl = np.abs(P @ n + d) < 0.06
A = np.stack([P[inl, 0], P[inl, 1], np.ones(inl.sum())], 1)
coef, *_ = np.linalg.lstsq(A, P[inl, 2], rcond=None)
import math
tilt = math.degrees(math.atan(math.hypot(coef[0], coef[1])))
print('  地面: z(0,0)=%+.3f m, 倾斜 %.2f°, 内点 %.1f%%' % (coef[2], tilt, 100 * c / len(S)))
raise SystemExit(0 if (abs(coef[2]) < 0.05 and tilt < 1.5) else 1)
PY
    then
      echo "[提示] 地面已经在 z≈0 附近，跳过摆正，直接用 mapframe 那份建包"
      PCD_LEVELED="$PCD_MAPFRAME"
    else
      die "第 2 步失败：level_map_pcd 摆不正这份点云（地面点太少）。换更密的 pcd_voxel_leaf 重新落盘，或 --skip-level"
    fi
  fi
fi

if [ "$DO_BUNDLE" = "1" ]; then
  echo
  echo "[3/3] 建先验地图包（点云: $PCD_LEVELED）"
  BUNDLE_FLAG=()
  [ "$FORCE" = "1" ] && BUNDLE_FLAG=(--force)
  "$SCRIPT_DIR/build_zloc_map_bundle.sh" --name "$NAME" --pcd "$PCD_LEVELED" \
    --config "$BACKEND_CONFIG" --sensor-z "$SENSOR_Z" "${BUNDLE_FLAG[@]}" \
    || die "第 3 步失败：build_zloc_map_bundle"
fi

cat <<EOF

完成。地图名只写了一次：$NAME
  $PCD_MAPFRAME
  $PCD_LEVELED
$([ "$DO_BUNDLE" = "1" ] && echo "  $BUNDLE_DIR/")

验收（两条走廊的墙-墙中位都要 <0.2 m）：
  python3 scripts/mapping/verify_map_vs_pcd.py --yaml $MAP_YAML --pcd $PCD_LEVELED

起导航（先验包按 -m 自动找）：
  ./scripts/bringup/start_real_nav.sh -m $NAME --lio --lio-frontend zlio --zloc-backend --zloc-prior
EOF
