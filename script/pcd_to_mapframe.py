#!/usr/bin/env python3
"""把 Point-LIO 的 camera_init 系点云转到栅格地图系, 产出 maps/<名字>_mapframe.pcd。

为什么需要它
------------
Point-LIO 落盘的 scans.pcd (以及 script/build_pcd_from_bag.sh 产出的 maps/pcd/<名字>.pcd)
是以 **camera_init** (建图起点雷达系) 表达的; 而 Nav2 的地图系是 pgm 栅格图的坐标系,
两者相差一个**固定的平面旋转**——雷达相对 base_link 的 yaw 不是 0(见 launch 的
lidar_rpy)。CAD 实测的安装角是 yaw = 1.099557429 rad = 63°(旧外参是 -90°),
所以换模型后这个偏移量必须重新由数据求出(本脚本本来就是搜索出来的, 不写死):
实测旧配置 227_1011 差 -90.00°。直接把这个点云喂给 --reloc, 车会被定位到差一个
yaw 的姿态上,
**而 GICP 自己不会报错**(见 config/real/nav2_params_srm.yaml 里 small_gicp_relocalization
那段的"情况一/情况二"警告)。

本工具离线把这个变换求出来、烘进点云, 输出**与输入同目录的 <名字>_mapframe.pcd**
(原始件不动, 两种点云靠名字区分系别), 头部按仓库既有约定(见 pcd/real/srm_site_01.pcd)
标注坐标系与所用变换。名字 + 头部注释是两道防线: 工具靠头部识破"已经转过了", 防止再转一次。

怎么求变换
----------
不靠猜: 把点云里"墙"那一层(z 高度带)投影到 XY, 与 pgm 的占据栅格做最近邻距离评分,
先在 (yaw, dx, dy) 上粗搜, 再在最优点附近细化, 取中位最近邻最小的解。
参照量级: 已对正的 srm_site_01.pcd 与 srm_site_01.pgm 是 0.026 m; 若最终残差远大于此,
说明不是"差个平面变换"那么简单(时间戳/外参/两张图不同源), 本工具会拒绝输出而不是给你一个
看着像成功的错文件。

用法::

    # 最常用: maps/pcd/227_1011.pcd -> maps/pcd/227_1011_mapframe.pcd
    python3 script/pcd_to_mapframe.py \
        --pcd maps/pcd/227_1011.pcd \
        --map maps/227_1011/227_1011.yaml

    # 只看能不能对齐, 不写文件
    python3 script/pcd_to_mapframe.py --pcd maps/pcd/227_1011.pcd \
        --map maps/227_1011/227_1011.yaml --dry-run

    # 已知变换时跳过搜索 (~13 秒); 或用 --out 指定别的位置
    python3 script/pcd_to_mapframe.py --pcd in.pcd --map map.yaml --out out.pcd \
        --no-search --yaw -90.0 --dx 0.250 --dy -0.100

原始件随时可由录包重建: ./script/build_pcd_from_bag.sh --name <名字> --force

退出码: 0 成功; 1 对齐残差过大/输入已转过/文件不可用; 2 参数错误。
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import time

import numpy as np
import yaml

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover
    print("[错误] 需要 scipy (sudo apt install python3-scipy)", file=sys.stderr)
    raise SystemExit(2)

# 判定门限 (米): 参照已对正的 srm_site_01 是 0.026 m。
GOOD_MEDIAN = 0.05      # 低于它算"对上了"
ACCEPT_MEDIAN = 0.10    # 高于它直接拒绝输出


def read_pgm(path: str) -> np.ndarray:
    """读 P5 pgm。注释行(#)在头里任意位置都可能出现, 按 token 解析最稳。"""
    data = open(path, "rb").read()
    tokens: list[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            while pos < len(data) and data[pos:pos + 1] != b"\n":
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b"P5":
        raise ValueError("只支持二进制 P5 pgm, 收到 %r" % magic)
    if maxval > 255:
        raise ValueError("只支持 8 位 pgm (maxval<=255), 收到 %d" % maxval)
    pos += 1  # maxval 之后恰好一个空白字符
    buf = np.frombuffer(data[pos:pos + width * height], dtype=np.uint8)
    if buf.size != width * height:
        raise ValueError("pgm 数据长度不足: 期望 %d 字节, 实际 %d" % (width * height, buf.size))
    return buf.reshape(height, width)


def load_grid(map_yaml: str):
    """读 maps/<名字>.yaml + 对应 pgm, 返回占据栅格的世界坐标 (N,2)、分辨率、图幅。"""
    cfg = yaml.safe_load(open(map_yaml, encoding="utf-8"))
    image = cfg["image"]
    pgm_path = image if os.path.isabs(image) else os.path.join(os.path.dirname(map_yaml), image)
    pix = read_pgm(pgm_path)
    height, width = pix.shape
    resolution = float(cfg["resolution"])
    origin_x, origin_y = float(cfg["origin"][0]), float(cfg["origin"][1])
    negate = int(cfg.get("negate", 0))
    # trinary: 0 = 占据(黑); negate=1 时反过来。中间值(通常 205)是未知格。
    occupied = (pix > 127) if negate else (pix < 100)
    rows, cols = np.nonzero(occupied)
    world_x = origin_x + (cols + 0.5) * resolution
    # pgm 第 0 行是最上面的像素, 对应最大的 y
    world_y = origin_y + (height - rows - 0.5) * resolution
    return np.stack([world_x, world_y], axis=1), resolution, (width, height), pgm_path


def read_pcd(path: str):
    """读 binary PCD, 返回 (全部字段的 (N, K) float32 数组, 头部字典, 数据区字节数)。"""
    raw = open(path, "rb").read()
    marker = b"DATA binary\n"
    if marker not in raw:
        raise ValueError("只支持 DATA binary 的 PCD (ASCII 版本请先用 pcl 转换)")
    end = raw.index(marker) + len(marker)
    header = raw[:end].decode("ascii", "replace")

    def field(name: str) -> str:
        m = re.search(r"^%s (.*)$" % name, header, re.M)
        if not m:
            raise ValueError("PCD 头部缺少 %s" % name)
        return m.group(1).strip()

    names = field("FIELDS").split()
    sizes = [int(v) for v in field("SIZE").split()]
    types = field("TYPE").split()
    counts = [int(v) for v in field("COUNT").split()]
    points = int(field("POINTS"))
    if any(t != "F" or s != 4 or c != 1 for t, s, c in zip(types, sizes, counts)):
        raise ValueError("只支持 float32 单值字段, 收到 TYPE=%s SIZE=%s COUNT=%s"
                         % (types, sizes, counts))
    body = np.frombuffer(raw[end:], dtype=np.float32)
    if body.size < points * len(names):
        raise ValueError("PCD 数据区不足: 期望 %d 个 float, 实际 %d"
                         % (points * len(names), body.size))
    arr = body[:points * len(names)].reshape(points, len(names)).copy()
    return arr, {"fields": names, "points": points, "header": header}, len(raw) - end


def planar_matrix(yaw_deg: float, dx: float, dy: float) -> np.ndarray:
    th = math.radians(yaw_deg)
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, dx], [s, c, dy]], dtype=np.float64)


def score(grid_tree: cKDTree, xy: np.ndarray, tf: np.ndarray) -> float:
    moved = xy @ tf[:, :2].T + tf[:, 2]
    return float(np.median(grid_tree.query(moved, k=1, workers=-1)[0]))


def inlier_ratio(grid_tree: cKDTree, xy: np.ndarray, tf: np.ndarray, tol: float) -> float:
    moved = xy @ tf[:, :2].T + tf[:, 2]
    d, _ = grid_tree.query(moved, k=1, workers=-1)
    return float((d < tol).mean())


def _median_at(tree: cKDTree, rot: np.ndarray, dx: float, dy: float) -> float:
    d, _ = tree.query(rot + (dx, dy), k=1, workers=-1)
    return float(np.median(d))


def _refine_xy(tree: cKDTree, rot: np.ndarray, dx: float, dy: float, steps):
    """坐标下降细化平移。

    必须连续细化, 不能只用固定网格: 评分面在 0.1 m 量级就非常陡 (实测同一 yaw 下
    (0.2,-0.1) 是 0.030 m, 挪到网格节点 (0.25,0.0) 就是 0.068 m)。峰顶一旦落在网格
    节点之间, 所有 yaw 的分数都被拉平到差不多的水平 -> yaw 分辨力被抹掉, 搜索会锁在
    错的 yaw 上并交出一个"看着能用、其实歪了 2°"的结果。
    """
    best = _median_at(tree, rot, dx, dy)
    for step in steps:
        improved = True
        while improved:
            improved = False
            for ox, oy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                v = _median_at(tree, rot, dx + ox * step, dy + oy * step)
                if v < best - 1e-9:
                    best, dx, dy = v, dx + ox * step, dy + oy * step
                    improved = True
    return best, dx, dy


def search_transform(grid: np.ndarray, sample: np.ndarray, yaw_range: float,
                     coarse_step: float, coarse_trans: float, verbose: bool = True):
    """三段式搜索: 粗(抽稀栅格 + 网格平移, 只用来找候选 yaw) -> 中(逐 yaw 连续细化平移)
    -> 细(yaw 也连续细化)。真实解可能离 identity 90° 以上, 所以粗段必须全 yaw 范围。
    """
    # --- 粗段: 目的只有一个, 找出 yaw 的几个候选盆地 (分数不准没关系) ---
    coarse_keys = np.floor(grid / 0.5).astype(np.int64)
    _, idx = np.unique(coarse_keys, axis=0, return_index=True)
    coarse_tree = cKDTree(grid[np.sort(idx)])
    coarse_sample = sample[::max(1, sample.shape[0] // 500)]

    t0 = time.time()
    results = []
    for yaw in np.arange(-yaw_range, yaw_range + 1e-9, coarse_step):
        tf = planar_matrix(yaw, 0.0, 0.0)
        rot = coarse_sample @ tf[:, :2].T
        for dx in np.arange(-coarse_trans, coarse_trans + 1e-9, 1.0):
            for dy in np.arange(-coarse_trans, coarse_trans + 1e-9, 1.0):
                results.append((_median_at(coarse_tree, rot, dx, dy),
                                float(dx), float(dy), float(yaw)))
    results.sort()
    if verbose:
        print("  粗搜索: 中位 %.4f m @ yaw=%.1f° dx=%.0f dy=%.0f  (%.0f 秒, 取前 %d 个 yaw 候选)"
              % (results[0][0], results[0][3], results[0][1], results[0][2],
                 time.time() - t0, 3))

    # --- 中段: 对候选 yaw 及其邻域 (粗步长内每 2°) 逐个把平移连续细化, 再比 yaw ---
    tree = cKDTree(grid)
    candidates = []
    for _, dx, dy, yaw in results[:3]:
        candidates.extend(
            (y, dx, dy) for y in np.arange(yaw - coarse_step, yaw + coarse_step + 1e-9, 2.0))
    best = None
    for yaw, dx, dy in candidates:
        tf = planar_matrix(yaw, 0.0, 0.0)
        rot = sample @ tf[:, :2].T
        v, rx, ry = _refine_xy(tree, rot, dx, dy, (0.5, 0.1, 0.02))
        if best is None or v < best[0]:
            best = (v, rx, ry, yaw)
    if verbose:
        print("  中搜索: 中位 %.4f m @ yaw=%.2f° dx=%.3f dy=%.3f  (%d 个 yaw 候选)"
              % (best[0], best[3], best[1], best[2], len(candidates)))

    # --- 细段: yaw 也连续细化 (0.5° -> 0.1°), 每一步都把平移重新优化 ---
    _, bx, by, byaw = best
    for step in (0.5, 0.1):
        improved = True
        while improved:
            improved = False
            for sign in (1, -1):
                yaw = byaw + sign * step
                tf = planar_matrix(yaw, 0.0, 0.0)
                rot = sample @ tf[:, :2].T
                v, rx, ry = _refine_xy(tree, rot, bx, by, (0.1, 0.02))
                if v < best[0] - 1e-9:
                    best = (v, rx, ry, yaw)
                    bx, by, byaw = rx, ry, yaw
                    improved = True
    if verbose:
        print("  精搜索: 中位 %.4f m @ yaw=%.2f° dx=%.3f dy=%.3f"
              % (best[0], best[3], best[1], best[2]))
    return best


def write_pcd_tmp(path: str, arr: np.ndarray, fields: list, note_lines: list) -> str:
    """写到 path 旁边的临时文件, 返回临时路径 (复核通过后由调用方原子改名)。

    不直接写 path: 就地转换时 path 可能就是唯一的原始件, 先毁了再发现写错就没救了。
    """
    count = arr.shape[0]
    lines = ["# .PCD v0.7 - Point Cloud Data file format"]
    lines += ["# " + n for n in note_lines]
    lines += [
        "VERSION 0.7",
        "FIELDS " + " ".join(fields),
        "SIZE " + " ".join(["4"] * len(fields)),
        "TYPE " + " ".join(["F"] * len(fields)),
        "COUNT " + " ".join(["1"] * len(fields)),
        "WIDTH %d" % count,
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        "POINTS %d" % count,
        "DATA binary",
    ]
    tmp = path + ".mapframe.tmp"
    with open(tmp, "wb") as stream:
        stream.write(("\n".join(lines) + "\n").encode("ascii"))
        stream.write(arr.astype(np.float32).tobytes())
    return tmp


def mounting_hint() -> str:
    """把 launch 里的挂载外参算成平面量, 供与实测值对照。"""
    xyz = (0.12081, -0.056721, 0.346388)
    rpy = (0.523598776, 0.0, 1.099557429)
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    r = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    return "挂载外参 lidar_xyz/rpy 的平面量: yaw %.2f°, 平移 (%.3f, %.3f) m" % (yaw, xyz[0], xyz[1])


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="把 camera_init 系点云就地转到栅格地图系 (每张图只留一份 maps/pcd/<名字>.pcd)",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pcd", required=True, help="输入点云 (camera_init 系), 默认就地覆盖它")
    parser.add_argument("--map", required=True, help="目标栅格图 yaml (maps/<名字>/<名字>.yaml)")
    parser.add_argument("--out", help="输出 pcd 路径; 省略 = 与输入同目录的 <名字>_mapframe.pcd")
    parser.add_argument("--z-band", nargs=2, type=float, default=[0.3, 2.0],
                        metavar=("LO", "HI"), help="参与对齐的高度带 (默认 0.3 2.0), 用来排掉地面/天花板")
    parser.add_argument("--sample", type=int, default=1200, help="搜索用采样点数 (默认 1200)")
    parser.add_argument("--yaw-range", type=float, default=180.0, help="粗搜索 yaw 范围 ±度 (默认 180)")
    parser.add_argument("--coarse-step", type=float, default=10.0, help="粗搜索 yaw 步长 度 (默认 10)")
    parser.add_argument("--coarse-trans", type=float, default=6.0, help="粗搜索平移范围 ±米 (默认 6)")
    parser.add_argument("--no-search", action="store_true", help="跳过搜索, 用 --yaw/--dx/--dy")
    parser.add_argument("--yaw", type=float, help="显式 yaw (度)")
    parser.add_argument("--dx", type=float, help="显式 dx (米)")
    parser.add_argument("--dy", type=float, help="显式 dy (米)")
    parser.add_argument("--dry-run", action="store_true", help="只评估, 不写文件")
    parser.add_argument("--force", action="store_true",
                        help="残差超门限 / 输入已标注坐标系时仍然继续 (会把图转坏)")
    args = parser.parse_args(argv)

    for path, what in ((args.pcd, "输入点云"), (args.map, "地图 yaml")):
        if not os.path.isfile(path):
            print("[错误] %s不存在: %s" % (what, path), file=sys.stderr)
            return 2
    if args.dry_run and args.out:
        print("[错误] --dry-run 不写文件, 不能和 --out 一起用", file=sys.stderr)
        return 2

    # 默认输出与输入同目录的 <名字>_mapframe.pcd: 名字自带"已在哪个系", 原始件不动。
    # 这样两种点云可以同时存在而不会被认错 —— 只靠 --prior-pcd <名字> 的搜索顺序来区分
    # (start_real_nav.sh 里 _mapframe 版排在前面, 见那里的 resolve_prior_pcd)。
    if args.out:
        out = args.out
    else:
        stem, ext = os.path.splitext(args.pcd)
        out = stem + "_mapframe" + (ext or ".pcd")
    in_place = os.path.abspath(out) == os.path.abspath(args.pcd)

    grid, resolution, shape, pgm_path = load_grid(args.map)
    arr, info, body_bytes = read_pcd(args.pcd)
    fields = info["fields"]
    if fields[:3] != ["x", "y", "z"]:
        print("[错误] PCD 前三个字段不是 x y z: %s" % fields, file=sys.stderr)
        return 2

    # 已经转过的点云不能再转: 靠头部注释识破 (输出名虽带 _mapframe, 但手工改名/误传
    # 都可能绕过它)。再转一次 = 绕 Z 多转 -90°, 而 GICP 不会报错。
    already = "# frame:" in info["header"]
    if already and not args.force:
        print("[错误] 输入点云头部已标注坐标系, 它是已经转好的地图系点云:", file=sys.stderr)
        for line in info["header"].splitlines():
            if line.startswith("# frame:") or line.startswith("# applied"):
                print("        " + line, file=sys.stderr)
        print("[错误] 再转一次会绕 Z 多转 90°, GICP 不会报错, 车会被定位在错的姿态上。", file=sys.stderr)
        print("[提示] 要转的是原始件 (Point-LIO camera_init 系那份, 头部没有 frame 注释)。", file=sys.stderr)
        print("[提示] 原始件没了就从录包重建:", file=sys.stderr)
        print("[提示]   ./script/build_pcd_from_bag.sh --name <名字> --force", file=sys.stderr)
        print("[提示] 确认要强行再转: 加 --force。", file=sys.stderr)
        return 1

    print("输入点云  : %s (%d 点, %d 字段)%s"
          % (args.pcd, info["points"], len(fields),
             "  [已是地图系]" if already else "  [camera_init 系]"))
    print("目标栅格  : %s -> %s (%dx%d @%.3f m, 占据格 %d)"
          % (args.map, os.path.basename(pgm_path), shape[0], shape[1], resolution, len(grid)))
    print("  " + mounting_hint())
    print("输出      : %s%s" % (out, "  <-- 与输入同一个文件 (--out 指到了输入自身)" if in_place else ""))

    z = arr[:, 2]
    band = (z >= args.z_band[0]) & (z <= args.z_band[1])
    if band.sum() < 100:
        print("[错误] z∈[%g, %g] 区间内只有 %d 个点, 无法对齐; 用 --z-band 调整。"
              % (args.z_band[0], args.z_band[1], band.sum()), file=sys.stderr)
        return 1
    band_xy = arr[band][:, :2].astype(np.float64)
    step = max(1, band_xy.shape[0] // args.sample)
    sample = band_xy[::step]
    print("对齐用点  : z∈[%g,%g] 共 %d 点, 采样 %d 点" % (args.z_band[0], args.z_band[1], band_xy.shape[0], sample.shape[0]))

    identity = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    tree = cKDTree(grid)
    ident_median = score(tree, sample, identity)
    print("  identity 处中位最近邻: %.4f m  (参照: 已对正的 srm_site_01 是 0.026)" % ident_median)

    if args.no_search:
        if args.yaw is None or args.dx is None or args.dy is None:
            print("[错误] --no-search 需要同时给 --yaw/--dx/--dy", file=sys.stderr)
            return 2
        tf = planar_matrix(args.yaw, args.dx, args.dy)
        median = score(tree, sample, tf)
        print("  用给定变换: yaw=%.2f° dx=%.3f dy=%.3f -> 中位 %.4f m" % (args.yaw, args.dx, args.dy, median))
    else:
        median, dx, dy, yaw = search_transform(grid, sample, args.yaw_range, args.coarse_step, args.coarse_trans)
        tf = planar_matrix(yaw, dx, dy)

    inlier = inlier_ratio(tree, sample, tf, 0.1)
    print("最终对齐  : yaw=%.2f°  dx=%.3f  dy=%.3f" % (
        math.degrees(math.atan2(tf[1, 0], tf[0, 0])), tf[0, 2], tf[1, 2]))
    print("            中位最近邻 %.4f m, 0.1 m 内占比 %.1f%%" % (median, inlier * 100))

    if median > ACCEPT_MEDIAN:
        print("[错误] 中位最近邻 %.4f m 超过 %.2f m 门限 —— 两者多半不同源, 不是差个平面变换。"
              % (median, ACCEPT_MEDIAN), file=sys.stderr)
        print("[错误] 检查: 点云与 yaml 是否同一次建图产物 / 高度带里是不是混进了家具和地面。", file=sys.stderr)
        if not args.force:
            return 1
        print("[警告] --force: 继续, 但这份 pcd 拿去重定位会是错的。", file=sys.stderr)
    elif median > GOOD_MEDIAN:
        print("[警告] 中位最近邻 %.4f m 偏大 (参照 0.026 m), 建议先核对 --z-band 与地图是否同源。"
              % median, file=sys.stderr)

    if args.dry_run:
        print("--dry-run: 未写出文件。")
        return 0

    yaw_deg = math.degrees(math.atan2(tf[1, 0], tf[0, 0]))
    out_arr = arr.copy()
    xy = out_arr[:, :2].astype(np.float64)
    out_arr[:, :2] = (xy @ tf[:, :2].T + tf[:, 2]).astype(np.float32)
    origin = yaml.safe_load(open(args.map, encoding="utf-8"))["origin"]
    notes = [
        "frame: pgm map frame of %s (origin %.2f,%.2f, res %.3f)"
        % (os.path.relpath(args.map, os.getcwd()), origin[0], origin[1], resolution),
        "applied to Point-LIO camera_init cloud: yaw %.2f deg, translation (%.3f, %.3f) m"
        % (yaw_deg, tf[0, 2], tf[1, 2]),
        "built by script/pcd_to_mapframe.py from %s" % os.path.basename(args.pcd),
        "alignment: median nn distance %.4f m, %.1f%% within 0.1 m, %d points"
        % (median, inlier * 100, info["points"]),
    ]
    # 先写临时文件 -> 回读复核 -> 通过才原子替换。就地覆盖时这一步尤其重要:
    # 输入就是唯一的原始件, 写坏了就没有第二份可退。
    tmp = write_pcd_tmp(out, out_arr, fields, notes)
    back, back_info, _ = read_pcd(tmp)
    back_sample = back[band][::step][:, :2].astype(np.float64)
    back_median = score(tree, back_sample, identity)
    size_mb = os.path.getsize(tmp) / 1e6

    print("\n写临时文件: %s (%.1f MB, %d 点)" % (tmp, size_mb, back_info["points"]))
    print("回读复核  : 中位最近邻 %.4f m  (identity, 应当与上面的最终对齐值一致)"
          % back_median)
    print("            坐标范围 x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]"
          % (back[:, 0].min(), back[:, 0].max(), back[:, 1].min(),
             back[:, 1].max(), back[:, 2].min(), back[:, 2].max()))
    if abs(back_median - median) > 0.01:
        print("[错误] 回读复核与预期不符 (%.4f vs %.4f), 文件没写对。" % (back_median, median),
              file=sys.stderr)
        os.remove(tmp)
        print("[错误] 已删掉临时文件, %s 原样未动。" % args.pcd, file=sys.stderr)
        return 1

    os.replace(tmp, out)
    print("已写出    : %s" % out)
    print("头部标注  :")
    for line in back_info["header"].splitlines()[:5]:
        print("            " + line)
    if not in_place:
        name = os.path.basename(out)
        if name.endswith("_mapframe.pcd"):
            name = name[:-len("_mapframe.pcd")]
        else:
            name = os.path.splitext(name)[0]
        print("\n提示: 这份 pcd 已经是地图系, 可以直接重定位 (--prior-pcd 会优先命中 _mapframe 那份):")
        print("        ./script/start_real_nav.sh --reloc -m %s --prior-pcd %s" % (name, name))
        print("      对已经转好的点云再跑一次本工具会被拒绝 (头部已标注坐标系), 那是防多转 90° 的保护。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
