#!/usr/bin/env python3
"""把 <名字>_mapframe.pcd 按地面平面摆正，供 zloc backend 先验地图包使用。

为什么需要它
============
`pcd_to_mapframe.py` 只用 (yaw, dx, dy) 把点云对到 PGM，z 方向是按 CAD 安装外参
一次性平移过去的，没有按**实测地面**校平。实车与 CAD 的雷达高度差、建图漂移、
以及建图起点不是绝对水平，都会让 map 系的地面落在 z≈0.05~0.10 m 且带 ~1° 的
roll/pitch。backend 先验模式没有重力/tilt 因子，6-DoF 配准会把这 1° 和几厘米
原样带进 `map→odom`，Nav2 的 global_costmap（map 系）与 local_costmap（odom 系，
又 static_layer 复制了同一张 /map）就出现"两层"：8 m 局部窗口上高度差可到 10 cm。

本工具只做一件事：用地面点拟合平面，把该平面的法向旋到 +Z、把地面平移到 z=0，
其余字段（intensity/normal 等）原样保留，输出 `<名字>_mapframe_leveled.pcd`。
原始件不动；确认后再用它重建 backend bundle。

注意：只改 3D PCD，不动 2D PGM/yaml。地图系的 x/y 原点与 yaw 不重新求解；旋转
是绕 map 原点做的，地面上的点在 x/y 上不变，只有高处点的 x/y 会有 z·sin(θ) 量级
的微小位移（θ≈1° 时 4 m 高处约 7 cm）——这是"把倾斜的点云真正摆正"的必然结果。

用法
====
    # 只看拟合结果，不写文件
    python3 scripts/mapping/level_map_pcd.py \
        --pcd maps/pcd/219_1121_mapframe.pcd --dry-run

    # 写 maps/pcd/219_1121_mapframe_leveled.pcd
    python3 scripts/mapping/level_map_pcd.py \
        --pcd maps/pcd/219_1121_mapframe.pcd

    # 然后重建 bundle（注意 --sensor-z 用实测雷达离地高度）
    ./scripts/mapping/build_zloc_map_bundle.sh \
        --name 219_1121 \
        --pcd maps/pcd/219_1121_mapframe_leveled.pcd \
        --out-bundle "$PWD/maps/pcd/map_bundles/219_1121" \
        --sensor-z 0.2966 --force

退出码：0 成功；1 拟合/校验失败；2 参数或文件格式错误。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np


# 默认只接受平缓的地面：倾斜超过这个角度就拒绝，避免把墙/桌面误当地面。
MAX_TILT_DEG = 15.0
# 地面内点阈（米）。地面点的噪声约 1~3 cm。
INLIER_DIST = 0.02


def read_pcd(path):
    """读取 binary PCD，返回 (header_lines, fields, data[N,nf], header_text)。"""
    with open(path, "rb") as stream:
        header_lines = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"PCD 头不完整: {path}")
            header_lines.append(line)
            if line.startswith(b"DATA"):
                break
        data_bytes = stream.read()

    text = b"".join(header_lines).decode("ascii", errors="strict")
    meta = {}
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("#") or not line:
            continue
        key, _, value = line.partition(" ")
        meta[key.upper()] = value.strip()

    fields = meta["FIELDS"].split()
    sizes = [int(v) for v in meta["SIZE"].split()]
    types = meta["TYPE"].split()
    counts = [int(v) for v in meta["COUNT"].split()]
    points = int(meta["POINTS"])
    if meta["DATA"].lower() != "binary":
        raise ValueError(f"只支持 DATA binary，收到 {meta['DATA']!r}")
    if any(t != "F" or s != 4 for t, s in zip(types, sizes)):
        raise ValueError(f"只支持 float32 字段，收到 TYPE={types} SIZE={sizes}")
    point_step = sum(s * c for s, c in zip(sizes, counts))
    expected = points * point_step
    if len(data_bytes) != expected:
        raise ValueError(f"点云字节数不符: {len(data_bytes)} != {expected}")
    raw = np.frombuffer(data_bytes, dtype=np.float32).reshape(points, point_step // 4).copy()
    # 计算每个字段的起始列（当前所有字段都是 1 个 float32；保留通用写法）。
    offsets = np.cumsum([0] + [c for c in counts])
    columns = {}
    for index, name in enumerate(fields):
        columns[name] = offsets[index]
    return header_lines, text, fields, columns, raw, points


def write_pcd(path, header_lines, data, note):
    """按原字段布局写 binary PCD，并在注释块末尾追加一行 note。"""
    lines = [line.decode("ascii", errors="strict").rstrip("\n") for line in header_lines]
    insert_at = 0
    for index, line in enumerate(lines):
        if line.startswith("#"):
            insert_at = index + 1
    lines.insert(insert_at, note)
    # 结构化行（VERSION/FIELDS/.../DATA）保持原样，只插入一行注释。
    new_header = "\n".join(lines) + "\n"
    with open(path, "wb") as stream:
        stream.write(new_header.encode("ascii"))
        stream.write(np.asarray(data, dtype=np.float32).tobytes())
    return new_header


def plane_from_points(points):
    """最小二乘拟合 z = a*x + b*y + c，返回 (a,b,c)。"""
    matrix = np.column_stack((points[:, 0], points[:, 1], np.ones(len(points))))
    coef, *_ = np.linalg.lstsq(matrix, points[:, 2], rcond=None)
    return float(coef[0]), float(coef[1]), float(coef[2])


def fit_floor(points, iterations=800, seed=20261124):
    """在“最低一层”点里做 RANSAC + 最小二乘细化，返回 (normal, d, mask, rms, tilt)。

    直接对全图采样命中率太低（地面只占 ~4%），但地面几乎一定落在最低 z 的薄带里。
    先取最低 1% 到其上方 0.25 m 的带（219 是 z≈0.05~0.18，带进少量墙脚/低障碍），
    在带内 RANSAC；用整条带的内点数投票，地面（内点最多）会赢过局部桌面/墙脚。
    """
    z = points[:, 2]
    z_low = float(np.percentile(z, 1.0))
    band = (z > z_low - 0.10) & (z < z_low + 0.25)
    band_points = points[band]
    if len(band_points) < 5000:
        raise ValueError(f"最低层点数太少: {len(band_points)}")

    # RANSAC 只在带内抽样；整条带用于计票（~1e5 点，800 次约 1 秒）。
    sample = band_points[:: max(1, len(band_points) // 20000)]
    rng = np.random.default_rng(seed)
    cos_limit = math.cos(math.radians(MAX_TILT_DEG))
    best = None  # (inliers, normal, d)
    for _ in range(iterations):
        triad = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(triad[1] - triad[0], triad[2] - triad[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        if normal[2] < 0:
            normal = -normal
        if normal[2] < cos_limit:
            continue
        d = -float(normal @ triad[0])
        height = -d / normal[2]
        if height < z_low - 0.10 or height > z_low + 0.25:
            continue
        inliers = int(np.count_nonzero(np.abs(band_points @ normal + d) < INLIER_DIST))
        if best is None or inliers > best[0]:
            best = (inliers, normal, d)
    if best is None or best[0] < 5000:
        raise ValueError("RANSAC 没找到足够大的地面平面")

    normal, d = best[1], best[2]
    # 用全量内点做最小二乘细化，并迭代收缩残差。
    for _ in range(4):
        mask = np.abs(band_points @ normal + d) < INLIER_DIST
        if int(np.count_nonzero(mask)) < 5000:
            break
        a, b, c = plane_from_points(band_points[mask])
        scale = math.sqrt(a * a + b * b + 1.0)
        normal = np.array([-a / scale, -b / scale, 1.0 / scale])
        d = -c / scale

    residual = points @ normal + d
    mask = np.abs(residual) < INLIER_DIST
    if int(np.count_nonzero(mask)) < 5000:
        raise ValueError("细化后地面内点不足")
    rms = float(np.sqrt(np.mean(residual[mask] ** 2)))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, normal[2]))))
    if tilt > MAX_TILT_DEG:
        raise ValueError(f"拟合平面倾斜 {tilt:.2f}° 超过 {MAX_TILT_DEG}°")
    return normal, d, mask, rms, tilt


def level_rotation(normal):
    """返回把 normal 旋到 +Z 的旋转矩阵。"""
    target = np.array([0.0, 0.0, 1.0])
    axis = np.cross(normal, target)
    sin_angle = float(np.linalg.norm(axis))
    cos_angle = float(normal @ target)
    if sin_angle < 1e-12:
        return np.eye(3) if cos_angle > 0 else np.diag([1.0, -1.0, -1.0])
    axis /= sin_angle
    skew = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    angle = math.atan2(sin_angle, cos_angle)
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--name", help="地图名: 自动推 maps/pcd/<名字>_mapframe.pcd")
    parser.add_argument("--pcd", help="输入地图系 PCD; 给了 --name 可省")
    parser.add_argument("--out", default="", help="输出 PCD（默认 <stem>_leveled.pcd）")
    parser.add_argument("--dry-run", action="store_true", help="只拟合打印，不写文件")
    parser.add_argument("--force", action="store_true", help="覆盖已存在的输出")
    args = parser.parse_args(argv)

    if args.name and not args.pcd:
        ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        args.pcd = os.path.join(ws, "maps", "pcd", args.name + "_mapframe.pcd")
    if not args.pcd:
        parser.error("需要 --pcd 或 --name")

    pcd = os.path.abspath(args.pcd)
    if not os.path.isfile(pcd):
        print(f"[错误] 输入不存在: {pcd}", file=sys.stderr)
        return 2
    if args.out:
        out = os.path.abspath(args.out)
    else:
        stem, ext = os.path.splitext(pcd)
        out = stem + "_leveled" + (ext or ".pcd")
    if out == pcd:
        print("[错误] 输出不能等于输入；原始件必须保留", file=sys.stderr)
        return 2
    if os.path.exists(out) and not args.force and not args.dry_run:
        print(f"[错误] 输出已存在（加 --force 覆盖）: {out}", file=sys.stderr)
        return 2

    try:
        header_lines, _header_text, fields, columns, data, points = read_pcd(pcd)
    except (OSError, ValueError) as error:
        print(f"[错误] {error}", file=sys.stderr)
        return 2
    if not {"x", "y", "z"}.issubset(columns):
        print(f"[错误] PCD 缺少 x/y/z 字段: {fields}", file=sys.stderr)
        return 2

    ix, iy, iz = columns["x"], columns["y"], columns["z"]
    # 取 xyz 的 float64 视图，其它字段留在 data 里不动。
    xyz = data[:, [ix, iy, iz]].astype(np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) < 10000:
        print(f"[错误] 有效点太少: {len(xyz)}", file=sys.stderr)
        return 1

    try:
        normal, d, mask, rms, tilt = fit_floor(xyz)
    except ValueError as error:
        print(f"[错误] 地面拟合失败: {error}", file=sys.stderr)
        return 1

    rotation = level_rotation(normal)
    translation = np.array([0.0, 0.0, d])
    leveled = xyz @ rotation.T + translation
    print("========== 地面平面拟合 ==========")
    print(f"  输入      : {pcd} ({points} 点)")
    print(f"  法向      : ({normal[0]:+.5f}, {normal[1]:+.5f}, {normal[2]:+.5f})")
    print(f"  倾斜      : {tilt:.3f}°   RMS {rms * 100:.2f} cm")
    print(f"  地面高度  : z(0,0) = {-d / normal[2]:+.4f} m（相对 map 原点）")
    print(f"  内点      : {int(mask.sum())} / {len(xyz)} ({mask.mean() * 100:.1f}%)")
    print(f"  补偿      : R(normal→+Z) 后 z 平移 {d:+.4f} m")
    post_median = float(np.median(leveled[mask, 2]))
    post_rms = float(np.sqrt(np.mean((leveled[mask, 2] - post_median) ** 2)))
    print(f"  校平后地面: z 中位 {post_median * 100:+.2f} cm, RMS {post_rms * 100:.2f} cm")
    if args.dry_run:
        print("  [dry-run] 不写文件")
        return 0

    # 原始点云里可能有非有限点；只对有限行做变换，其余保持原值。
    finite = np.isfinite(data[:, [ix, iy, iz]]).all(axis=1)
    transformed = data[finite][:, [ix, iy, iz]].astype(np.float64) @ rotation.T + translation
    data[finite, ix] = transformed[:, 0].astype(np.float32)
    data[finite, iy] = transformed[:, 1].astype(np.float32)
    data[finite, iz] = transformed[:, 2].astype(np.float32)

    note = (
        "# leveled by floor plane fit (level_map_pcd.py): "
        f"n=({normal[0]:+.5f},{normal[1]:+.5f},{normal[2]:+.5f}) "
        f"tilt={tilt:.3f}deg dz={d:+.4f}m rms={rms * 100:.2f}cm "
        f"inliers={int(mask.sum())}/{len(xyz)}"
    )
    try:
        write_pcd(out, header_lines, data, note)
    except OSError as error:
        print(f"[错误] 写文件失败: {error}", file=sys.stderr)
        return 1
    print(f"  输出      : {out}")
    print(f"  头部标注  : {note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
