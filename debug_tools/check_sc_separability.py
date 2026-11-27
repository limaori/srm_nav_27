#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""先验地图包的自检：Scan Context 到底能不能把"这里"和"别处"分开。

为什么需要这个工具
==================
换了一张更大的先验图之后，最典型的翻车不是配准精度不够，而是**全局描述子先失去
判别力**：`common.scan_context.distance_threshold` 这个闸门后面本来应该"正确位置
远小于阈值、错误位置远大于阈值"，一旦这条不变式不成立，检索就是随机的，几何验证
再从 `candidates` 个候选里挑一个"残差好看"的 —— 对称走廊里 180°/几十米的错解
完全能通过。

而这张薄图在**小地图上根本不会暴露**：219_1121 只有 48 帧、8.7×11.6 m，任何候选
离真值都只有几米，选错也错不远；227to219_1125 是 1960 帧、51×82 m 的两段同构走廊，
错一次就是几十米。这就是"同一份代码换张图才炸"的机理。

判据（本工具的结论口径）
========================
对每个抽样关键帧 i：

  正确位置 = 与 i 相距 1~4.5 m 的关键帧（同一条走廊上的邻帧）
  错误位置 = 与 i 相距 >= `--far` 米（默认 20 m）的关键帧

用 backend **存储里那份描述子**（不是重算的）和 backend **同一套距离函数**算：

    d_true = min over 正确位置
    d_false = min over 错误位置

健康的地图应满足：

    d_true  <<  threshold  <<  d_false

不健康（本工具判 FAIL）的两种表现形式：

  1. `d_false < threshold` —— 闸门拦不住错解，检索等于全放行；
  2. `d_false < d_true`    —— 错解排序比正解还靠前，`candidates` 再按距离排序
                              挑前 15 个，正解直接被挤掉。

用法::

    ./debug_tools/check_sc_separability.py --bundle maps/pcd/map_bundles/227to219_1125
    ./debug_tools/check_sc_separability.py --bundle ... --threshold 0.8 --far 20
    ./debug_tools/check_sc_separability.py --bundle ... --stride 32 --far-candidates 400

退出码：0 = 可分（d_false 明显大于阈值且大于 d_true）；1 = 不可分；2 = 参数/文件问题。

口径说明（为什么不是"重算描述子"）
==================================
描述子是 `build_pcd_map_bundle` 落盘的二进制（魔数 0x53435859，每帧
`1 + augment_count × (8 + rings × sectors × 4)` 字节），本工具直接解析，因此
`--sc-*` 改了没重建包、或建包和运行期配置不一致这类问题会在这里直接现形 ——
不用去现场复现。距离函数与 `src/scan_context/descriptor.cpp::descriptorDistance`
一致：按扇区列余弦取均值，`1 - mean(cos)`，扇区循环移 `search_ratio` 给的范围
（默认按配置里的 1.0，即整周 360°，所以 180° 混叠在这里是"合法"候选）。
"""

import argparse
import os
import struct
import sys

import numpy as np

MAGIC = 0x53435859


def parse_descriptor_file(path):
    """解析 backend 的描述子二进制，返回 (D[N,V,R,S], config dict)。"""
    raw = open(path, 'rb').read()
    off = 0

    def rd(fmt):
        nonlocal off
        size = struct.calcsize('<' + fmt)
        vals = struct.unpack_from('<' + fmt, raw, off)
        off += size
        return vals

    magic, version, count = rd('III')
    if magic != MAGIC:
        raise SystemExit('[错误] %s 不是 Scan Context 描述子文件 (magic=0x%08x)' % (path, magic))
    enabled, leveled = rd('BB')
    rings, sectors = rd('ii')
    max_radius, downsample, lidar_height = rd('ddd')
    min_bin_points, augment = rd('ii')
    lateral_shift, search_ratio = rd('dd')
    frames = []
    for _ in range(count):
        (variants,) = rd('I')
        out = []
        for _ in range(variants):
            r, c = rd('ii')
            if r != rings or c != sectors:
                raise SystemExit('[错误] 第 %d 帧描述子尺寸 %dx%d 与头部 %dx%d 不一致'
                                 % (len(frames), r, c, rings, sectors))
            vals = np.frombuffer(raw, dtype='<f4', count=r * c, offset=off)
            off += 4 * r * c
            out.append(vals.reshape(r, c).astype(np.float32))
        frames.append(np.stack(out))
    cfg = dict(version=version, enabled=enabled, leveled=leveled, rings=rings,
               sectors=sectors, max_radius=max_radius, downsample=downsample,
               lidar_height=lidar_height, min_bin_points=min_bin_points,
               augment_count=augment, lateral_shift=lateral_shift,
               search_ratio=search_ratio, count=count)
    return np.stack(frames), cfg


def load_keyframes(path):
    rows = []
    for line in open(path, 'r', encoding='utf-8'):
        if line.startswith('#') or not line.strip():
            continue
        rows.append([float(x) for x in line.split()[:8]])
    return np.asarray(rows)


def distance_to(query_image, candidates, search_ratio):
    """与 backend `descriptorDistance` 同口径：列余弦均值 + 扇区循环移搜索。

    query_image : (R, S)   查询帧的第 0 个变体（backend 也用 front() 当 query）
    candidates  : (M, V, R, S)
    返回 (M,) 每个候选在全部变体、全部合法移位上取下界的距离。
    """
    rings, sectors = query_image.shape
    q_norm = np.linalg.norm(query_image, axis=0)
    radius = max(0, int(np.ceil(0.5 * search_ratio * sectors)))
    shifts = range(-radius, radius + 1)
    best = np.full(candidates.shape[0], np.inf)
    for v in range(candidates.shape[1]):
        cand = candidates[:, v]
        c_norm = np.linalg.norm(cand, axis=1)                  # (M, S)
        cand_t = cand.transpose(0, 2, 1)                       # (M, S, R)
        for shift in shifts:
            qs = np.roll(query_image, -shift, axis=1)          # 与 backend 的 q=(sector+shift)%S 对齐
            dots = np.einsum('msr,rs->ms', cand_t, qs)         # (M, S)
            engaged = (np.roll(q_norm, -shift)[None, :] > 0) & (c_norm > 0)
            sim = dots / (np.roll(q_norm, -shift)[None, :] * c_norm + 1e-12)
            used = np.maximum(engaged.sum(axis=1), 1)
            dist = 1.0 - np.where(engaged, sim, 0.0).sum(axis=1) / used
            best = np.minimum(best, dist)
    return best


def main():
    ap = argparse.ArgumentParser(
        description='检查先验地图包的 Scan Context 是否能把"这里"和"别处"分开')
    ap.add_argument('--bundle', required=True, help='地图包目录（含 keyframes.txt 与 sc_descriptors.bin）')
    ap.add_argument('--threshold', type=float, default=0.8,
                    help='运行期 common.scan_context.distance_threshold（默认 0.8，与 backend.yaml 一致）')
    ap.add_argument('--near', nargs=2, type=float, default=[1.0, 4.5],
                    metavar=('MIN', 'MAX'), help='"正确位置"的距离带，米（默认 1.0 4.5）')
    ap.add_argument('--far', type=float, default=20.0, help='"错误位置"的最小距离，米（默认 20）')
    ap.add_argument('--stride', type=int, default=64, help='抽样步长（默认每 64 帧取一帧）')
    ap.add_argument('--far-candidates', type=int, default=250,
                    help='每次查询随机抽多少个"错误位置"来比（默认 250，0 表示全量）')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--search-ratio', type=float, default=None,
                    help='覆盖描述子文件里的 search_ratio（默认用文件里存的）')
    args = ap.parse_args()

    kf_path = os.path.join(args.bundle, 'keyframes.txt')
    sc_path = os.path.join(args.bundle, 'sc_descriptors.bin')
    for p in (kf_path, sc_path):
        if not os.path.isfile(p):
            raise SystemExit('[错误] 缺文件: %s' % p)

    D, cfg = parse_descriptor_file(sc_path)
    kf = load_keyframes(kf_path)
    if len(kf) != cfg['count']:
        raise SystemExit('[错误] keyframes.txt 有 %d 帧，描述子有 %d 帧 —— 两者不是同一次建包'
                         % (len(kf), cfg['count']))
    ratio = cfg['search_ratio'] if args.search_ratio is None else args.search_ratio
    xy = kf[:, 2:4]
    n_frames = len(kf)

    nz = (D != 0).sum(axis=(2, 3))
    col = (np.abs(D).sum(axis=2) > 0).sum(axis=2)
    per_col = (D != 0).sum(axis=2)
    spikes = (per_col[col > 0] <= 2).mean() if (col > 0).any() else 1.0

    print('地图包: %s' % args.bundle)
    print('  帧数 %d  描述子 % dx%d  变体 %d  max_radius %.1f m  downsample %.2f m  '
          'lidar_height %.4f  search_ratio %.2f'
          % (n_frames, cfg['rings'], cfg['sectors'], cfg['augment_count'],
             cfg['max_radius'], cfg['downsample'], cfg['lidar_height'], ratio))
    print('  非零格/帧 中位 %.0f / %d   有效扇区/帧 中位 %.0f / %d   '
          '单点扇区(<=2 个非零环)占比 %.0f%%'
          % (np.median(nz), cfg['rings'] * cfg['sectors'], np.median(col),
             cfg['sectors'], 100.0 * spikes))
    print('  覆盖范围 x %.1f..%.1f  y %.1f..%.1f m'
          % (xy[:, 0].min(), xy[:, 0].max(), xy[:, 1].min(), xy[:, 1].max()))

    rng = np.random.default_rng(args.seed)
    near_lo, near_hi = args.near
    rows = []
    for qi in range(0, n_frames, max(1, args.stride)):
        sep = np.linalg.norm(xy - xy[qi], axis=1)
        near = np.where((sep >= near_lo) & (sep <= near_hi))[0]
        far = np.where(sep >= args.far)[0]
        if len(near) == 0 or len(far) == 0:
            continue
        if args.far_candidates and len(far) > args.far_candidates:
            far = rng.choice(far, args.far_candidates, replace=False)
        d_near = float(distance_to(D[qi, 0], D[near], ratio).min())
        d_far_all = distance_to(D[qi, 0], D[far], ratio)
        j = int(np.argmin(d_far_all))
        rows.append((qi, int(far[j]), d_near, float(d_far_all[j]),
                     float(np.linalg.norm(xy[far[j]] - xy[qi]))))
    if not rows:
        raise SystemExit('[错误] 没有可用样本：地图可能太小，或 --near/--far 设得不合适')

    r = np.asarray(rows)
    d_true, d_false = r[:, 2], r[:, 3]
    margin = d_false - d_true
    print('\n抽样 %d 帧（正确位置 %.1f~%.1f m，错误位置 >= %.0f m）'
          % (len(r), near_lo, near_hi, args.far))
    print('  正确位置距离 d_true : 中位 %.3f  p90 %.3f  max %.3f' % (
        np.median(d_true), np.percentile(d_true, 90), d_true.max()))
    print('  错误位置距离 d_false: 中位 %.3f  p10 %.3f  min %.3f' % (
        np.median(d_false), np.percentile(d_false, 10), d_false.min()))
    print('  余量 d_false-d_true: 中位 %+.3f  min %+.3f' % (
        np.median(margin), margin.min()))
    for thr in (0.6, 0.7, args.threshold):
        print('  错误位置 < %.2f 的比例: %d/%d (%.0f%%)'
              % (thr, int((d_false < thr).sum()), len(r), 100.0 * (d_false < thr).mean()))
    worst = r[np.argsort(margin)[:5]]
    print('  最危险的 5 帧（错误位置最接近甚至优于正确位置）:')
    for q in worst:
        print('    kf%5d (%6.1f,%6.1f) d_true=%.3f  <- 错配 kf%5d 距 %5.1f m  d_false=%.3f'
              % (q[0], xy[int(q[0]), 0], xy[int(q[0]), 1], q[2], q[1], q[4], q[3]))

    pass_gate = (d_false < args.threshold).mean()
    ordering_ok = (margin > 0).mean()
    print()
    if pass_gate > 0.05 or ordering_ok < 0.95:
        print('[结论] FAIL：这份包的 Scan Context 不可分。')
        print('        %.0f%% 的错误位置能通过 distance_threshold=%.2f 的闸门；'
              '%.0f%% 的抽样里错误位置的排序比正确位置还靠前。'
              % (100 * pass_gate, args.threshold, 100 * (1 - ordering_ok)))
        print('        此时恢复/重定位的检索等价于随机挑候选，靠几何验证兜底；'
              '同构走廊下 180°/几十米的错解会"残差好看"地通过验收。')
        print('        处置：把 max_radius 收小到关键帧附近（如 25~30 m）、收紧 candidates/'
              'search_ratio，并用同一组 --sc-* 重建包；或者直接把先验图裁到实际会跑的那条走廊。')
        return 1
    print('[结论] PASS：错误位置稳定高于阈值、且排序劣于正确位置，检索可用。')
    print('        建议把 distance_threshold 取在 d_true 的 p90 与 d_false 的 p10 之间：'
          '约 %.2f ~ %.2f。' % (np.percentile(d_true, 90), np.percentile(d_false, 10)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
