#!/usr/bin/env python3
"""把一张 pgm 先验图刚性配准到点云，再分走廊段统计墙-墙残差。

用法：
  python3 scripts/mapping/verify_map_vs_pcd.py --yaml maps/xxx/xxx.yaml \
      --pcd maps/pcd/xxx_mapframe_leveled.pcd
"""
import argparse
import os
import sys

import numpy as np
import yaml
from scipy.spatial import cKDTree

ARMS = {
    'arm1 (202 段)': (2, 32, -21, -2),
    'arm2 (227 段)': (30, 50, -23, -2),
}


def load_pgm(path):
    d = open(path, 'rb').read()
    i = 0
    t = []
    while len(t) < 4:
        while d[i:i + 1].isspace():
            i += 1
        if d[i:i + 1] == b'#':
            while d[i:i + 1] != b'\n':
                i += 1
            continue
        s = i
        while not d[i:i + 1].isspace():
            i += 1
        t.append(d[s:i])
    w, h = int(t[1]), int(t[2])
    i += 1
    return np.frombuffer(d[i:], dtype=np.uint8)[:w * h].reshape(h, w)


def read_pcd(path):
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
    dt = {'F': {4: 'f4', 8: 'f8'}, 'U': {1: 'u1', 2: 'u2', 4: 'u4', 8: 'u8'},
          'I': {1: 'i1', 2: 'i2', 4: 'i4', 8: 'i8'}}
    dtype = np.dtype([(f, '<' + dt[t][s]) for f, t, s in zip(fields, types, sizes)])
    return np.frombuffer(payload, dtype=dtype, count=int(hdr['POINTS']))


def cells(yaml_path, select):
    m = yaml.safe_load(open(yaml_path, encoding='utf-8'))
    res = float(m['resolution'])
    ox, oy = float(m['origin'][0]), float(m['origin'][1])
    a = load_pgm(yaml_path.rsplit('/', 1)[0] + '/' + m['image'])
    H, W = a.shape
    cj, ci = np.nonzero(select(a))
    pts = np.stack([ox + (ci + 0.5) * res, oy + (H - 0.5 - cj) * res], 1)
    return pts, a


def icp(Q, P, yaw_range=8.0, step=0.2, iters=8, gate=0.35, seed=0):
    Ps = P[np.random.default_rng(seed).choice(len(P), min(80000, len(P)), replace=False)]
    tree = cKDTree(Ps)
    best = None
    for yaw in np.arange(-yaw_range, yaw_range + 1e-9, step):
        c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
        R = np.array([[c, -s], [s, c]])
        d = Ps.mean(0) - Q.mean(0) @ R.T
        dist, _ = tree.query(Q[::3] @ R.T + d)
        sc = np.median(dist)
        if best is None or sc < best[0]:
            best = (sc, R, d.copy())
    _, R, d = best
    for it in range(iters):
        Qr = Q @ R.T + d
        dist, idx = tree.query(Qr)
        keep = dist < gate
        if keep.sum() < 200:
            break
        A, B = Qr[keep], Ps[idx[keep]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, S, Vt = np.linalg.svd(H)
        Rr = Vt.T @ U.T
        if np.linalg.det(Rr) < 0:
            Vt[-1] *= -1
            Rr = Vt.T @ U.T
        R = Rr @ R
        d = Rr @ d + cb - Rr @ ca
        res = np.median(dist[keep])
        print('  ICP %d: 内点 %5d  中位 %.3f m' % (it + 1, keep.sum(), res))
    return R, d, Ps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', help='地图名: 自动推 maps/<名字>/<名字>.yaml 与 '
                                   'maps/pcd/<名字>_mapframe_leveled.pcd（没有就退回 _mapframe.pcd）')
    ap.add_argument('--yaml')
    ap.add_argument('--pcd')
    ap.add_argument('--wall-min', type=float, default=0.25)
    ap.add_argument('--wall-max', type=float, default=1.8)
    ap.add_argument('--box', nargs=4, type=float, action='append', metavar=('X0', 'X1', 'Y0', 'Y1'),
                    help='只统计这个矩形里的墙格（可重复）。不给就自动找残差最大的区域')
    ap.add_argument('--cell', type=float, default=4.0, help='自动分区的手掌大小（米，默认 4）')
    ap.add_argument('--force', action='store_true', help='占据格异常多时仍然继续（默认直接拒绝）')
    args = ap.parse_args()

    if args.name:
        ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        if not args.yaml:
            args.yaml = os.path.join(ws, 'maps', args.name, args.name + '.yaml')
        if not args.pcd:
            leveled = os.path.join(ws, 'maps', 'pcd', args.name + '_mapframe_leveled.pcd')
            plain = os.path.join(ws, 'maps', 'pcd', args.name + '_mapframe.pcd')
            args.pcd = leveled if os.path.isfile(leveled) else plain
    if not args.yaml or not args.pcd:
        ap.error('需要 --yaml/--pcd，或给一个 --name')

    Q, a = cells(args.yaml, lambda x: x == 0)
    stats = {v: 100.0 * (a == v).mean() for v in (0, 205)}
    free = 100.0 * (a >= 250).mean()
    print('图: %dx%d @ %.3f m   free %.1f%%  occupied %.1f%%  unknown %.1f%%'
          % (a.shape[1], a.shape[0], 0.05, free, stats[0], stats[205]))
    print('  墙格 %d' % len(Q))

    # 正常 2D 图占据通常 <10%；一旦 >25%，几乎都是"未知(205)被写成了 0"——
    # 典型成因: 用 GIMP 打开 pgm 再导出（本机两次都这样: 202to227 94%，227to219_1125 91%）。
    if stats[0] > 25.0 and not args.force:
        print('[错误] 这张图的占据格占 %.1f%%，正常应 <10%% —— 多半是"未知(205)被写成 0"。' % stats[0],
              file=sys.stderr)
        print('[错误] 先修图: 0=占据 / 205=未知 / 254=自由 三种值必须保留；不要用 GIMP 重导出 pgm。',
              file=sys.stderr)
        print('[提示] 确认图没问题、只是场地特殊: 加 --force 继续。', file=sys.stderr)
        return 2

    p = read_pcd(args.pcd)
    X, Y, Z = p['x'].astype(float), p['y'].astype(float), p['z'].astype(float)
    w = (Z > args.wall_min) & (Z < args.wall_max)
    P = np.stack([X[w], Y[w]], 1)
    R, d, Ps = icp(Q, P)
    print('配准: yaw %+.2f°, 平移 (%.2f, %.2f)'
          % (np.degrees(np.arctan2(R[1, 0], R[0, 0])), d[0], d[1]))

    Qw = Q @ R.T + d
    tree = cKDTree(P)
    dist, _ = tree.query(Qw)
    def report(name, m):
        if m.sum() < 50:
            print('  %-16s 墙格太少 (%d)' % (name, m.sum()))
            return
        dd = dist[m]
        print('  %-16s 墙格 %5d  中位 %.3f m  p90 %.3f m  <=0.15m %.0f%%'
              % (name, m.sum(), np.median(dd), np.percentile(dd, 90), 100 * (dd <= 0.15).mean()))

    print('\n墙-墙残差（配准后）:')
    report('全部', np.ones(len(dist), bool))
    if args.box:
        for i, (x0, x1, y0, y1) in enumerate(args.box, 1):
            report('box%d [%.1f,%.1f]x[%.1f,%.1f]' % (i, x0, x1, y0, y1),
                   (Qw[:, 0] > x0) & (Qw[:, 0] < x1) & (Qw[:, 1] > y0) & (Qw[:, 1] < y1))
    else:
        # 自动找最差区域: 按 cell 米分块，只保留格数 >=50 的块，按中位残差倒序
        cell = max(1.0, args.cell)
        ix = np.floor(Qw[:, 0] / cell).astype(int)
        iy = np.floor(Qw[:, 1] / cell).astype(int)
        blocks = {}
        for bx, by, dd in zip(ix, iy, dist):
            blocks.setdefault((bx, by), []).append(dd)
        rows = [(np.median(v), len(v), k) for k, v in blocks.items() if len(v) >= 50]
        rows.sort(reverse=True)
        if not rows:
            print('  （没有格数 >=50 的分块，画不出热点）')
        for med, n, (bx, by) in rows[:6]:
            print('  热点 %5.1f,%5.1f m  墙格 %4d  中位 %.3f m' % (bx * cell, by * cell, n, med))
        bad = sum(1 for med, _, _ in rows if med > 0.3)
        print('  >0.3 m 的分块: %d / %d（好图应为 0）' % (bad, len(rows)))


if __name__ == '__main__':
    main()
