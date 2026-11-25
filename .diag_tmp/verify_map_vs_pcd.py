#!/usr/bin/env python3
"""把一张 pgm 先验图刚性配准到点云，再分走廊段统计墙-墙残差。

用法：
  python3 .diag_tmp/verify_map_vs_pcd.py --yaml maps/xxx/xxx.yaml \
      --pcd maps/pcd/xxx_mapframe_leveled.pcd
"""
import argparse

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
    ap.add_argument('--yaml', required=True)
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--wall-min', type=float, default=0.25)
    ap.add_argument('--wall-max', type=float, default=1.8)
    args = ap.parse_args()

    Q, a = cells(args.yaml, lambda x: x == 0)
    stats = {v: 100.0 * (a == v).mean() for v in (0, 205)}
    free = 100.0 * (a >= 250).mean()
    print('图: %dx%d @ %.3f m   free %.1f%%  occupied %.1f%%  unknown %.1f%%'
          % (a.shape[1], a.shape[0], 0.05, free, stats[0], stats[205]))
    print('  墙格 %d' % len(Q))

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
    print('\n分臂墙-墙残差（配准后）:')
    for name, (x0, x1, y0, y1) in ARMS.items():
        m = (Qw[:, 0] > x0) & (Qw[:, 0] < x1) & (Qw[:, 1] > y0) & (Qw[:, 1] < y1)
        if m.sum() < 50:
            print('  %-14s 墙格太少 (%d)' % (name, m.sum()))
            continue
        dd = dist[m]
        print('  %-14s 墙格 %5d  中位 %.3f m  p90 %.3f m  <=0.15m %.0f%%'
              % (name, m.sum(), np.median(dd), np.percentile(dd, 90), 100 * (dd <= 0.15).mean()))


if __name__ == '__main__':
    main()
