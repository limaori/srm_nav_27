#!/usr/bin/env python3
"""把一张 2D 栅格图（pgm+yaml）和一份地图系点云做对比：

1) 栅格统计：free / occupied / unknown 的比例（判断 unknown 有没有被写成 0）
2) 逐站法向偏移：pgm 自由带中心 vs 点云走廊中心（arm1=202 段 / arm2=227 段）

用法：
  python3 .diag_tmp/check_map_vs_pcd.py --yaml maps/202to227/202to227.yaml \
      --pcd maps/pcd/202to227_mapframe_leveled.pcd
"""
import argparse
import numpy as np
import yaml


def load_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    idx = 0
    toks = []
    while len(toks) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] != b'\n':
                idx += 1
            continue
        s = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        toks.append(data[s:idx])
    w, h = int(toks[1]), int(toks[2])
    idx += 1
    return np.frombuffer(data[idx:], dtype=np.uint8)[:w * h].reshape(h, w)


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', required=True)
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--free-thresh', type=float, default=0.25)
    ap.add_argument('--occ-thresh', type=float, default=0.65)
    args = ap.parse_args()

    meta = yaml.safe_load(open(args.yaml, encoding='utf-8'))
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    negate = int(meta.get('negate', 0))
    pgm_path = args.yaml.rsplit('/', 1)[0] + '/' + meta['image']
    a = load_pgm(pgm_path)
    H, W = a.shape

    # 按 PGM 取值直接分类：0=占据, >=250=自由, 其余(典型 205)=未知。
    # 不用 occ 阈值反算：free_thresh 0.25 会把 205 也算成 free（map_server 也是这样），
    # 那样统计出来"未知"永远是 0，看不出问题。
    free = a >= 250
    occupied = a == 0
    unknown = ~free & ~occupied

    print('=== 栅格统计: %s (%dx%d @ %.3f m, origin %.2f,%.2f)' % (pgm_path, W, H, res, ox, oy))
    for name, m in (('free', free), ('occupied', occupied), ('unknown', unknown)):
        print('  %-8s %8d  %5.1f%%' % (name, m.sum(), 100.0 * m.mean()))

    p = read_pcd(args.pcd)
    X = p['x'].astype(float)
    Y = p['y'].astype(float)
    Z = p['z'].astype(float)
    wall = (Z > 0.25) & (Z < 1.8)
    X, Y = X[wall], Y[wall]

    py, px = np.mgrid[0:H, 0:W]
    mx = ox + (px + 0.5) * res
    my = oy + (H - 0.5 - py) * res

    def analyse(name, box, refdir):
        x0, x1, y0, y1 = box
        mw = (X > x0) & (X < x1) & (Y > y0) & (Y < y1)
        u = np.array(refdir, float) / np.linalg.norm(refdir)
        P = np.stack([X[mw], Y[mw]], 1)
        Pc = P - P.mean(0)
        wv, vv = np.linalg.eigh(np.cov(Pc.T))
        u2 = vv[:, np.argmax(wv)]
        u2 = u2 if u2 @ u > 0 else -u2
        n = np.array([-u2[1], u2[0]])
        t_p = P @ n
        s_p = P @ u2
        mf = (mx > x0) & (mx < x1) & (my > y0) & (my < y1) & free
        F = np.stack([mx[mf], my[mf]], 1)
        t_f = F @ n
        s_f = F @ u2
        print('\n=== %s: 墙点 %d, 自由格 %d' % (name, len(P), len(F)))
        print('   s[m]   pcd 左墙/右墙 t    走廊中心    pgm 自由带中心 t   偏移(自由-墙)')
        offs = []
        for s0 in np.arange(np.percentile(s_p, 3) + 1, np.percentile(s_p, 97) - 1, 3.0):
            ms = np.abs(s_p - s0) < 0.3
            mfs = np.abs(s_f - s0) < 0.3
            if ms.sum() < 40 or mfs.sum() < 40:
                continue
            h, e = np.histogram(t_p[ms], bins=np.arange(t_p[ms].min(), t_p[ms].max() + 0.05, 0.05))
            idx = np.argsort(h)[::-1]
            peaks = []
            for i in idx:
                c = (e[i] + e[i + 1]) / 2
                if all(abs(c - q) > 0.5 for q, _ in peaks):
                    peaks.append((c, h[i]))
                if len(peaks) == 2:
                    break
            peaks.sort()
            if len(peaks) < 2:
                continue
            wall_c = (peaks[0][0] + peaks[1][0]) / 2
            free_c = float(np.median(t_f[mfs]))
            off = free_c - wall_c
            offs.append(off)
            print('  %5.1f   %+6.2f / %+6.2f   %+7.2f      %+8.2f        %+6.2f'
                  % (s0, peaks[0][0], peaks[1][0], wall_c, free_c, off))
        if offs:
            print('  --> %s 偏移: median %+.2f m, |max| %.2f m'
                  % (name, float(np.median(offs)), float(np.max(np.abs(offs)))))

    analyse('arm1 (202 段)', (2, 32, -21, -2), (1, -0.5))
    analyse('arm2 (227 段)', (30, 50, -23, -2), (1, 1))


if __name__ == '__main__':
    main()
