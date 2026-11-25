#!/usr/bin/env python3
"""栅格图净空体检：走廊宽度分布 / 卡点清单 / 净空热力图。

用法:
  python3 .diag_tmp/map_clearance_report.py maps/227to219_1125/227to219_1125.yaml \
      --robot-radius 0.30 --out .diag_tmp/mapview
"""
import argparse
import os
import struct
import zlib

import numpy as np
import yaml
from scipy import ndimage


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


def write_png(path, arr):
    h, w, _ = arr.shape
    raw = b''.join(b'\x00' + arr[r].tobytes() for r in range(h))

    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)

    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n'
                + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def grid_to_world(rows, cols, res, ox, oy, H):
    # pgm 第 0 行 = 世界 y 最大
    x = ox + (cols + 0.5) * res
    y = oy + (H - rows - 0.5) * res
    return x, y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml_path')
    ap.add_argument('--robot-radius', type=float, default=0.30)
    ap.add_argument('--inflation', type=float, default=0.30)
    ap.add_argument('--out', default='.diag_tmp/mapview')
    args = ap.parse_args()

    m = yaml.safe_load(open(args.yaml_path, encoding='utf-8'))
    pgm_path = os.path.join(os.path.dirname(args.yaml_path), m['image'])
    res = float(m['resolution'])
    ox, oy = float(m['origin'][0]), float(m['origin'][1])
    a = load_pgm(pgm_path)
    H, W = a.shape

    # trinary: free≈254, unknown≈205, occupied≈0
    free = a >= 250
    occ = a <= 100
    unk = ~free & ~occ
    print(f"图像 {W}x{H}  res={res}  origin=({ox:.2f},{oy:.2f})")
    print(f"自由 {free.mean()*100:.1f}%  / 占用 {occ.mean()*100:.1f}%  / 未知 {unk.mean()*100:.1f}%")

    # 障碍（含未知边界）距离场，单位米
    dt = ndimage.distance_transform_edt(~(occ | unk)) * res
    clear = np.where(free, dt, 0.0)

    # 只看"机器人能站进去"的自由格
    walk = clear
    print("\n=== 自由空间净空（到最近障碍/未知的距离）===")
    for q in (5, 10, 25, 50, 75, 90, 99):
        print(f"  p{q:>2} = {np.percentile(walk[free], q):.2f} m")
    print(f"  max = {walk[free].max():.2f} m")

    r = args.robot_radius
    band = free & (clear < r)
    print(f"\n净空 < robot_radius({r:.2f}) 的自由格: {band.sum()} "
          f"({band.sum()/free.sum()*100:.2f}% 的自由区)，即规划器必须绕开的“贴墙带”")
    for th in (0.35, 0.40, 0.45, 0.50):
        b = free & (clear < th)
        print(f"  净空 < {th:.2f} m: {b.sum():>6d} 格  ({b.sum()*res*res:6.2f} m²)")

    # 连通域：过滤掉零碎小自由块（噪声）
    lbl, n = ndimage.label(free)
    sizes = ndimage.sum(free, lbl, range(1, n + 1))
    big = np.isin(lbl, np.nonzero(sizes > 200)[0] + 1)
    print(f"\n连通自由域 {n} 个，>200 格的主域 {big.sum()/free.sum()*100:.1f}% 的自由区")

    # 窄口检测：对主域做骨架化近似 —— 沿每行/每列找“墙-墙”跨度
    print("\n=== 逐行/逐列墙-墙跨度（<1.2 m 的视为窄道/门）===")
    spans = []
    for name, arr in (('列', free.T),):
        for i in range(arr.shape[0]):
            row = arr[i]
            idx = np.nonzero(row)[0]
            if idx.size < 2:
                continue
            # 连续段
            splits = np.nonzero(np.diff(idx) > 1)[0]
            starts = np.r_[0, splits + 1]
            ends = np.r_[splits, idx.size - 1]
            for s, e in zip(starts, ends):
                length = (idx[e] - idx[s] + 1) * res
                if 0.6 < length < 1.2:
                    spans.append((i, idx[s], idx[e], length))
    if spans:
        spans = np.array(spans, dtype=float)
        print(f"  窄跨度样本 {len(spans)} 条，跨度中位数 {np.median(spans[:,3]):.2f} m")
        # 聚类成“门”位置
        pts = []
        for row, c0, c1, L in spans[::max(1, len(spans)//400)]:
            x0, y = grid_to_world(row, c0, res, ox, oy, H)
            x1, _ = grid_to_world(row, c1, res, ox, oy, H)
            pts.append((round((x0 + x1) / 2, 2), round(y, 2), round(L, 2)))
        pts.sort()
        print("  抽样位置 (x, y, 跨度m):")
        for p in pts[:40]:
            print("   ", p)
    else:
        print("  没有 <1.2 m 的跨度")

    # 热力图
    os.makedirs(args.out, exist_ok=True)
    vmax = 1.2
    t = np.clip(clear / vmax, 0, 1)
    rgb = np.zeros((H, W, 3), np.uint8)
    # 障碍黑、未知灰、自由按净空蓝->绿->黄->红（越窄越红）
    rgb[occ] = (30, 30, 30)
    rgb[unk] = (120, 120, 120)
    r_ch = np.clip(1.6 * t - 0.2, 0, 1)
    g_ch = np.clip(1.6 - 1.6 * t, 0, 1)
    b_ch = np.clip(0.6 - 1.2 * t, 0, 1)
    sel = free
    rgb[sel] = np.stack([(r_ch[sel] * 255), (g_ch[sel] * 200), (b_ch[sel] * 255)], -1).astype(np.uint8)
    # 贴墙带描白
    rgb[band] = (255, 255, 255)
    out = os.path.join(args.out, os.path.basename(args.yaml_path).replace('.yaml', '_clearance.png'))
    write_png(out, rgb)
    print(f"\n净空热力图(白=净空<{r:.2f}m 的贴墙带): {out}")


if __name__ == '__main__':
    main()
