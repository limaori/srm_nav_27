#!/usr/bin/env python3
"""把两张 2D 栅格图（pgm+yaml）按世界坐标并排渲染，便于肉眼对比。

用法：
  python3 .diag_tmp/compare_pgm.py --a maps/202to227/202to227.yaml \
      --b maps/202to227_replay/202to227_replay.yaml --out .diag_tmp/mapview/compare.png
"""
import argparse
import struct
import zlib

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


def write_png(path, arr):
    h, w, _ = arr.shape
    raw = b''.join(b'\x00' + arr[r].tobytes() for r in range(h))
    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n'
                + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def load_map(yaml_path):
    m = yaml.safe_load(open(yaml_path, encoding='utf-8'))
    pgm = yaml_path.rsplit('/', 1)[0] + '/' + m['image']
    a = load_pgm(pgm)
    return a, float(m['resolution']), float(m['origin'][0]), float(m['origin'][1]), pgm


def to_canvas(a, res, ox, oy, X0, Y1, W, H, res_out):
    """把栅格图重采样到公共画布（画布左上角=世界 (X0, Y1)）。"""
    h, w = a.shape
    canvas = np.full((H, W), 205, np.uint8)
    # 每个画布像素 -> 世界坐标 -> 该图的像素
    py, px = np.mgrid[0:H, 0:W]
    wx = X0 + (px + 0.5) * res_out
    wy = Y1 - (py + 0.5) * res_out
    ci = np.floor((wx - ox) / res).astype(int)
    cj = np.floor((Y1_top(a, res, ox, oy) - wy) / res).astype(int)
    valid = (ci >= 0) & (ci < w) & (cj >= 0) & (cj < h)
    canvas[valid] = a[cj[valid], ci[valid]]
    return canvas


def Y1_top(a, res, ox, oy):
    return oy + a.shape[0] * res


def colour(a):
    rgb = np.zeros(a.shape + (3,), np.uint8)
    rgb[a >= 250] = [245, 245, 245]      # free
    rgb[(a >= 200) & (a < 250)] = [128, 128, 128]   # unknown
    rgb[a == 0] = [0, 0, 0]              # occupied
    rgb[(a > 0) & (a < 200) & (a != 205)] = [128, 128, 128]
    return rgb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--a', required=True)
    ap.add_argument('--b', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--res', type=float, default=0.10)
    args = ap.parse_args()

    A = load_map(args.a)
    B = load_map(args.b)
    xs, ys = [], []
    for a, res, ox, oy, _ in (A, B):
        xs += [ox, ox + a.shape[1] * res]
        ys += [oy, oy + a.shape[0] * res]
    X0, X1, Y0, Y1 = min(xs), max(xs), min(ys), max(ys)
    W = int((X1 - X0) / args.res)
    H = int((Y1 - Y0) / args.res)
    ca = to_canvas(A[0], A[1], A[2], A[3], X0, Y1, W, H, args.res)
    cb = to_canvas(B[0], B[1], B[2], B[3], X0, Y1, W, H, args.res)
    gap = np.full((H, 8, 3), 60, np.uint8)
    both = np.concatenate([colour(ca), gap, colour(cb)], axis=1)
    write_png(args.out, both)
    for name, a in (('A', A), ('B', B)):
        img = a[0]
        free = (img >= 250).mean() * 100
        occ = (img == 0).mean() * 100
        unk = ((img > 0) & (img < 250) & (img != 0)).mean() * 100
        print('%s: %s  %dx%d  free %.1f%%  occupied %.1f%%  unknown %.1f%%'
              % (name, a[4], img.shape[1], img.shape[0], free, occ, unk))
    print('saved', args.out, both.shape)


if __name__ == '__main__':
    main()
