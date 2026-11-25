#!/usr/bin/env python3
"""用当前 Nav2 参数在新图上复算全局路径形状（ThetaStar 的代价模型）。

目的：验证"直线贴边是不是膨胀半径=内接半径导致代价场没有居中梯度"。

用法: python3 .diag_tmp/sim_global_plan.py maps/xxx/xxx.yaml --robot-radius 0.30
"""
import argparse
import heapq
import os
import sys

import numpy as np
import yaml
from scipy import ndimage

sys.path.insert(0, '.diag_tmp')
from map_clearance_report import load_pgm, write_png

W_EUC, W_TRAV = 1.0, 10.0
NB = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
      (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]


def astar(trav, cost, start, goal):
    H, W = trav.shape
    g = np.full((H, W), np.inf, np.float64)
    prev = np.full((H, W), -1, np.int32)
    g[start] = 0
    h = lambda r, c: np.hypot(r - goal[0], c - goal[1])
    pq = [(h(*start), 0.0, start[0] * W + start[1])]
    flat_prev = lambda i: (i // W, i % W)
    while pq:
        _, gv, i = heapq.heappop(pq)
        r, c = flat_prev(i)
        if gv > g[r, c] + 1e-6:
            continue
        if (r, c) == goal:
            break
        for dr, dc, step in NB:
            rr, cc = r + dr, c + dc
            if not (0 <= rr < H and 0 <= cc < W) or not trav[rr, cc]:
                continue
            ng = gv + step * (W_EUC + W_TRAV * cost[rr, cc])
            if ng < g[rr, cc] - 1e-6:
                g[rr, cc] = ng
                prev[rr, cc] = r * W + c
                heapq.heappush(pq, (ng + h(rr, cc), ng, rr * W + cc))
    if not np.isfinite(g[goal]):
        return None
    path = [goal]
    while path[-1] != start:
        i = prev[path[-1]]
        path.append((i // W, i % W))
    return np.array(path[::-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yaml_path')
    ap.add_argument('--robot-radius', type=float, default=0.30)
    ap.add_argument('--out', default='.diag_tmp/mapview')
    args = ap.parse_args()

    m = yaml.safe_load(open(args.yaml_path, encoding='utf-8'))
    res = float(m['resolution'])
    ox, oy = float(m['origin'][0]), float(m['origin'][1])
    a = load_pgm(os.path.join(os.path.dirname(args.yaml_path), m['image']))
    H, W = a.shape
    free = a >= 250
    occ = a <= 100
    dt = ndimage.distance_transform_edt(~occ) * res

    # 端点：取自由区直径的两端
    lbl, _ = ndimage.label(free)
    sizes = ndimage.sum(free, lbl, range(1, lbl.max() + 1))
    main_lbl = int(np.argmax(sizes)) + 1
    r = args.robot_radius
    trav = free & (dt >= r)          # 现状：253 硬阻挡带 = 内接半径
    ys, xs = np.nonzero(trav & (lbl == main_lbl))
    far = np.argmax(dt[ys, xs])
    s = (int(ys[far]), int(xs[far]))
    d2 = (ys - s[0]) ** 2 + (xs - s[1]) ** 2
    t = np.argmax(d2)
    goal = (int(ys[t]), int(xs[t]))
    print(f"端点: start=(row{s[0]},col{s[1]}) goal=(row{goal[0]},col{goal[1]})  可通行格 {trav.sum()}")

    def to_world(p):
        return ox + (p[:, 1] + 0.5) * res, oy + (H - 0.5 - p[:, 0]) * res

    # 现状：膨胀带外代价 0（inflation_radius == robot_radius -> 衰减带宽度为 0）
    cost_now = np.where(dt < r, 253.0, 0.0)
    p_now = astar(trav, cost_now, s, goal)

    # 对照：把膨胀半径放大到 1.0 m（新图最窄 1.3 m 仍能过），做出居中梯度
    R_INF, SCALE = 1.0, 3.0
    cost_wide = np.where(dt <= r, 253.0,
                         253.0 * np.exp(-SCALE * (dt - r)))
    p_wide = astar(trav, cost_wide, s, goal)

    for name, p in (('现状 inflation=0.30', p_now), ('对照 inflation=1.00', p_wide)):
        if p is None:
            print(f"{name}: 无解")
            continue
        c = dt[p[:, 0], p[:, 1]]
        L = np.sum(np.hypot(*np.diff(p, axis=0).T)) * res
        print(f"{name}: 长度 {L:.2f} m  净空 min {c.min():.2f}  p10 {np.percentile(c,10):.2f} "
              f"中位 {np.median(c):.2f} m")

    # 画图
    os.makedirs(args.out, exist_ok=True)
    rgb = np.full((H, W, 3), 245, np.uint8)
    rgb[free] = (255, 255, 255)
    band = free & (dt < r)
    rgb[band] = (215, 215, 215)
    rgb[occ] = (40, 40, 40)
    for p, col in ((p_now, (220, 30, 30)), (p_wide, (30, 90, 220))):
        if p is None:
            continue
        for dr in (-1, 0, 1):
            rr = np.clip(p[:, 0] + dr, 0, H - 1)
            rgb[rr, p[:, 1]] = col
    out = os.path.join(args.out, os.path.basename(args.yaml_path).replace('.yaml', '_plan_ab.png'))
    write_png(out, rgb)
    print(f"{out}  (红=现状参数, 蓝=放大膨胀半径, 灰=净空<robot_radius 的贴墙带)")


if __name__ == '__main__':
    main()
