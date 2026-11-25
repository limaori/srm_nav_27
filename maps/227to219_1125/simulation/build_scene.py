#!/usr/bin/env python3
"""给 227to219_1125 生成 Gazebo 仿真场景（世界 + 机器人出生点 + 导航参数）。

产出（都写在本目录，不动源图和实车配置）：
  occupied_cells.stl        栅格障碍按 0.8 m 挤出成三角网（只留贴自由区的壳，
                            图里 91% 是占用格，全量挤出会有几十万格/上百 MB）
  world.sdf                 srm_empty + 上面那张网
  sim.yaml                  机器人出生点（地图系坐标）
  nav2_params_yastar.yaml   仿真参数 + YAstar 规划器段（单一来源，直接改这里调参）
  scene_manifest.json       出生点/目标点/连通性自检结果
  scene_preview.png         预览图

用法:
  python3 maps/227to219_1125/simulation/build_scene.py
"""
import heapq
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
MAP = ROOT / 'maps/227to219_1125/227to219_1125.yaml'
OBSTACLE_HEIGHT = 0.8
SHELL_CELLS = 6          # 只保留离自由区 6 格（30 cm）以内的占用格 -> 封闭外壳
SPAWN_MIN_CLEARANCE = 0.45
GOAL_MIN_CLEARANCE = 0.50

cfg = yaml.safe_load(MAP.read_text(encoding='utf-8'))
res = float(cfg['resolution'])
ox, oy, oyaw = cfg['origin']
assert abs(oyaw) < 1e-9, '本工具的网格挤出假设 origin 的 yaw = 0'

image = np.flipud(np.asarray(Image.open(MAP.parent / cfg['image'])))
occ = (255.0 - image.astype(float)) / 255.0 > float(cfg['occupied_thresh'])
free = image >= 250
if cfg.get('negate'):
    occ, free = free, ~free
h, w = occ.shape
clear = ndimage.distance_transform_edt(free) * res          # 自由格到最近障碍的距离
clear = np.where(free, clear, 0.0)


def cell_to_world(j, i):
    return ox + (i + 0.5) * res, oy + (j + 0.5) * res


def world_to_cell(x, y):
    return int((y - oy) / res), int((x - ox) / res)


# ---------------------------------------------------------------- 外壳挤出
shell = occ & ndimage.binary_dilation(free, iterations=SHELL_CELLS)
tris = []


def quad(a, b, c, d):
    tris.append((a, b, c))
    tris.append((a, c, d))


for j, i in zip(*np.where(shell)):
    x, y, z = ox + i * res, oy + j * res, OBSTACLE_HEIGHT
    a, b, c, d = (x, y, 0), (x + res, y, 0), (x + res, y + res, 0), (x, y + res, 0)
    A, B, C, D = (x, y, z), (x + res, y, z), (x + res, y + res, z), (x, y + res, z)
    quad(A, B, C, D)
    quad(d, c, b, a)
    if j == 0 or not shell[j - 1, i]:
        quad(a, b, B, A)
    if i == w - 1 or not shell[j, i + 1]:
        quad(b, c, C, B)
    if j == h - 1 or not shell[j + 1, i]:
        quad(c, d, D, C)
    if i == 0 or not shell[j, i - 1]:
        quad(d, a, A, D)

mesh = OUT / 'occupied_cells.stl'
with mesh.open('wb') as f:
    f.write(b'227to219_1125 occupancy shell (0.8 m, 6-cell skin)'.ljust(80, b' '))
    f.write(struct.pack('<I', len(tris)))
    for tri in tris:
        a, b, c = (np.asarray(t, dtype=float) for t in tri)
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        n = n / norm if norm > 1e-12 else np.zeros(3)
        f.write(struct.pack('<12fH', *n, *a, *b, *c, 0))

base = ET.parse(ROOT / 'src/simulation/srm27_gazebo_simulator/worlds/srm_empty.sdf')
world = base.getroot().find('world')
model = ET.SubElement(world, 'model', name='map_obstacles')
ET.SubElement(model, 'static').text = 'true'
link = ET.SubElement(model, 'link', name='walls')
for kind in ('collision', 'visual'):
    element = ET.SubElement(link, kind, name='map_' + kind)
    geometry = ET.SubElement(element, 'geometry')
    mesh_el = ET.SubElement(geometry, 'mesh')
    ET.SubElement(mesh_el, 'uri').text = 'file://' + str(mesh)
    ET.SubElement(mesh_el, 'scale').text = '1 1 1'
    if kind == 'visual':
        material = ET.SubElement(element, 'material')
        ET.SubElement(material, 'ambient').text = '0.35 0.50 0.68 1'
        ET.SubElement(material, 'diffuse').text = '0.35 0.50 0.68 1'
ET.indent(base, space='  ')
base.write(OUT / 'world.sdf', encoding='utf-8', xml_declaration=True)

# ---------------------------------------------------------------- 出生点 + 目标点
labs, _ = ndimage.label(free)
sizes = np.bincount(labs.ravel())
sizes[0] = 0
comp = int(sizes.argmax())
spawnable = (labs == comp) & (clear >= SPAWN_MIN_CLEARANCE)
sj, si = np.unravel_index(np.argmax(np.where(spawnable, clear, -1.0)), clear.shape)
spawn = cell_to_world(sj, si)
spawn_clearance = float(clear[sj, si])

# 可通行图（净空 >= 0.35 m，按 0.05 m 网格 8 邻接测地距离）
trav = (labs == comp) & (clear >= 0.35)
geo = np.full((h, w), np.inf)
geo[sj, si] = 0.0
pq = [(0.0, sj, si)]
nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
while pq:
    d, j, i = heapq.heappop(pq)
    if d > geo[j, i] + 1e-12:
        continue
    for dj, di, step in nbrs:
        jj, ii = j + dj, i + di
        if 0 <= jj < h and 0 <= ii < w and trav[jj, ii]:
            nd = d + step * res
            if nd < geo[jj, ii] - 1e-12:
                geo[jj, ii] = nd
                heapq.heappush(pq, (nd, jj, ii))

reachable = np.isfinite(geo) & (clear >= GOAL_MIN_CLEARANCE)
max_geo = float(geo[reachable].max())
goals = []
for name, frac in (('far_end_a', 0.55), ('far_end_b', 0.80), ('far_end_c', 1.00)):
    target = frac * max_geo
    cand = reachable & (geo >= target - 0.25) & (geo <= target + 0.25)
    if not cand.any():
        continue
    gj, gi = np.unravel_index(np.argmax(np.where(cand, clear, -1.0)), clear.shape)
    goals.append({'name': name, 'xy': [round(v, 4) for v in cell_to_world(gj, gi)],
                  'geodesic_m': round(float(geo[gj, gi]), 2),
                  'clearance_m': round(float(clear[gj, gi]), 2)})
goals.append({'name': 'return_spawn', 'xy': [round(v, 4) for v in spawn],
              'geodesic_m': 0.0, 'clearance_m': round(spawn_clearance, 2)})
(OUT / 'goals.json').write_text(
    json.dumps([{'name': g['name'], 'xy': g['xy']} for g in goals], ensure_ascii=False, indent=2) + '\n',
    encoding='utf-8')

# ---------------------------------------------------------------- sim.yaml
default_sim = yaml.safe_load(
    (ROOT / 'src/simulation/srm27_gazebo_simulator/config/srm_sim.yaml').read_text(encoding='utf-8'))
default_sim.update(world='srm_empty', world_sdf=str(OUT / 'world.sdf'), gui=False)
default_sim['robot'].update(x=float(spawn[0]), y=float(spawn[1]), yaw=0.0)
(OUT / 'sim.yaml').write_text(yaml.safe_dump(default_sim, sort_keys=False), encoding='utf-8')

# ---------------------------------------------------------------- 导航参数（含 YAstar）
nav = yaml.safe_load(
    (ROOT / 'src/bringup/srm27_nav_bringup/config/simulation/current/'
     'nav2_params_srm_fakeyaw.yaml').read_text(encoding='utf-8'))
nav['planner_server']['ros__parameters']['planner_plugins'] = ['GridBased']
nav['planner_server']['ros__parameters']['GridBased'] = {
    # 与 nav_srm_simulation_launch.py 的 YASTAR_PLANNER_OVERRIDE 完全一致，
    # 调参改这里即可，不用再动 launch。
    'plugin': 'srm27_nav_planner::YAstarGlobalPlanner',
    'occupancy_threshold': 252,
    'treat_unknown_as_obstacle': False,
    'mask_num_layers': 4,
    'cost_inflate_radius': 1.0,
    'simplify_epsilon': 0.1,
    'save_point_max_radius': 0.4,
    'goal_snap_max_radius': 0.4,
    'orientation_mode': 'tangent',
    'debug_timing': False,
}


def force_sim_time(node):
    if isinstance(node, dict):
        for key, value in node.items():
            if key == 'use_sim_time':
                node[key] = True
            else:
                force_sim_time(value)


force_sim_time(nav)
(OUT / 'nav2_params_yastar.yaml').write_text(
    yaml.safe_dump(nav, sort_keys=False), encoding='utf-8')

# ---------------------------------------------------------------- manifest / 预览
manifest = {
    'map_yaml': str(MAP),
    'resolution': res,
    'origin': cfg['origin'],
    'dimensions': [w, h],
    'free_fraction': round(float(free.mean()), 4),
    'occupied_cells_total': int(occ.sum()),
    'shell_cells': int(shell.sum()),
    'mesh_triangles': len(tris),
    'obstacle_height_m': OBSTACLE_HEIGHT,
    'spawn_xy': [round(v, 4) for v in spawn],
    'spawn_clearance_m': round(spawn_clearance, 3),
    'map_to_odom': {'x': round(float(spawn[0]), 4), 'y': round(float(spawn[1]), 4), 'yaw': 0.0},
    'geodesic_max_m': round(max_geo, 2),
    'goals': goals,
    'planner': 'YAstar',
    'smoother': 'nav2 SimpleSmoother (sim 默认)',
    'controller': 'MincoMpcController',
    'odometry': 'Gazebo ground truth',
    'limitations': [
        '只挤出贴自由区 6 格内的占用格（外壳），障碍内部为空 —— 机器人进不去内部，行为等价。',
        '没有历史障碍回放、没有真实定位漂移。',
    ],
}
(OUT / 'scene_manifest.json').write_text(
    json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

preview = Image.fromarray(np.flipud(image)).convert('RGB').resize((w * 2, h * 2), Image.NEAREST)
draw = ImageDraw.Draw(preview)
for label, (x, y), colour in ([('S', spawn, 'blue')]
                              + [(str(i + 1), g['xy'], 'red') for i, g in enumerate(goals)]):
    px, py = (x - ox) / res * 2, (h - (y - oy) / res) * 2
    draw.ellipse((px - 7, py - 7, px + 7, py + 7), fill=colour)
    draw.text((px + 9, py - 8), label, fill=colour)
preview.save(OUT / 'scene_preview.png')

print(json.dumps({k: manifest[k] for k in
                  ('free_fraction', 'occupied_cells_total', 'shell_cells', 'mesh_triangles',
                   'spawn_xy', 'spawn_clearance_m', 'geodesic_max_m', 'goals')},
                 ensure_ascii=False, indent=2))
print(f"mesh {mesh.stat().st_size / 1e6:.1f} MB, world {OUT / 'world.sdf'}")
