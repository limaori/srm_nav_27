"""Build the 227_1113pro simulation without editing the source map or real config."""
import hashlib
import json
import shutil
import struct
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from PIL import Image

SOURCE = Path(__file__).resolve().parent
ROOT = SOURCE.parents[2]
OUT = ROOT / 'log_diag/sim_tdt_smoother_20261117'
OUT.mkdir(parents=True, exist_ok=True)
MAP = ROOT / 'maps/227_1113pro/227_1113pro.yaml'
cfg = yaml.safe_load(MAP.read_text())
r = cfg['resolution']
ox, oy, yaw = cfg['origin']
assert yaw == 0
image_path = MAP.parent / cfg['image']
image = np.flipud(np.asarray(Image.open(image_path)))
occ = (255 - image.astype(float)) / 255 > cfg['occupied_thresh']
h, w = occ.shape
triangles = []


def quad(a, b, c, d):
    triangles.extend([(a, b, c), (a, c, d)])


for j, i in zip(*np.where(occ)):
    x, y, z = ox + i*r, oy + j*r, 0.8
    a, b, c, d = (x,y,0), (x+r,y,0), (x+r,y+r,0), (x,y+r,0)
    A, B, C, D = (x,y,z), (x+r,y,z), (x+r,y+r,z), (x,y+r,z)
    quad(A,B,C,D)
    quad(d,c,b,a)
    if j == 0 or not occ[j-1,i]: quad(a,b,B,A)
    if i == w-1 or not occ[j,i+1]: quad(b,c,C,B)
    if j == h-1 or not occ[j+1,i]: quad(c,d,D,C)
    if i == 0 or not occ[j,i-1]: quad(d,a,A,D)
mesh = OUT / 'occupied_cells.stl'
with mesh.open('wb') as f:
    f.write(b'227_1113pro occupied cells extruded to 0.8 m'.ljust(80, b' '))
    f.write(struct.pack('<I', len(triangles)))
    for tri in triangles:
        a,b,c = np.asarray(tri)
        normal = np.cross(b-a,c-a)
        normal /= np.linalg.norm(normal)
        f.write(struct.pack('<12fH', *normal, *a, *b, *c, 0))
tree = ET.parse(ROOT / 'src/srm27_gazebo_simulator/worlds/srm_empty.sdf')
world = tree.getroot().find('world')
model = ET.SubElement(world, 'model', name='map_obstacles')
ET.SubElement(model, 'static').text = 'true'
link = ET.SubElement(model, 'link', name='walls')
for kind in ['collision', 'visual']:
    element = ET.SubElement(link, kind, name='map_'+kind)
    geometry = ET.SubElement(element, 'geometry')
    m = ET.SubElement(geometry, 'mesh')
    ET.SubElement(m, 'uri').text = 'file://' + str(mesh)
    ET.SubElement(m, 'scale').text = '1 1 1'
ET.indent(tree, space='  ')
tree.write(OUT/'world.sdf', encoding='utf-8', xml_declaration=True)
sim = yaml.safe_load((ROOT/'src/srm27_gazebo_simulator/config/srm_sim.yaml').read_text())
sim.update(world='srm_empty', world_sdf=str(OUT/'world.sdf'), gui=False)
sim['robot'].update(x=0.236, y=-0.110, yaw=0.0)
(OUT/'sim.yaml').write_text(yaml.safe_dump(sim, sort_keys=False))

# Keep simulation sensor wiring; transplant the handoff's navigation algorithms
# and costmap geometry. Remove the real IMU's fixed yaw offset by retaining the
# simulation fake_vel_transform section.
nav = yaml.safe_load((ROOT/'src/bringup/srm27_nav_bringup/config/simulation/current/nav2_params_srm_fakeyaw.yaml').read_text())
real = yaml.safe_load((ROOT/'log_diag/tdt_smoother_ab_20261116/nav2_params_tdt_smoother.yaml').read_text())
for name in ['controller_server', 'planner_server', 'smoother_server', 'bt_navigator', 'global_costmap', 'local_costmap']:
    nav[name] = real[name]
for name in ['global_costmap', 'local_costmap']:
    voxel = nav[name][name]['ros__parameters']['intensity_voxel_layer']
    for source in voxel['observation_sources'].split():
        voxel[source]['sensor_frame'] = 'front_mid360'
nav['planner_server']['ros__parameters']['GridBased'] = {
    'plugin': 'tdt_nav_kit::TdtGlobalPlanner', 'occupancy_threshold': 240,
    'treat_unknown_as_obstacle': True, 'save_point_max_radius': 0.4,
    'path_resample_step': 0.20,
}
nav['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(ROOT/'src/bringup/srm27_nav_bringup/nav2_bt/navigate_to_pose_w_smoothing.xml')


def sim_time(obj):
    if isinstance(obj, dict):
        for key, value in obj.items():
            if key == 'use_sim_time': obj[key] = True
            else: sim_time(value)


sim_time(nav)
(OUT/'nav2_params_sim_tdt_fakeyaw.yaml').write_text(yaml.safe_dump(nav, sort_keys=False))
for threshold in [248,250]:
    nav['planner_server']['ros__parameters']['GridBased']['occupancy_threshold']=threshold
    (OUT/f'nav2_params_threshold{threshold}_fakeyaw.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
shutil.copyfile(ROOT/'src/bringup/srm27_nav_bringup/config/real/current/speed_limits_slow.yaml',OUT/'speed_limits.yaml')
meta = dict(map_yaml=str(MAP), map_sha256=hashlib.sha256(image_path.read_bytes()).hexdigest(),
            dimensions=[w,h], resolution=r, origin=cfg['origin'], spawn=[0.236,-0.110,0.0],
            goal=[-1.48,2.02], obstacle_height_m=0.8, occupied_cells=int(occ.sum()),
            ros_domain_id=87, odometry='Gazebo ground truth', occupancy_threshold=150,
            speed_limit_mps=1.0, minco_required_clearance_m=.35,
            default_nav_params=str(SOURCE/'nav2_params_fixed_fakeyaw.yaml'),
            limitations=['Unknown cells are not extruded.', 'No historical live obstacle replay.',
                         'Synthetic vertical walls; no real localization errors.'])
(OUT/'scene_manifest.json').write_text(json.dumps(meta, indent=2))
print(json.dumps(meta, indent=2))

# Save the final accepted configuration beside the generated scene as well.
shutil.copyfile(SOURCE/'nav2_params_fixed_fakeyaw.yaml',OUT/'nav2_params_fixed_fakeyaw.yaml')
