#!/usr/bin/env python3
"""Build a reversible map-derived simulation; the source PGM remains unchanged."""
from pathlib import Path
import math,struct,json,copy,heapq,xml.etree.ElementTree as ET
import yaml,numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt,label
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).resolve().parent
mp=OUT.parent/'227_1116_remap.yaml';cfg=yaml.safe_load(mp.read_text());r=float(cfg['resolution']);ox,oy,oa=cfg['origin'];assert abs(oa)<1e-8
im=np.flipud(np.array(Image.open(mp.parent/cfg['image'])));occ=im==0;h,w=occ.shape
# One static triangle mesh avoids thousands of separate Gazebo collision objects.
tris=[]
def quad(a,b,c,d):tris.extend([(a,b,c),(a,c,d)])
for j,i in zip(*np.where(occ)):
 x=ox+i*r;y=oy+j*r;z=.8
 a=(x,y,0);b=(x+r,y,0);c=(x+r,y+r,0);d=(x,y+r,0);A=(x,y,z);B=(x+r,y,z);C=(x+r,y+r,z);D=(x,y+r,z)
 quad(A,B,C,D);quad(d,c,b,a)
 if j==0 or not occ[j-1,i]:quad(a,b,B,A)
 if i==w-1 or not occ[j,i+1]:quad(b,c,C,B)
 if j==h-1 or not occ[j+1,i]:quad(c,d,D,C)
 if i==0 or not occ[j,i-1]:quad(d,a,A,D)
mesh=OUT/'occupied_cells.stl'
with mesh.open('wb') as f:
 f.write(b'SRM occupancy-map extrusion: resolution 0.05m, height 0.8m'.ljust(80,b' '));f.write(struct.pack('<I',len(tris)))
 for tri in tris:
  a,b,c=np.asarray(tri);n=np.cross(b-a,c-a);n=n/np.linalg.norm(n);f.write(struct.pack('<12fH',*n,*a,*b,*c,0))
base=ET.parse(ROOT/'src/srm27_gazebo_simulator/worlds/srm_empty.sdf');world=base.getroot().find('world')
model=ET.SubElement(world,'model',name='map_obstacles');ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='walls')
for kind in ['collision','visual']:
 e=ET.SubElement(link,kind,name='map_'+kind);g=ET.SubElement(e,'geometry');me=ET.SubElement(g,'mesh');ET.SubElement(me,'uri').text='file://'+str(mesh);ET.SubElement(me,'scale').text='1 1 1'
 if kind=='visual':
  ma=ET.SubElement(e,'material');ET.SubElement(ma,'ambient').text='0.35 0.50 0.68 1';ET.SubElement(ma,'diffuse').text='0.35 0.50 0.68 1'
ET.indent(base,space='  ');base.write(OUT/'world.sdf',encoding='utf-8',xml_declaration=True)
clear=distance_transform_edt(im==254)*r;labs,n=label(clear>=.42);sizes=np.bincount(labs.ravel());sizes[0]=0;comp=sizes.argmax();ys,xs=np.where(labs==comp);k=np.argmax(clear[ys,xs]);spawn=[float(ox+(xs[k]+.5)*r),float(oy+(ys[k]+.5)*r)]
sim=yaml.safe_load((ROOT/'src/srm27_gazebo_simulator/config/srm_sim.yaml').read_text());sim.update(world='srm_empty',world_sdf=str(OUT/'world.sdf'),gui=False);sim['robot'].update(x=spawn[0],y=spawn[1],yaw=0.0);(OUT/'sim.yaml').write_text(yaml.safe_dump(sim,sort_keys=False))
# 2026-11-24：原 tdt_nav_kit 拆包并入 src/planner；这里按 bringup 的同口径注入两段插件参数。
nav=yaml.safe_load((ROOT/'src/bringup/srm27_nav_bringup/config/simulation/current/nav2_params_srm_fakeyaw.yaml').read_text())
nav['planner_server']['ros__parameters']['GridBased']={'plugin':'srm27_nav_planner::YAstarGlobalPlanner','occupancy_threshold':252,'treat_unknown_as_obstacle':False,'mask_num_layers':1,'cost_inflate_radius':1.0,'simplify_epsilon':0.1,'save_point_max_radius':0.4,'goal_snap_max_radius':0.4,'path_resample_step':0.20,'orientation_mode':'tangent'}
nav['smoother_server']['ros__parameters'].update({'smoother_plugins':['minimum_snap_smoother'],'minimum_snap_smoother':{'plugin':'srm27_nav_smoother::MinimumSnapSmoother','occupancy_threshold':252,'treat_unknown_as_obstacle':False,'order':5,'max_dx':3,'strict_collision':False,'backend':'path','collision_iter':6,'norm_time':True,'max_corridor_range':2.5,'corridor_shrink':0.0,'max_speed':3.0,'max_acc':1.0,'path_resample_step':0.30,'path_resample_max_points':2000,'fallback_to_resampled':True,'corner_round_cut':0.40,'corner_round_min_turn_deg':25.0,'corner_round_sample_step':0.20,'init_vel_from_odom':False,'odom_topic':'odom','init_vel_max_age':0.30,'min_points':2,'orientation_mode':'tangent','publish_sfc':True,'sfc_topic':'~/sfc_corridor','sfc_line_width':0.02}})
nav['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(ROOT/'src/bringup/srm27_nav_bringup/nav2_bt/navigate_to_pose_w_smoothing.xml')
(OUT/'nav2_params_srm_tdt_fakeyaw.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
# Snap only test target choices to a known free cell with >=0.5 m clearance.
def snap(x,y):
 ys,xs=np.where((labs==comp)&(clear>=.50));ds=(ox+(xs+.5)*r-x)**2+(oy+(ys+.5)*r-y)**2;k=np.argmin(ds);return [round(float(ox+(xs[k]+.5)*r),4),round(float(oy+(ys[k]+.5)*r),4)]
goals=[{'name':'short_diagonal','xy':snap(spawn[0]-.9,spawn[1]-.9)},{'name':'cross_room','xy':snap(.245,.585)},{'name':'around_obstacles','xy':snap(-2.955,-.565)},{'name':'return_spawn','xy':spawn}]
meta={'map_yaml':str(mp),'obstacle_cells':int(occ.sum()),'mesh_triangles':len(tris),'obstacle_height_m':.8,'unknown_cells_extruded':False,'spawn_xy':spawn,'spawn_clearance_m':float(clear[ys[k],xs[k]]),'goals':goals,'planner':'YAstar','smoother':'MinimumSnapSmoother','controller':'MincoMpcController','odometry':'Gazebo ground truth','goal_tolerance_m':.5}
# Independently check source-map traversability for each target and generate a preview.
meta['spawn_clearance_m']=float(clear[int((spawn[1]-oy)/r),int((spawn[0]-ox)/r)])
(OUT/'scene_manifest.json').write_text(json.dumps(meta,indent=2))
from PIL import ImageDraw
preview=Image.fromarray(np.flipud(im)).convert('RGB').resize((w*3,h*3),Image.NEAREST);dr=ImageDraw.Draw(preview)
for label_,(x,y),color in [('S',spawn,'blue')]+[(str(i+1),g['xy'],'red') for i,g in enumerate(goals)]:
 px=(x-ox)/r*3;py=(h-(y-oy)/r)*3;dr.ellipse((px-6,py-6,px+6,py+6),fill=color);dr.text((px+8,py-8),label_,fill=color)
preview.save(OUT/'scene_preview.png');print(json.dumps(meta,indent=2))
