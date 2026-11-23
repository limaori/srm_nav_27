#!/usr/bin/env python3
"""Evaluate recorded robot centers against the exact occupied-cell square faces."""
import argparse,csv,json,math,hashlib
from pathlib import Path
import numpy as np,yaml
from PIL import Image
from scipy.spatial import cKDTree
P=argparse.ArgumentParser();P.add_argument('run_dir');P.add_argument('--radius',type=float,default=.33);args=P.parse_args();SCENE=Path(__file__).resolve().parent;RUN=Path(args.run_dir).resolve();m=yaml.safe_load((SCENE.parent/'227_1116_remap.yaml').read_text());r=m['resolution'];ox,oy,_=m['origin'];image=SCENE.parent/m['image'];a=np.flipud(np.array(Image.open(image)));j,i=np.where(a==0);centers=np.c_[ox+(i+.5)*r,oy+(j+.5)*r];tree=cKDTree(centers)
records=list(csv.DictReader((RUN/'ground_truth.csv').open()));results=json.loads((RUN/'results.json').read_text());diagnostics=json.loads((RUN/'diagnostics.json').read_text());summary={'source_map_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),'envelope_radius_m':args.radius,'method':'Distance to occupied-cell axis-aligned square faces; subtract robot envelope radius; interpolate recorded XY segments at <=2mm spacing. This is a geometric replay check, not contact-sensor evidence.','legs':[]}
def distances(points):
 nearest,_=tree.query(points)
 # Every potentially nearer square has its center within this band, since
 # distance(center,point) - half_cell_diagonal bounds its face distance below.
 candidates=tree.query_ball_point(points,nearest+r*math.sqrt(2))
 return np.array([np.linalg.norm(np.maximum(np.abs(centers[idx]-p)-r/2,0),axis=1).min() for p,idx in zip(points,candidates)])
for res in results:
 rows=[p for p in records if p['leg']==res['name']]
 if not rows or 'endpoint_error_m' not in res:
  summary['legs'].append({'name':res['name'],'status':res['status'],'gt_samples':len(rows),'envelope_margin_m':None})
  continue
 points=np.array([[float(p['x']),float(p['y'])] for p in rows]);dense=[]
 for p,q in zip(points,points[1:]):
  n=max(1,math.ceil(np.linalg.norm(q-p)/.002));dense.extend(p+(q-p)*u for u in np.linspace(0,1,n+1))
 dense=np.array(dense) if dense else points
 # k=16 is ample for a first candidate; query a guaranteed enclosing band at
 # the minimum point to include every square that could improve that candidate.
 values=distances(dense);best=int(np.argmin(values));point=dense[best];d,_=tree.query(point);candidates=tree.query_ball_point(point,d+r*math.sqrt(2));exact=float(np.linalg.norm(np.maximum(np.abs(centers[candidates]-point)-r/2,0),axis=1).min())
 ds=[s for s in diagnostics if s['leg']==res['name']];entry={'name':res['name'],'status':res['status'],'endpoint_error_m':res['endpoint_error_m'],'max_gt_speed_mps':res['max_gt_speed_mps'],'wall_s':res['wall_s'],'actual_distance_m':res['actual_distance_m'],'center_clearance_m':exact,'envelope_margin_m':exact-args.radius,'minimum_xy':point.tolist(),'gt_samples':len(rows),'nav2_recoveries':res['recoveries'],'internal_recovery_max_enter_count':max([int(d.get('recovery_enter_count',0)) for d in ds],default=0)};summary['legs'].append(entry)
measured=[s for s in summary['legs'] if s['envelope_margin_m'] is not None]
summary['all_succeeded']=bool(summary['legs']) and all(s['status']=='SUCCEEDED' for s in summary['legs'])
summary['minimum_envelope_margin_m']=min((s['envelope_margin_m'] for s in measured),default=None)
summary['maximum_observed_speed_mps']=max((s['max_gt_speed_mps'] for s in measured),default=None)
summary['minimum_margin_goal_m']=.05
summary['margin_goal_met']=bool(measured) and len(measured)==len(summary['legs']) and summary['minimum_envelope_margin_m']>=.05
(RUN/'clearance_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
