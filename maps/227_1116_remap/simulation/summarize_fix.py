#!/usr/bin/env python3
from pathlib import Path
import json,csv,hashlib,re
import numpy as np,yaml
from PIL import Image
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
SCENE=Path(__file__).resolve().parent;ROOT=SCENE.parents[2];OLD=ROOT/'log_diag/sim_tdt_227_1116_remap';OUT=ROOT/'log_diag/sim_tdt_227_1116_remap_fix';RUN=OUT/'run1'
old=json.loads((OLD/'comparison.json').read_text());baseline=old['baseline'];results=json.loads((RUN/'results.json').read_text());samples=list(csv.DictReader((RUN/'ground_truth.csv').open()));diags=json.loads((RUN/'diagnostics.json').read_text())
m=yaml.safe_load((SCENE.parent/'227_1116_remap.yaml').read_text());a=np.flipud(np.array(Image.open(SCENE.parent/m['image'])));r=m['resolution'];ox,oy,_=m['origin'];j,i=np.where(a==0);centers=np.c_[ox+(i+.5)*r,oy+(j+.5)*r];tree=cKDTree(centers)
metrics=[]
for result in results:
 res={k:v for k,v in result.items() if k not in ['raw_path','smoothed_path','cross_track_error_rms_m']};ds=[d for d in diags if d['leg']==res['name']];res['internal_recovery_max_enter_count']=max([int(d.get('recovery_enter_count',0)) for d in ds],default=0)
 pts=np.array([[float(t['x']),float(t['y'])] for t in samples if t['leg']==res['name']]);clear=[]
 for p in pts:
  _,idx=tree.query(p,k=8);clear.append(float(np.min(np.linalg.norm(np.maximum(np.abs(centers[idx]-p)-r/2,0),axis=1))))
 res['min_sampled_center_distance_to_extruded_obstacle_m']=min(clear);res['envelope_clearance_m']=min(clear)-.33;metrics.append(res)
summary={'source_map_sha256':hashlib.sha256((SCENE.parent/m['image']).read_bytes()).hexdigest(),'configuration':{'max_linear_speed_mps':2.6,'max_linear_accel_mps2':7.,'goal_xy_tolerance_m':.5,'terminal_tolerance_auto':True,'terminal_exit_margin_m':.15,'reaction_latency_s':.1,'state_timeout_s':.5,'odometry':'Gazebo ground truth','ROS_DOMAIN_ID':37},'fixes':[{'cause':'Terminal stop used extrapolated state while Nav2 goal checking used measured pose; hysteresis retained zero command outside success region.','change':'Use Nav2 measured pose transformed to planning frame; cap both stop entry/exit radii at available goal-checker tolerance.'},{'cause':'Simulation reaction_latency=0 inherited LIO-compatible state_timeout=0.5s and repeatedly rejected safe short stopping trajectories.','change':'Set simulation reaction latency explicitly to 0.1s; keep speed, braking, collision, prediction-window and goal-tolerance checks.'}],'functional_tests':{'controller_gtests_passed':64,'trajectory_validator_gtests_passed':29,'static_checks':'clang_format and clang_tidy could not run: existing per-package .clang-format/.clang-tidy files are missing.'},'baseline':baseline,'after_fix':metrics,'limitations':['One four-goal simulation sequence; actual segment starts differ after prior goals.','Synthetic 0.8m obstacle heights and ground-truth localization.','Endpoint acceptance tolerance stays 0.5m.','Two candidate trajectories were collision-rejected and replanned; no recovery was needed.','Near one obstacle the sampled envelope clearance is about 3cm, below the nominal planning margin of 5cm; no physical-contact sensor was recorded.']}
(OUT/'fix_summary.json').write_text(json.dumps(summary,indent=2))
colors=['#df6b19','#00846b','#8a4bb5','#2463bc'];names=['Short diagonal','Cross room','Around obstacles','Return to spawn'];fig,axes=plt.subplots(1,2,figsize=(13,6),sharex=True,sharey=True)
b=json.loads((OLD/'results.json').read_text());repeat=json.loads((OLD/'baseline_obstacle_repeat/results.json').read_text());bs=list(csv.DictReader((OLD/'ground_truth.csv').open()));rs=list(csv.DictReader((OLD/'baseline_obstacle_repeat/ground_truth.csv').open()));tests=[[(b[0],bs),(b[1],bs),(repeat[0],rs),(b[3],bs)],[(v,samples) for v in results]]
for ax,cases,title in zip(axes,tests,['Before fix: 2.6 m/s','After fix: 2.6 m/s (same 0.5m goal tolerance)']):
 ax.imshow(a,origin='lower',extent=(ox,ox+a.shape[1]*r,oy,oy+a.shape[0]*r),cmap='gray',vmin=0,vmax=254)
 for k,(result,ss) in enumerate(cases):
  pts=np.array([[float(s['x']),float(s['y'])] for s in ss if s['leg']==result['name']]);path=np.array(result.get('smoothed_path',[]));color=colors[k]
  if len(path):ax.plot(*path.T,'--',color=color,alpha=.45,lw=1)
  if len(pts):ax.plot(*pts.T,color=color,lw=2,label=f"{k+1}. {names[k]}: {result['status'].lower()}");ax.scatter(*pts[-1],c=color,s=30)
  ax.scatter(*result['goal'],c=color,marker='x',s=65)
 ax.set_xlim(-3.7,.95);ax.set_ylim(-1.25,2.25);ax.set_aspect('equal');ax.set_xlabel('map x (m)');ax.set_title(title,fontsize=11);ax.legend(loc='upper left',fontsize=7.5);ax.grid(alpha=.12)
axes[0].set_ylabel('map y (m)');fig.suptitle('TDT + MINCO/MPC: terminal-stop and reaction-latency fixes',fontsize=13);fig.text(.5,.02,'Solid: Gazebo ground truth; dashed: initial smoothed path; x: goal. Before-fix obstacle leg was repeated separately.',ha='center',fontsize=8);fig.tight_layout(rect=[0,.10,1,.94]);fig.savefig(OUT/'before_after.png',dpi=170);plt.close(fig)
for x in metrics:print(x['name'],x['status'],'error',x['endpoint_error_m'],'BT recovery',x['recoveries'],'internal recovery',x['internal_recovery_max_enter_count'],'max speed',round(x['max_gt_speed_mps'],3))
