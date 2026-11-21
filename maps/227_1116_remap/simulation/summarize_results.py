#!/usr/bin/env python3
from pathlib import Path
import json,csv,hashlib
import numpy as np,yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
SCENE=Path(__file__).resolve().parent;ROOT=SCENE.parents[2];OUT=ROOT/'log_diag/sim_tdt_227_1116_remap'
def load(d):return json.loads((d/'results.json').read_text()),list(csv.DictReader((d/'ground_truth.csv').open())),json.loads((d/'diagnostics.json').read_text())
b,bt,bd=load(OUT);rep,rt,rd=load(OUT/'baseline_obstacle_repeat');s,st,sd=load(OUT/'slow')
selected=[(b[0],bt,bd),(b[1],bt,bd),(rep[0],rt,rd),(b[3],bt,bd)]
def metric(result,samples,diags):
 pts=np.array([[float(t['x']),float(t['y'])] for t in samples if t['leg']==result['name'] and float(t['speed'])>.03]);path=np.array(result.get('smoothed_path',[]));deviations=[]
 if len(path)>1 and len(pts):
  a=path[:-1];v=path[1:]-a;den=np.sum(v*v,axis=1);den=np.maximum(den,1e-12)
  for p in pts:
   u=np.clip(np.sum((p-a)*v,axis=1)/den,0,1);deviations.append(float(np.sqrt(np.min(np.sum((p-a-u[:,None]*v)**2,axis=1)))))
 ds=[d for d in diags if d['leg']==result['name']]
 res={k:v for k,v in result.items() if k not in ['raw_path','smoothed_path','cross_track_error_rms_m']}
 res['initial_smoothed_path_deviation_rms_m']=float(np.sqrt(np.mean(np.square(deviations)))) if deviations else None
 res['initial_smoothed_path_deviation_max_m']=max(deviations) if deviations else None
 res['internal_recovery_max_enter_count']=max([int(d.get('recovery_enter_count',0)) for d in ds],default=0)
 res['note']='Diagnostic cross_track_error remained zero; independent deviation uses initially planned path and includes recovery/dynamic replanning.'
 return res
summary={'scene':json.loads((SCENE/'scene_manifest.json').read_text()),'conditions':json.loads((OUT/'test_conditions.json').read_text()),'baseline':[metric(*x) for x in selected],'slow':[metric(x,st,sd) for x in s],'limitations':['Single four-goal sequence per setting; starts differ after earlier goal completion/stall.','Baseline around_obstacles was repeated after a delayed external cancellation invalidated its first attempt.','Obstacle heights are synthetic 0.8m; no real slopes or moving obstacles.','Ground-truth odometry isolates navigation from real localization errors.','Goal acceptance tolerance remains 0.5m; this is not centimeter-accuracy validation.','Grid-center clearances are not continuous collision/contact measurements.']}
summary['scene']['pgm_sha256']=hashlib.sha256((SCENE.parent/'227_1116_remap.pgm').read_bytes()).hexdigest()
(OUT/'comparison.json').write_text(json.dumps(summary,indent=2))
m=yaml.safe_load((SCENE.parent/'227_1116_remap.yaml').read_text());a=np.flipud(np.array(Image.open(SCENE.parent/m['image'])));r=m['resolution'];ox,oy,_=m['origin'];colors=['#df6b19','#00846b','#8a4bb5','#2463bc'];titles=['Baseline: 2.6 m/s; terminal radius 0.50 m','Comparison: 0.8 m/s; terminal radius 0.35 m'];names=['Short diagonal','Cross room','Around obstacles','Return to spawn']
fig,axes=plt.subplots(1,2,figsize=(13,6),sharex=True,sharey=True)
for ax,tests,title in zip(axes,[selected,[(x,st,sd) for x in s]],titles):
 ax.imshow(a,origin='lower',extent=(ox,ox+a.shape[1]*r,oy,oy+a.shape[0]*r),cmap='gray',vmin=0,vmax=254)
 for i,(result,samples,_) in enumerate(tests):
  path=np.array(result.get('smoothed_path',[]));run=np.array([[float(t['x']),float(t['y'])] for t in samples if t['leg']==result['name']]);color=colors[i]
  if len(path):ax.plot(*path.T,'--',color=color,alpha=.45,lw=1)
  if len(run):ax.plot(*run.T,color=color,lw=2,label=f"{i+1}. {names[i]}: {result['status'].lower()}");ax.scatter(*run[-1],color=color,s=32)
  ax.scatter(*result['goal'],color=color,marker='x',s=65,zorder=5)
 ax.set_xlim(-3.7,.95);ax.set_ylim(-1.25,2.25);ax.set_aspect('equal');ax.grid(alpha=.12);ax.set_xlabel('map x (m)');ax.set_title(title,fontsize=11);ax.legend(loc='upper left',fontsize=7.5)
axes[0].set_ylabel('map y (m)');fig.suptitle('Map 227_1116_remap: TDT YAstar + TDT smoother + MINCO/MPC',fontsize=13);fig.text(.5,.02,'Solid: Gazebo ground truth; dashed: initial smoothed path; x: requested goal. Baseline obstacle leg is a separate repeat.',ha='center',fontsize=8);fig.tight_layout(rect=[0,.10,1,.94]);fig.savefig(OUT/'comparison.png',dpi=170);plt.close(fig)
for key in ['baseline','slow']:
 print(key)
 for x in summary[key]:print(x['name'],x['status'],'error',x['endpoint_error_m'],'BT recoveries',x['recoveries'],'internal recovery entries',x['internal_recovery_max_enter_count'],'planner wall ms',x['planner_action_wall_ms'])
