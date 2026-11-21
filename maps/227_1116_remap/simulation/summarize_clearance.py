#!/usr/bin/env python3
"""Compare the pre-fix close pass with same-map original-speed validation runs."""
from pathlib import Path
import json,csv,hashlib,difflib
import numpy as np,yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
SCENE=Path(__file__).resolve().parent;ROOT=SCENE.parents[2];OUT=ROOT/'log_diag/sim_tdt_227_1116_remap_clearance';OLD=ROOT/'log_diag/sim_tdt_227_1116_remap_fix/run1';RUNS=[OUT/'seed_run1',OUT/'seed_run2'];before=json.loads((OLD/'clearance_summary.json').read_text());after=[json.loads((p/'clearance_summary.json').read_text()) for p in RUNS]
summary={'fix':'Preserve the actually emitted zero command and timestamp as the next acceleration/slew-rate baseline, including the planning-grace and trajectory-rejection paths. Preserve spin-mode angular command history. Filter actual inward velocity using measured ESDF clearance and the reaction-plus-braking distance bound; keep tangential/outward motion. Improve interior local reference points toward the existing optimizer clearance target with bounded monotonic ESDF ascent before clipping; preserve requested endpoints and independent hard validation.','parameters_changed':{'optimization_clearance_margin_m':[0.10,0.15], 'speed_and_acceleration_limits_unchanged':True, 'collision_threshold_unchanged':True},'speed_limit_mps':2.6,'acceleration_limit_mps2':7.,'required_clearance_m':.38,'envelope_radius_m':.33,'before':before,'validation_runs':after,'functional_tests':{'core_passed':225,'controller_passed':77,'new_regressions':13,'static_checks':'Existing clang config files are missing; linter tests excluded, functional gtests passed.'},'limits':['Geometric replay of a synthetic extruded 2D map; ground-truth localization.','Metric is distance to occupied-cell faces minus a 0.33m envelope, not a physical contact measurement.','Recorded XY segments interpolated at <=2mm; baseline samples were coarser than new 100Hz samples.','Intermediate mission starts vary slightly with where the preceding goal stops.']}
summary['minimum_validated_margin_m']=min(v['minimum_envelope_margin_m'] for v in after);summary['maximum_observed_speed_mps']=max(v['maximum_observed_speed_mps'] for v in after);summary['all_goals_succeeded']=all(v['all_succeeded'] for v in after);summary['all_runs_meet_5cm']=all(v['margin_goal_met'] for v in after)
for p in ['src/bringup/srm27_nav_bringup/config/simulation/current/nav2_params_srm_fakeyaw.yaml','src/bringup/srm27_nav_bringup/config/simulation/speed_limits.yaml']:
 summary.setdefault('config_sha256',{})[p]=hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
summary['goal_tolerance_m']=.5
summary['successful_goals']=sum(leg['status']=='SUCCEEDED' for v in after for leg in v['legs'])
summary['nav2_recoveries_total']=sum(leg['nav2_recoveries'] or 0 for v in after for leg in v['legs'])
summary['internal_recovery_max_enter_count']=max(leg['internal_recovery_max_enter_count'] for v in after for leg in v['legs'])
summary['live_parameters']=json.loads((OUT/'live_parameters.json').read_text())
summary['intermediate_retest']={'directory':'guard_run2','all_goals_succeeded':True,'cross_room_nav2_recoveries':23,'accepted_as_final_validation':False,'reason':'Velocity guard alone left the locally clipped corner seed repeatedly invalid; added bounded clearance improvement of reference points before final runs.'}
summary['command_guard_samples']={p.name:sum(d.get('command_owner')=='navigation_clearance_guard' for d in json.loads((p/'diagnostics.json').read_text())) for p in RUNS}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
m=yaml.safe_load((SCENE.parent/'227_1116_remap.yaml').read_text());a=np.flipud(np.array(Image.open(SCENE.parent/m['image'])));r=m['resolution'];ox,oy,_=m['origin'];fig,axes=plt.subplots(1,3,figsize=(15,5.8),gridspec_kw={'width_ratios':[1.1,1.1,1]});colors=['#df6b19','#00846b','#8a4bb5','#2463bc']
for ax,run,metrics,title in [(axes[0],OLD,before,'Before: original speed'),(axes[1],RUNS[-1],after[-1],'After: same speed limits')]:
 rows=list(csv.DictReader((run/'ground_truth.csv').open()));ax.imshow(a,origin='lower',extent=(ox,ox+a.shape[1]*r,oy,oy+a.shape[0]*r),cmap='gray',vmin=0,vmax=254)
 for i,leg in enumerate(metrics['legs']):
  pts=np.array([[float(p['x']),float(p['y'])] for p in rows if p['leg']==leg['name']]);ax.plot(*pts.T,c=colors[i],lw=1.6,label=f"{i+1}: {leg['envelope_margin_m']*100:.1f}cm min")
 worst=min(metrics['legs'],key=lambda x:x['envelope_margin_m']);point=worst['minimum_xy'];ax.add_patch(Circle(point,.33,fill=False,ec='#d62728',lw=1.3));ax.scatter(*point,c='#d62728',s=25);ax.set_xlim(-3.5,.85);ax.set_ylim(-1.1,2.1);ax.set_aspect('equal');ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)');ax.set_title(title,fontsize=11);ax.legend(fontsize=7,loc='upper left');ax.grid(alpha=.15)
x=np.arange(4);ax=axes[2];vals=[v['envelope_margin_m']*100 for v in before['legs']];ax.bar(x-.25,vals,.25,label='Before',color='#a9a9a9')
for i,s in enumerate(after):ax.bar(x+i*.25,[v['envelope_margin_m']*100 for v in s['legs']],.25,label=f'After run {i+1}',color=['#168a8a','#5465ae'][i])
ax.axhline(5,color='#c22',ls='--',lw=1,label='5cm target');ax.set_xticks(x,labels=['Short','Cross','Obstacle','Return'],rotation=25);ax.set_ylabel('Minimum envelope margin (cm)');ax.set_title('Same-map clearance replay',fontsize=11);ax.legend(fontsize=7);ax.grid(axis='y',alpha=.2);fig.suptitle('TDT + MINCO/MPC: close-pass optimization at unchanged 2.6m/s limit',fontsize=13);fig.tight_layout(rect=[0,.03,1,.94]);fig.savefig(OUT/'clearance_comparison.png',dpi=170);plt.close(fig)
print('all succeeded:',summary['all_goals_succeeded'],'minimum margin:',summary['minimum_validated_margin_m'],'observed peak:',summary['maximum_observed_speed_mps'],'5cm met:',summary['all_runs_meet_5cm'])
