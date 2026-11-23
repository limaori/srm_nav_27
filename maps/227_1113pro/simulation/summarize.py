"""Produce the final acceptance artifact and an exportable verification figure."""
import hashlib
import json
import re
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SOURCE=Path(__file__).resolve().parent
ROOT=SOURCE.parents[2]
OUT=ROOT/'log_diag/sim_tdt_smoother_20261117'
cfg=yaml.safe_load((SOURCE.parent/'227_1113pro.yaml').read_text())
image=np.flipud(np.asarray(Image.open(SOURCE.parent/cfg['image'])))
r=cfg['resolution'];ox,oy,_=cfg['origin']
fig,axes=plt.subplots(1,3,figsize=(16,5.6))
ax=axes[0]
ax.imshow(image,origin='lower',cmap='gray',vmin=0,vmax=254,
          extent=(ox,ox+image.shape[1]*r,oy,oy+image.shape[0]*r))
old=json.loads((OUT/'threshold240/run_result.json').read_text())
raw=np.asarray(old['raw_path']);ax.plot(*raw.T,':',color='#e05252',label='Original raw (smoothing aborted)')
summary=[]
for i,name in enumerate(['fixed_run1','fixed_run2']):
    color=['#1765bd','#009977'][i]
    path=OUT/name
    result=json.loads((path/'run_result.json').read_text())
    capture=json.loads((path/'capture.json').read_text())
    acceptance=json.loads((path/'acceptance.json').read_text())
    log=(OUT/f'launch_{name}.log').read_text()
    events={key:len(re.findall(pattern,log)) for key,pattern in {
        'smoother_collision':'Smoothed path leads to a collision',
        'smoother_solve_failure':'OSQP solve failed',
        'smoother_timeout':'TdtSmoother: exceeded max_time',
        'controller_validation_rejection':r'规划失败 \(validation_failed\)',
        'qp_over_budget':r'QP 求解 .*超过预算',
        'backing_up':'backing up'}.items()}
    acceptance['log_event_counts']=events
    acceptance['passed']=acceptance['passed'] and all(value==0 for value in events.values())
    summary.append(acceptance)
    samples=np.asarray(capture['ground_truth']);t=samples[:,0]-result['navigation_start_sim_time']
    smoothed=np.asarray(result['smoothed_path'])
    if i==0:ax.plot(*smoothed.T,'--',color='#d69000',lw=1.8,label='Fixed smoothed path')
    ax.plot(*samples[:,1:3].T,color=color,lw=1.5,label=f'Actual run {i+1}')
    speed=np.linalg.norm(np.diff(samples[:,1:3],axis=0),axis=1)/np.diff(samples[:,0])
    axes[1].plot(t[1:],speed,color=color,label=f'Run {i+1}')
    grid=np.load(path/'global_lethal_union.npz')
    yy,xx=np.where(grid['data']>=100)
    centers=np.column_stack((grid['origin'][0]+(xx+.5)*r,grid['origin'][1]+(yy+.5)*r))
    clearance=np.array([np.linalg.norm(np.maximum(np.abs(centers-p)-r/2,0),axis=1).min() for p in samples[:,1:3]])
    axes[2].plot(t,clearance,color=color,label=f'Run {i+1}')
    m=acceptance['metrics']
    axes[2].annotate(f"Run {i+1} exact segment min: {m['actual_trajectory_clearance_to_all_observed_lethal_cells']['min_m']:.3f} m",
                     xy=(.03,.1+i*.07),xycoords='axes fraction',color=color,fontsize=9)
ax.scatter([.236],[-.110],color='#009977',marker='o',s=35)
ax.scatter([-1.48],[2.02],color='#c42b2b',marker='*',s=110,label='Goal')
ax.set_xlim(-2,1.1);ax.set_ylim(-.5,2.8);ax.set_aspect('equal')
ax.set_title('227_1113pro: path and actual motion');ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)')
ax.legend(fontsize=8,loc='lower left')
axes[1].axhline(1.,color='#cc4444',ls='--',label='1.0 m/s limit');axes[1].set_ylim(-.02,1.08)
axes[1].set_title('Speed from Gazebo world-position differences');axes[1].set_ylabel('speed (m/s)')
axes[2].axhline(.30,color='#cc4444',ls='--',label='0.30 m acceptance gate');axes[2].set_ylim(.25,.8)
axes[2].set_title('Clearance to all observed lethal cell squares');axes[2].set_ylabel('clearance (m)')
for a in axes[1:]:
    a.set_xlabel('simulation seconds from navigation start');a.set_xlim(-.2,8);a.legend(fontsize=8);a.grid(alpha=.2)
fig.suptitle('Collision checking ON | fallback OFF | 1.0 m/s limit | two cold-start closed-loop runs passed',fontsize=13)
fig.tight_layout();fig.savefig(OUT/'closed_loop_acceptance.png',dpi=170)

stages=[]
for name in ['collision_debug','closed_loop_run1','closed_loop_run2']:
    result=json.loads((OUT/name/'run_result.json').read_text())
    entry={'run':name,'smoothing_status':result['smoothing_status'],
           'navigation_status':result.get('navigation_status'),'accepted':False}
    if name=='collision_debug':entry['smoothed_centerline_clearance_m']=result['smoothed_clearance']['min_m']
    else:
        # Deliberately retain rejected experiments as evidence, not successes.
        import sys
        sys.path.insert(0,str(SOURCE))
        from exact_clearance import path_clearance
        capture=json.loads((OUT/name/'capture.json').read_text());grid=np.load(OUT/name/'global_lethal_union.npz')
        entry['actual_clearance_m']=path_clearance([s[1:3] for s in capture['ground_truth']],grid['data'],float(grid['resolution']),grid['origin'])['min_m']
    stages.append(entry)
answer={
    'passed':all(x['passed'] for x in summary),'accepted_cold_start_runs':len(summary),
    'map':'227_1113pro','start':[.236,-.110,0.0],'goal':[-1.48,2.02],
    'independent_clearance_gate_m':.30,'speed_limit_mps':1.0,
    'changes':{'global_inflation_radius_m':[.31,.50],'planner_occupancy_threshold':[240,150],
               'smoother_robot_base_frame':['base_link','gimbal_yaw_fake'],
               'fallback_to_resampled':[True,False],'minco_clearance_margin_m':[.02,.07],
               'minco_required_clearance_m':[.30,.35]},
    'preserved':{'costmap_robot_radius_m':.30,'footprint_padding_m':.01,'bt_collision_check':True,
                 'live_obstacle_layers':True,'minco_robot_radius_m':.28,'physical_hardware_used':False},
    'fixed_config':str(SOURCE/'nav2_params_fixed_fakeyaw.yaml'),
    'fixed_config_sha256':hashlib.sha256((SOURCE/'nav2_params_fixed_fakeyaw.yaml').read_bytes()).hexdigest(),
    'rejected_experiments':stages,'accepted_runs':summary,'final_state':'Simulation stopped after final run.',
    'measurement_note':'Stopping is assessed from world-position change and zero commands; Gazebo raw odometry can retain stale nonzero twist after stopping.',
    'source_reference':'https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_smoother/src/nav2_smoother.cpp',
}
(OUT/'acceptance_summary.json').write_text(json.dumps(answer,indent=2))
print(json.dumps({'passed':answer['passed'],'accepted_cold_start_runs':len(summary),
                  'runs':[{k:x['metrics'][k] for k in ['run','endpoint_error_m','peak_gt_position_speed_mps','navigation_sim_seconds']} for x in summary]},indent=2))
