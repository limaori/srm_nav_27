"""Independent closed-loop acceptance from recorded geometry and motion."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from exact_clearance import path_clearance


def nearest_path_distance(points,path):
    path=np.asarray(path,dtype=float)
    a,b=path[:-1],path[1:]
    delta=b-a
    denom=np.maximum(np.sum(delta*delta,axis=1),1e-20)
    output=[]
    for p in points:
        t=np.clip(np.sum((p-a)*delta,axis=1)/denom,0,1)
        output.append(float(np.linalg.norm(p-(a+t[:,None]*delta),axis=1).min()))
    return np.asarray(output)


def analyze(directory):
    r=json.loads((directory/'run_result.json').read_text())
    c=json.loads((directory/'capture.json').read_text())
    grid=np.load(directory/'global_lethal_union.npz')
    gt=np.asarray(c['ground_truth'])
    cmds=np.asarray(c['commands'])
    start=r['navigation_start_sim_time'];end=r['navigation_end_sim_time']
    moving=gt[(gt[:,0]>=start)&(gt[:,0]<=end)]
    clearance=path_clearance(gt[:,1:3],grid['data'],float(grid['resolution']),grid['origin'])
    stop=gt[gt[:,0]>=gt[-1,0]-2]
    stop_range=float(np.linalg.norm(np.ptp(stop[:,1:3],axis=0)))
    # The simulator can retain its last nonzero twist after it stops. Assess
    # actual motion using finite differences of world-position ground truth.
    dt=np.diff(gt[:,0]);speed=np.linalg.norm(np.diff(gt[:,1:3],axis=0),axis=1)/dt
    cmd_speed=np.hypot(cmds[:,1],cmds[:,2])
    stopped_commands=cmds[cmds[:,0]>=cmds[-1,0]-2]
    stopped_cmd_max=float(np.max(np.abs(stopped_commands[:,1:])))
    runtime=[]
    for p in c['runtime_paths']:
        if p['topic'] not in ['plan','plan_smoothed']: continue
        if p['frame']!='map': raise ValueError('Unexpected global path frame')
        runtime.append({'topic':p['topic'],'points':len(p['points']),
                        'clearance_m':path_clearance(p['points'],grid['data'],float(grid['resolution']),grid['origin'])['min_m']})
    tracking=nearest_path_distance(moving[:,1:3],r['smoothed_path'])
    smoother_results=r['smoother_action_results']
    diags=c['diagnostics']
    finite_minco=[float(d['minimum_clearance']) for d in diags
                 if d.get('planning_result')=='success' and math.isfinite(float(d.get('minimum_clearance','nan')))]
    metrics={
        'run':directory.name,'navigation_status':r['navigation_status'],
        'raw_preflight_clearance_m':r['raw_clearance']['min_m'],
        'smoothed_preflight_clearance_m':r['smoothed_clearance']['min_m'],
        'actual_trajectory_clearance_to_all_observed_lethal_cells':clearance,
        'runtime_global_paths':runtime,'raw_costmap_messages':int(grid['count']),
        'navigation_sim_seconds':end-start,'navigation_wall_seconds':r['navigation_wall_seconds'],
        'actual_distance_m':float(np.linalg.norm(np.diff(moving[:,1:3],axis=0),axis=1).sum()),
        'peak_gt_position_speed_mps':float(speed.max()),'peak_command_speed_mps':float(cmd_speed.max()),
        'moving_mean_gt_position_speed_mps':float(speed[speed>.02].mean()),
        'endpoint_error_m':r['endpoint_error_m'],'recoveries':r['recoveries'],
        'cross_track_to_preflight_mean_m':float(tracking.mean()),
        'cross_track_to_preflight_max_m':float(tracking.max()),
        'cross_track_to_preflight_p95_m':float(np.quantile(tracking,.95)),
        'last_two_seconds_displacement_range_m':stop_range,
        'last_two_seconds_max_command_component':stopped_cmd_max,
        'minimum_reported_validated_minco_clearance_m':min(finite_minco) if finite_minco else None,
        'smooth_actions':smoother_results,
    }
    checks={
        'preflight_planning_and_smoothing_succeeded':r['planning_status']==4 and r['smoothing_status']==4 and r['was_completed'],
        'preflight_clearances_at_least_030':min(r['raw_clearance']['min_m'],r['smoothed_clearance']['min_m'])>=.30,
        'runtime_global_paths_clearance_at_least_030':len(runtime)>=2 and min(p['clearance_m'] for p in runtime)>=.30,
        'actual_trajectory_clearance_at_least_030':clearance['min_m']>=.30,
        'all_smoother_actions_completed_successfully':len(smoother_results)>=2 and all(x['status']==4 and x['was_completed'] for x in smoother_results),
        'navigator_succeeded':r['navigation_status']=='SUCCEEDED',
        'no_navigation_recovery':r['recoveries']==0,
        'endpoint_within_configured_020_m':r['endpoint_error_m']<=.20,
        'speed_limit_1_mps_respected':float(speed.max())<=1.0+1e-6 and float(cmd_speed.max())<=1.0+1e-6,
        'full_speed_test_exercised':float(speed.max())>=.90,
        'stopped_for_two_seconds':stop_range<=.002 and stopped_cmd_max<=1e-6 and len(stop)>=10,
        'fallback_disabled':r['verified_parameters']['smoother_server']['minimum_snap_smoother.fallback_to_resampled'] is False,
        'execution_clearance_not_lowered':sum(r['verified_parameters']['controller_server'][k] for k in ['FollowPath.safety.robot_radius','FollowPath.safety.clearance_margin'])>=.30,
        'bt_collision_check_enabled':r.get('bt_collision_check_enabled',False),
    }
    answer={'passed':all(checks.values()),'checks':checks,'metrics':metrics,
            'limitations':['Map-derived synthetic 0.8 m walls and Gazebo ground-truth odometry.',
                           'Clearance uses continuous line segments between captured world poses and the union of all observed lethal cell squares.',
                           'Speed limit is 1.0 m/s; the robot accelerates, brakes, and slows for turns.',
                           'No real hardware, ZLIO localization, or historical live obstacle replay validated.']}
    (directory/'acceptance.json').write_text(json.dumps(answer,indent=2))
    print(json.dumps(answer,indent=2))
    return answer


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run')
    args=parser.parse_args()
    result=analyze(Path(__file__).resolve().parents[3]/'log_diag/sim_tdt_smoother_20261117'/args.run)
    raise SystemExit(0 if result['passed'] else 2)
