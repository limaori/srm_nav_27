#!/usr/bin/env python3
"""Exercise actual Nav2 action servers and collect Gazebo ground-truth evidence."""
import argparse,json,math,time,csv
from pathlib import Path
from collections import Counter
import numpy as np,yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose,ComputePathToPose,SmoothPath
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import PoseStamped,Twist
from diagnostic_msgs.msg import DiagnosticArray
from action_msgs.msg import GoalStatus
P=argparse.ArgumentParser();P.add_argument('--out',default='log_diag/sim_tdt_227_1116_remap');P.add_argument('--goals',default='0,1,2,3');P.add_argument('--timeout',type=float,default=100);P.add_argument('--manifest',type=Path);args=P.parse_args()
SCENE=Path(__file__).resolve().parent;ROOT=SCENE.parents[2];OUT=Path(args.out).resolve();OUT.mkdir(parents=True,exist_ok=True)
manifest=json.loads((args.manifest or SCENE/'scene_manifest.json').read_text());m=yaml.safe_load(Path(manifest['map_yaml']).read_text());a=np.flipud(np.array(Image.open(Path(manifest['map_yaml']).parent/m['image'])));r=m['resolution'];ox,oy,_=m['origin'];distance=distance_transform_edt(a!=0)*r

def coords(path):return [[p.pose.position.x,p.pose.position.y] for p in path.poses]
def length(path):return sum(math.dist(a,b) for a,b in zip(path,path[1:]))
def clearance(path):
 values=[]
 for p,q in zip(path,path[1:]):
  for u in np.linspace(0,1,max(2,int(math.dist(p,q)/.02)+1)):
   x=p[0]*(1-u)+q[0]*u;y=p[1]*(1-u)+q[1]*u;i=int((x-ox)/r);j=int((y-oy)/r)
   values.append(float(distance[j,i]) if 0<=j<a.shape[0] and 0<=i<a.shape[1] else 0.)
 return min(values) if values else None
class Probe(Node):
 def __init__(self):
  super().__init__('tdt_simulation_probe');self.set_parameters([Parameter('use_sim_time',value=True)])
  self.gt=None;self.samples=[];self.diags=[];self.cmds=[];self.feedback=None;self.leg='idle';self.results=[];self.runtime_paths=[]
  ns='/red_standard_robot1';self.nav=ActionClient(self,NavigateToPose,ns+'/navigate_to_pose');self.plan=ActionClient(self,ComputePathToPose,ns+'/compute_path_to_pose');self.smooth=ActionClient(self,SmoothPath,ns+'/smooth_path')
  self.create_subscription(Odometry,ns+'/chassis_odometry_gt',self.on_gt,qos_profile_sensor_data)
  self.create_subscription(DiagnosticArray,ns+'/FollowPath/diagnostics',self.on_diag,10)
  self.create_subscription(Twist,ns+'/cmd_vel_sim',self.on_cmd,10)
  for name in ['FollowPath/minco_trajectory','FollowPath/mpc_prediction']:
   self.create_subscription(NavPath,ns+'/'+name,lambda m,k=name:self.on_path(k,m),qos_profile_sensor_data)
 def on_gt(self,m):
  p=m.pose.pose.position;v=m.twist.twist.linear;self.gt=(p.x,p.y);t=m.header.stamp.sec+m.header.stamp.nanosec/1e9
  if not self.samples or t-self.samples[-1][0]>=.01:self.samples.append([t,p.x,p.y,math.hypot(v.x,v.y),self.leg])
 def on_path(self,k,m):
  self.runtime_paths.append({'topic':k,'leg':self.leg,'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'frame':m.header.frame_id,'xy':coords(m)})
 def on_diag(self,m):
  for s in m.status:
   d={v.key:v.value for v in s.values};d.update(leg=self.leg,time=time.time());self.diags.append(d)
 def on_cmd(self,m):self.cmds.append([time.time(),m.linear.x,m.linear.y,math.hypot(m.linear.x,m.linear.y),m.angular.z,self.leg])
 def pose(self,xy):
  p=PoseStamped();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg();p.pose.position.x=float(xy[0]);p.pose.position.y=float(xy[1]);p.pose.orientation.w=1.;return p
 def wait(self,f,timeout):
  until=time.monotonic()+timeout
  while rclpy.ok() and not f.done() and time.monotonic()<until:rclpy.spin_once(self,timeout_sec=.05)
  return f.result() if f.done() else None
 def settle(self,sec):
  end=time.monotonic()+sec
  while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
 def action(self,client,goal,timeout=15):
  h=self.wait(client.send_goal_async(goal),5)
  if not h or not h.accepted:return None
  res=self.wait(h.get_result_async(),timeout)
  if res is None:self.wait(h.cancel_goal_async(),3)
  return res
 def save(self):
  (OUT/'results.json').write_text(json.dumps(self.results,indent=2))
  (OUT/'diagnostics.json').write_text(json.dumps(self.diags,indent=2))
  (OUT/'runtime_paths.json').write_text(json.dumps(self.runtime_paths))
  with (OUT/'commands.csv').open('w') as f:
   w=csv.writer(f);w.writerow(['wall_time','vx','vy','speed','wz','leg']);w.writerows(self.cmds)
  with (OUT/'ground_truth.csv').open('w') as f:
   w=csv.writer(f);w.writerow(['sim_time','x','y','speed','leg']);w.writerows(self.samples)
  import matplotlib;matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  fig,ax=plt.subplots(figsize=(9,10));ax.imshow(a,origin='lower',cmap='gray',vmin=0,vmax=254,extent=(ox,ox+a.shape[1]*r,oy,oy+a.shape[0]*r));colors=['#ed6a24','#16a085','#9b59b6','#2563eb']
  for k,res in enumerate(self.results):
   color=colors[k%len(colors)];pts=res.get('smoothed_path',[])
   if pts:ax.plot(*np.array(pts).T,'--',color=color,alpha=.6)
   run=np.array([[s[1],s[2]] for s in self.samples if s[4]==res['name']])
   if len(run):ax.plot(*run.T,color=color,lw=2,label=res['name']+' ('+res.get('status','')+')')
   x,y=res['goal'];ax.scatter([x],[y],c=color,marker='x',s=65)
  ax.set_aspect('equal');ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)');ax.set_title('TDT YAstar + TDT smoother + MINCO/MPC\nSolid: Gazebo ground truth; dashed: requested smoothed path');ax.legend(loc='upper left',fontsize=8);fig.tight_layout();fig.savefig(OUT/'navigation_overlay.png',dpi=150);plt.close(fig)
 def run(self):
  for client,name in [(self.plan,'planner'),(self.smooth,'smoother'),(self.nav,'navigator')]:
   if not client.wait_for_server(timeout_sec=45):raise RuntimeError(name+' action server not ready')
  deadline=time.monotonic()+30
  while self.gt is None and time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.1)
  if self.gt is None:raise RuntimeError('No Gazebo ground-truth odometry')
  self.settle(3)
  for idx in map(int,args.goals.split(',')):
   spec=manifest['goals'][idx];self.leg=spec['name'];result={'name':self.leg,'goal':spec['xy'],'start':list(self.gt),'raw_path':[],'smoothed_path':[]};print('BEGIN',json.dumps(result),flush=True)
   g=ComputePathToPose.Goal();g.goal=self.pose(spec['xy']);g.planner_id='GridBased';g.use_start=False;t=time.monotonic();res=self.action(self.plan,g);result['planner_action_wall_ms']=round((time.monotonic()-t)*1000,2)
   if res and res.status==GoalStatus.STATUS_SUCCEEDED:
    result['raw_path']=coords(res.result.path);result['planning_ms']=res.result.planning_time.sec*1000+res.result.planning_time.nanosec/1e6;result['raw_min_obstacle_center_clearance_m']=clearance(result['raw_path'])
    sm=SmoothPath.Goal();sm.path=res.result.path;sm.smoother_id='tdt_smoother';sm.max_smoothing_duration.sec=1;sm.check_for_collisions=False;sr=self.action(self.smooth,sm)
    if sr and sr.status==GoalStatus.STATUS_SUCCEEDED:
     result['smoothed_path']=coords(sr.result.path);result['smoothing_ms']=sr.result.smoothing_duration.sec*1000+sr.result.smoothing_duration.nanosec/1e6;result['smoothed_min_obstacle_center_clearance_m']=clearance(result['smoothed_path'])
    else:result['smoothing_status']=None if sr is None else sr.status
   else:result['planning_status']=None if res is None else res.status
   print('PLAN',json.dumps({k:v for k,v in result.items() if k not in ['raw_path','smoothed_path']}),flush=True)
   goal=NavigateToPose.Goal();goal.pose=self.pose(spec['xy']);self.feedback=None
   handle=self.wait(self.nav.send_goal_async(goal,feedback_callback=lambda f:setattr(self,'feedback',f.feedback)),5)
   if not handle or not handle.accepted:
    result['status']='GOAL_RESPONSE_TIMEOUT' if handle is None else 'REJECTED'
    self.results.append(result);self.save()
    print('RESULT',json.dumps({'name':self.leg,'status':result['status']}),flush=True);continue
   fut=handle.get_result_async();start=time.monotonic();last=start
   while not fut.done() and time.monotonic()-start<args.timeout:
    rclpy.spin_once(self,timeout_sec=.05)
    if time.monotonic()-last>=5:
     last=time.monotonic();fb=self.feedback;print('TRACK',json.dumps({'leg':self.leg,'wall_s':round(last-start,1),'xy':self.gt,'distance_remaining':None if fb is None else round(fb.distance_remaining,3),'recoveries':None if fb is None else fb.number_of_recoveries,'last_planning_result':self.diags[-1].get('planning_result') if self.diags else None}),flush=True)
   if fut.done():result['status']={4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(fut.result().status,str(fut.result().status))
   else:self.wait(handle.cancel_goal_async(),4);self.wait(fut,6);result['status']='TIMEOUT'
   result['wall_s']=round(time.monotonic()-start,2);self.settle(2);result['end']=list(self.gt);result['endpoint_error_m']=round(math.dist(self.gt,spec['xy']),4);result['recoveries']=None if self.feedback is None else self.feedback.number_of_recoveries
   ss=[s for s in self.samples if s[4]==self.leg];result['actual_distance_m']=length([[s[1],s[2]] for s in ss]);result['min_obstacle_center_clearance_m']=clearance([[s[1],s[2]] for s in ss]);result['max_gt_speed_mps']=max([s[3] for s in ss],default=0)
   ds=[d for d in self.diags if d['leg']==self.leg];result['diagnostic_reasons']=dict(Counter(d.get('planning_result','') for d in ds));ct=[abs(float(d['cross_track_error'])) for d in ds if 'cross_track_error' in d and math.isfinite(float(d['cross_track_error']))];result['cross_track_error_rms_m']=float(np.sqrt(np.mean(np.square(ct)))) if ct else None
   self.results.append(result);self.save();print('RESULT',json.dumps({k:v for k,v in result.items() if k not in ['raw_path','smoothed_path']}),flush=True);self.leg='idle';self.settle(2)
  self.save();print('COMPLETE',str(OUT/'results.json'),flush=True)
rclpy.init();n=Probe()
try:n.run()
finally:n.destroy_node();rclpy.shutdown()
