"""Probe actual action servers; run one simulation goal only after clearance gates."""
import json
import math
import os
import time
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist, PolygonStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose, SmoothPath
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from exact_clearance import path_clearance

SCENE = Path(__file__).resolve().parents[3] / 'log_diag/sim_tdt_smoother_20261117'
OUT = SCENE / os.environ.get('SIM_RUN_NAME', time.strftime('run_%Y%m%d_%H%M%S'))
OUT.mkdir(parents=True, exist_ok=True)
THRESHOLD = int(os.environ.get('SIM_THRESHOLD', '150'))
STRICT_SOLVE = THRESHOLD < 240
EXECUTION_MARGIN = float(os.environ.get('SIM_EXECUTION_MARGIN','0.07'))
NS = '/red_standard_robot1'
GOAL = [-1.48, 2.02]


def points(path):
    return [[p.pose.position.x, p.pose.position.y] for p in path.poses]


def clearance(path, grid):
    costs = np.asarray(grid.data).reshape(grid.info.height, grid.info.width)
    return path_clearance(path,costs,grid.info.resolution,
                          [grid.info.origin.position.x,grid.info.origin.position.y])


class Probe(Node):
    def __init__(self):
        super().__init__('tdt_smoother_sim_probe')
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.grid = None
        self.gt = None
        self.samples, self.cmds, self.diags, self.input_paths = [], [], [], []
        self.feedback = None
        self.footprint = None
        self.runtime_paths = []
        self.smooth_status = {}
        self.lethal_union = None
        self.raw_map_count = 0
        self.nav_sample_index = None
        self.started = time.monotonic()
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,NS+'/global_costmap/costmap',lambda m:setattr(self,'grid',m),qos)
        self.create_subscription(PolygonStamped,NS+'/global_costmap/published_footprint',lambda m:setattr(self,'footprint',m),10)
        self.create_subscription(Costmap,NS+'/global_costmap/costmap_raw',self.on_raw_map,qos)
        self.create_subscription(GoalStatusArray,NS+'/smooth_path/_action/status',self.on_smooth_status,qos)
        for name in ['plan','plan_smoothed','FollowPath/minco_trajectory']:
            self.create_subscription(NavPath,NS+'/'+name,lambda m,k=name:self.on_path(k,m),10)
        self.create_subscription(Odometry,NS+'/chassis_odometry_gt',self.on_gt,qos_profile_sensor_data)
        self.create_subscription(Twist,NS+'/cmd_vel_sim',self.on_cmd,10)
        self.create_subscription(DiagnosticArray,NS+'/FollowPath/diagnostics',self.on_diag,10)
        self.create_subscription(NavPath,NS+'/FollowPath/planning_input_path',lambda m:self.input_paths.append({'frame':m.header.frame_id,'points':points(m)}),10)
        self.plan=ActionClient(self,ComputePathToPose,NS+'/compute_path_to_pose')
        self.smooth=ActionClient(self,SmoothPath,NS+'/smooth_path')
        self.nav=ActionClient(self,NavigateToPose,NS+'/navigate_to_pose')
        self.result={'goal':GOAL,'mode':'simulation','navigation_sent':False}

    def on_gt(self,m):
        p=m.pose.pose.position
        v=m.twist.twist.linear
        self.gt=[p.x,p.y]
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if not self.samples or t-self.samples[-1][0]>=.02:
            self.samples.append([t,p.x,p.y,math.hypot(v.x,v.y)])

    def on_cmd(self,m):
        self.cmds.append([time.monotonic()-self.started,m.linear.x,m.linear.y,m.angular.z])

    def on_diag(self,m):
        for status in m.status:
            self.diags.append(dict({v.key:v.value for v in status.values},capture_sim_time=self.get_clock().now().nanoseconds*1e-9))

    def on_raw_map(self,m):
        lethal=np.asarray(m.data,dtype=np.uint8).reshape(m.metadata.size_y,m.metadata.size_x)==254
        if self.lethal_union is None: self.lethal_union=lethal.copy()
        else: self.lethal_union |= lethal
        self.raw_map_count+=1

    def on_smooth_status(self,m):
        for status in m.status_list:
            self.smooth_status[bytes(status.goal_info.goal_id.uuid).hex()]=status

    def on_path(self,name,m):
        self.runtime_paths.append({'topic':name,'frame':m.header.frame_id,
                                   't':self.get_clock().now().nanoseconds*1e-9,'points':points(m)})

    def wait(self,future,seconds):
        deadline=time.monotonic()+seconds
        while rclpy.ok() and not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)
        return future.result() if future.done() else None

    def settle(self,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)

    def action(self,client,goal,timeout):
        handle=self.wait(client.send_goal_async(goal),5)
        if handle is None or not handle.accepted: return None
        result=self.wait(handle.get_result_async(),timeout)
        if result is None: self.wait(handle.cancel_goal_async(),3)
        return result

    def pose(self):
        p=PoseStamped()
        p.header.frame_id='map'
        p.header.stamp=self.get_clock().now().to_msg()
        p.pose.position.x,p.pose.position.y=GOAL
        p.pose.orientation.w=1.0
        return p

    def parameters(self):
        checks={
            'planner_server':{'GridBased.plugin':'tdt_nav_kit::TdtGlobalPlanner','GridBased.occupancy_threshold':THRESHOLD},
            'smoother_server':{'tdt_smoother.plugin':'tdt_nav_kit::TdtSmoother','tdt_smoother.occupancy_threshold':253,
                               **({'tdt_smoother.fallback_to_resampled':False,'robot_base_frame':'gimbal_yaw_fake'} if STRICT_SOLVE else {})},
            'controller_server':{'FollowPath.limits.max_linear_speed':1.0,
                                 'FollowPath.safety.robot_radius':0.28,'FollowPath.safety.clearance_margin':EXECUTION_MARGIN},
            'global_costmap/global_costmap':{'robot_radius':0.30,'intensity_voxel_layer.enabled':True,
                                             'footprint_padding':0.01,
                                             **({'inflation_layer.inflation_radius':0.50} if STRICT_SOLVE else {})},
            'local_costmap/local_costmap':{'robot_radius':0.30,'footprint_padding':0.01,'intensity_voxel_layer.enabled':True},
            'bt_navigator':{'default_nav_to_pose_bt_xml':str(Path(get_package_share_directory('srm27_nav_bringup'))/'behavior_trees/navigate_to_pose_w_smoothing.xml')},
            'map_server':{'yaml_filename':str(SCENE.parents[1]/'maps/227_1113pro/227_1113pro.yaml')},
        }
        verified={}
        for node,expected in checks.items():
            client=self.create_client(GetParameters,NS+'/'+node+'/get_parameters')
            if not client.wait_for_service(timeout_sec=10): raise RuntimeError('No params: '+node)
            request=GetParameters.Request();request.names=list(expected)
            response=self.wait(client.call_async(request),5)
            if response is None: raise RuntimeError('Param timeout: '+node)
            actual={}
            for key,value in zip(request.names,response.values):
                actual[key]={1:value.bool_value,2:value.integer_value,3:value.double_value,4:value.string_value}.get(value.type)
                if actual[key]!=expected[key]: raise RuntimeError(f'{node}.{key}: {actual[key]!r} != {expected[key]!r}')
            verified[node]=actual
        self.result['verified_parameters']=verified
        bt=ET.parse(verified['bt_navigator']['default_nav_to_pose_bt_xml'])
        if bt.findall('.//BackUp') or any(x.get('check_for_collisions')!='true' for x in bt.findall('.//SmoothPath')) or not bt.findall('.//SmoothPath'):
            raise RuntimeError('Unsafe BT: collision check disabled/missing, or BackUp present')
        self.result['bt_collision_check_enabled']=True
        self.result['bt_backup_nodes']=0

    def save(self):
        (OUT/'run_result.json').write_text(json.dumps(self.result,indent=2))
        (OUT/'capture.json').write_text(json.dumps({'ground_truth':self.samples,'commands':self.cmds,'diagnostics':self.diags,'input_paths':self.input_paths,'runtime_paths':self.runtime_paths,'navigation_sample_index':self.nav_sample_index}))
        if self.grid is not None:
            np.savez_compressed(OUT/'global_costmap.npz',data=np.asarray(self.grid.data).reshape(self.grid.info.height,self.grid.info.width),resolution=self.grid.info.resolution,origin=[self.grid.info.origin.position.x,self.grid.info.origin.position.y])
            if self.lethal_union is not None:
                np.savez_compressed(OUT/'global_lethal_union.npz',data=self.lethal_union.astype(np.int16)*100,resolution=self.grid.info.resolution,origin=[self.grid.info.origin.position.x,self.grid.info.origin.position.y],count=self.raw_map_count)
        if self.footprint is not None:
            (OUT/'footprint.json').write_text(json.dumps({'frame':self.footprint.header.frame_id,'points':[[p.x,p.y] for p in self.footprint.polygon.points]}))

    def run(self):
        for client,name in [(self.plan,'planner'),(self.smooth,'smoother'),(self.nav,'navigator')]:
            if not client.wait_for_server(timeout_sec=45): raise RuntimeError('No action server: '+name)
        deadline=time.monotonic()+30
        while (self.grid is None or self.gt is None) and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.1)
        if self.grid is None or self.gt is None: raise RuntimeError('Missing costmap or Gazebo odometry')
        self.settle(3)
        self.parameters()
        self.result['start_gt']=self.gt
        if math.dist(self.gt,[.236,-.110])>.03: raise RuntimeError('Unexpected simulation spawn')
        self.result['costmap_frame']=self.grid.header.frame_id
        goal=ComputePathToPose.Goal();goal.goal=self.pose();goal.planner_id='GridBased'
        raw=self.action(self.plan,goal,20)
        self.result['planning_status']=None if raw is None else raw.status
        if raw is None or raw.status!=4: return
        path=points(raw.result.path)
        self.result['raw_path']=path
        self.result['raw_clearance']=clearance(path,self.grid)
        goal=SmoothPath.Goal();goal.path=raw.result.path;goal.smoother_id='tdt_smoother'
        goal.max_smoothing_duration.sec=1;goal.check_for_collisions=True
        smooth=self.action(self.smooth,goal,20)
        self.result['smoothing_status']=None if smooth is None else smooth.status
        if smooth is None: return
        self.result['smoothed_path']=points(smooth.result.path)
        self.result['smoothed_poses']=[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in smooth.result.path.poses]
        self.result['smoothed_clearance']=clearance(self.result['smoothed_path'],self.grid)
        self.result['was_completed']=smooth.result.was_completed
        if smooth.status!=4: return
        self.result['gate_passed']=smooth.result.was_completed and all(self.result[k]['min_m']>=.30 for k in ['raw_clearance','smoothed_clearance'])
        print('PREFLIGHT',json.dumps({k:v for k,v in self.result.items() if k not in ['raw_path','smoothed_path','smoothed_poses']}),flush=True)
        if not self.result['gate_passed'] or os.environ.get('SIM_PREFLIGHT_ONLY')=='1': return
        goal=NavigateToPose.Goal();goal.pose=self.pose()
        self.result['navigation_sent']=True
        self.nav_sample_index=len(self.samples)
        self.result['navigation_start_sim_time']=self.get_clock().now().nanoseconds*1e-9
        handle=self.wait(self.nav.send_goal_async(goal,feedback_callback=lambda f:setattr(self,'feedback',f.feedback)),5)
        if handle is None or not handle.accepted: raise RuntimeError('Navigation rejected')
        future=handle.get_result_async();started=time.monotonic();last=started
        while not future.done() and time.monotonic()-started<90:
            rclpy.spin_once(self,timeout_sec=.05)
            if time.monotonic()-last>=5:
                last=time.monotonic()
                print('TRACK',self.gt,'elapsed',round(last-started,1),'last_plan',self.diags[-1].get('planning_result') if self.diags else None,flush=True)
        if not future.done():
            self.wait(handle.cancel_goal_async(),3)
            self.wait(future,5)
            self.result['navigation_status']='TIMEOUT'
        else: self.result['navigation_status']={4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(future.result().status,str(future.result().status))
        self.result['navigation_wall_seconds']=time.monotonic()-started
        self.result['navigation_end_sim_time']=self.get_clock().now().nanoseconds*1e-9
        self.settle(3)
        self.result.update(end_gt=self.gt,endpoint_error_m=math.dist(self.gt,GOAL),
                           recoveries=None if self.feedback is None else self.feedback.number_of_recoveries,
                           max_gt_speed_mps=max(s[3] for s in self.samples),
                           final_cmd=self.cmds[-1] if self.cmds else None,
                           final_gt_speed_mps=self.samples[-1][3],
                           diagnostic_reasons=dict(Counter(d.get('planning_result','') for d in self.diags)))
        # Query each completed SmoothPath action result, including those invoked
        # by the BT, so success cannot hide an interrupted/uncompleted solve.
        result_client=self.create_client(SmoothPath.Impl.GetResultService,NS+'/smooth_path/_action/get_result')
        completed=[]
        if not result_client.wait_for_service(timeout_sec=5): raise RuntimeError('No smoother result service')
        for key,status in list(self.smooth_status.items()):
            if status.status not in [4,5,6]: continue
            request=SmoothPath.Impl.GetResultService.Request();request.goal_id=status.goal_info.goal_id
            response=self.wait(result_client.call_async(request),5)
            completed.append({'id':key,'status':None if response is None else response.status,
                              'was_completed':None if response is None else response.result.was_completed,
                              'points':0 if response is None else len(response.result.path.poses)})
        self.result['smoother_action_results']=completed


if __name__=='__main__':
    rclpy.init()
    node=Probe()
    try:
        node.run()
    except Exception as error:
        node.result['error']=repr(error)
        raise
    finally:
        node.save()
        print('RESULT',json.dumps({k:v for k,v in node.result.items() if k not in ['raw_path','smoothed_path','smoothed_poses']}),flush=True)
        node.destroy_node()
        rclpy.shutdown()
