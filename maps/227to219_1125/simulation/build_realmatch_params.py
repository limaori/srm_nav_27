#!/usr/bin/env python3
"""生成「与实车同链」的仿真参数：nav2_params_yastar_realmatch.yaml

为什么要这份文件
================
默认仿真链路和实车链路**不是同一条**（2026-11-25 核对）：

  项                实车                                仿真默认
  ----------------- ---------------- ------------------------------
  平滑器            MinimumSnapSmoother                  SimpleSmoother
  行为树            含 SmoothPath                        不含 SmoothPath
                                                         → 平滑器从不被调用
  costmap 外形      0.30 / inflation 0.30（二值）        0.40 / 0.50（有梯度）
  corner_speed_ratio 0.45                               0.80
  minco.w_obstacle  100                                 500
  safety            0.245 / margin 0.0 / stuck 0.245     0.33 / 0.06 / 0.40

所以仿真里看到的拐角比实车钝/甩得远 3 倍左右（弯速 2.08 vs 1.17 m/s，
最小转弯半径 0.62 vs 0.20 m，90° 折角外扩 0.26 vs 0.08 m）。

这份文件以**当前仿真参数**为底（保证仿真专用节点段完整：ign_sim_pointcloud_tool
/ terrain_analysis / sensor_scan_generation 等一个不落），只把上表这些「链路差异项」
换成实车值；其余两边的 90 个 FollowPath 参数里有 80 个本来就相同。

用法
====
  ./maps/227to219_1125/simulation/run_realmatch.sh

再跑一遍本脚本可随时重新生成（实车 yaml 改了之后同步）。
"""
import copy
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SIM_BASE = HERE / 'nav2_params_yastar.yaml'
REAL = ROOT / 'src/bringup/srm27_nav_bringup/config/real/current/nav2_params_srm_minco_fakeyaw.yaml'
OUT = HERE / 'nav2_params_yastar_realmatch.yaml'

sim = yaml.safe_load(SIM_BASE.read_text(encoding='utf-8'))
real = yaml.safe_load(REAL.read_text(encoding='utf-8'))

changes = []


def note(msg):
    changes.append('  # - ' + msg)


# 1) 控制器：整段换成实车（同一插件；两边只差 12 项 = FollowPath 里 10 项
#    + general_goal_checker 的 xy_goal_tolerance / rot_stopped_velocity）
sim['controller_server']['ros__parameters'] = copy.deepcopy(
    real['controller_server']['ros__parameters'])
note('controller_server 整段 = 实车（corner_speed_ratio 0.80→0.45, '
     'w_obstacle 500→100, optimization_clearance_margin 0.15→0.02, '
     'safety 0.28/0.02→0.245/0.0, stuck_clearance 0.30→0.245, '
     'path_centering_max_displacement=0.3, '
     'general_goal_checker xy 容差 0.4→0.2、补 rot_stopped_velocity=0.05）')

# 2) 平滑器：实车是 MinimumSnapSmoother（corridor 后端）
sim['smoother_server']['ros__parameters'] = copy.deepcopy(
    real['smoother_server']['ros__parameters'])
note('smoother_server = 实车 MinimumSnapSmoother（原来 simple_smoother 从不被调用）')

# 3) 行为树：实车用带 SmoothPath 的版本，并把 SmoothPath 的 BT 节点注册上
sim_bt = sim['bt_navigator']['ros__parameters']
real_bt = real['bt_navigator']['ros__parameters']
sim_bt['default_nav_to_pose_bt_xml'] = real_bt['default_nav_to_pose_bt_xml']
sim_bt['default_nav_through_poses_bt_xml'] = real_bt['default_nav_through_poses_bt_xml']
libs = list(sim_bt.get('plugin_lib_names', []))
for name in real_bt.get('plugin_lib_names', []):
    if name not in libs:
        libs.append(name)
sim_bt['plugin_lib_names'] = libs
note('bt_navigator 换成 navigate_(to|through)_poses_w_smoothing.xml 并补 '
     'nav2_smooth_path_action_bt_node')

# 4) costmap：外形/膨胀与实车一致；未知格口径也一致。
#    注意 **sensor_frame 故意不抄**：实车是 livox_imu，仿真是 front_mid360，
#    抄过来 voxel 层会被过滤成空，机器人等于没有实时障碍。
for cm in ('global_costmap', 'local_costmap'):
    sc = sim[cm][cm]['ros__parameters']
    rc = real[cm][cm]['ros__parameters']
    sc['robot_radius'] = rc['robot_radius']
    sc['inflation_layer'] = copy.deepcopy(rc['inflation_layer'])
    if 'track_unknown_space' in rc:
        sc['track_unknown_space'] = rc['track_unknown_space']
    for source, cfg in (sc.get('intensity_voxel_layer') or {}).items():
        if isinstance(cfg, dict) and 'min_obstacle_height' in cfg:
            cfg['min_obstacle_height'] = 0.1
    if isinstance(sc.get('intensity_voxel_layer'), dict) and \
            'min_obstacle_height' in sc['intensity_voxel_layer']:
        sc['intensity_voxel_layer']['min_obstacle_height'] = 0.1
note('costmap robot_radius 0.40→0.30、inflation_layer 0.50→0.30、'
     'track_unknown_space=True、min_obstacle_height→0.1（sensor_frame 保留仿真的）')

# 5) 规划器：保持 YAstar，只把 unknown 口径对齐实车（True）
planner = sim['planner_server']['ros__parameters']['GridBased']
planner['treat_unknown_as_obstacle'] = True
note('planner_server.GridBased.treat_unknown_as_obstacle False→True（对齐实车）')

# 5b) 规划器侧 MINCO（2026-11-26 新增，默认不启用）：参数块随生成物一起产出，
#     否则每次重跑脚本都会把它删掉，切回来还得手抄一遍。
#     启用 = 把 GridBased.plugin 改成 srm27_minco_planner::MincoPlanner，
#     并把 MincoPlanner 段的键名改成 GridBased（BT 里 planner_id 就是 GridBased）。
_fp = real['controller_server']['ros__parameters']['FollowPath']
sim['planner_server']['ros__parameters']['yastar'] = copy.deepcopy(planner)
sim['planner_server']['ros__parameters']['MincoPlanner'] = {
    'plugin': 'srm27_minco_planner::MincoPlanner',
    'yastar_plugin_name': 'yastar',
    'base_frame': _fp['base_frame'],
    'output_frame': 'odom',
    'smooth_path_topic': 'plan_smoothed',
    'odom_topic': _fp['odom_topic'],
    'trajectory_topic': 'planning/minco_trajectory',
    'path_topic': 'planning/minco_path',
    'diagnostics_topic': 'planning/diagnostics',
    'opt_frequency': 20.0,
    'planning_horizon': _fp['planning_horizon'],
    'path_centering_max_displacement': _fp['path_centering_max_displacement'],
    'validity_window': 1.0,
    'terminal_reached_radius': _fp['terminal_reached_radius'],
    'goal_change_tolerance': _fp['goal_change_tolerance'],
    'diagnostics_period': _fp['diagnostics_period'],
    'state_timeout': _fp['state_timeout'],
    'publish_visualization': True,
    'use_warm_start': True,
    # 限速：实车参数文件不写 limits（唯一来源是 speed_limits.yaml 的
    # controller_server@minco 段），这里按那份文件的 minco 档取值。
    # 将来规划器启用后，需要在 speed_limits.yaml 增加 planner_server@minco 覆盖。
    'limits': {'max_linear_speed': 2.60, 'max_linear_accel': 7.0,
               'max_angular_speed': 8.50, 'max_angular_accel': 25.0},
    'safety': copy.deepcopy(_fp['safety']),
    'validator': {'sample_dt': 0.02, 'max_sample_spacing': 0.02,
                  'speed_tolerance_ratio': 0.02, 'accel_tolerance_ratio': 0.02,
                  'required_prefix_duration': 0.6},
    'minco': copy.deepcopy(_fp['minco']),
}
note('planner_server 增补 MincoPlanner/yastar 参数块（规划器侧 MINCO，默认不启用）')


# 6) 全量 use_sim_time=True（仿真必须）
def force_sim_time(node):
    if isinstance(node, dict):
        for key, value in node.items():
            if key == 'use_sim_time':
                node[key] = True
            else:
                force_sim_time(value)


force_sim_time(sim)
note('全文件 use_sim_time=True（仿真需要，实车那份是 False）')

header = (
    '# =============================================================================\n'
    '# 由 build_realmatch_params.py 生成，不要手改 —— 改完请重跑生成脚本。\n'
    '#\n'
    '# 目的：让**仿真**跑出和**实车**一致的规划/控制链路，用来调拐角、净空这类\n'
    '#       "调参结论要能搬回实车" 的东西。\n'
    '# 底稿：maps/227to219_1125/simulation/nav2_params_yastar.yaml（仿真专用节点段齐全）\n'
    '# 覆盖：src/bringup/srm27_nav_bringup/config/real/current/\n'
    '#       nav2_params_srm_minco_fakeyaw.yaml 里与仿真不同的那几项\n'
    '#\n'
    '# 本次覆盖：\n' + '\n'.join(changes) + '\n'
    '#\n'
    '# 与实车仍剩下的差异（仿真固有，不影响拐角结论）：\n'
    '#   * 定位：这里用 Gazebo 真值 odom；实车是 zlio/zloc/GICP。\n'
    '#     → 仿真测不出定位漂移导致的系统性贴边；那条要用实车数据看。\n'
    '#   * 障碍来源：这里是 Gazebo 雷达；实车是 MID-360 + terrain_analysis。\n'
    '# =============================================================================\n'
)
OUT.write_text(header + yaml.safe_dump(sim, sort_keys=False, allow_unicode=True), encoding='utf-8')
print(f'wrote {OUT}  ({OUT.stat().st_size / 1024:.0f} KB)')
for line in changes:
    print(line)
