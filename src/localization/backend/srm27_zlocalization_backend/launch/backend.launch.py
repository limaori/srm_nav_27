"""启动定位后端，并应用配置、地图和可视化选项。"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 解析启动参数中的布尔值并拒绝非法输入。
def launch_bool(value):
    if str(value).lower() not in ('true', 'false', '1', '0', 'yes', 'no', 'on', 'off'):
        raise ValueError(f'Invalid boolean: {value}')
    return str(value).lower() in ('true', '1', 'yes', 'on')


# 递归合并配置映射，以覆盖值替换对应叶参数。
def merge(base, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


# 读取单父继承配置并检查循环引用。
def load_params(path, ancestors=()):
    path = Path(path).resolve()
    if path in ancestors:
        raise ValueError(f'Configuration inheritance cycle: {path}')
    with path.open(encoding='utf-8') as stream:
        params = yaml.safe_load(stream) or {}
    if not isinstance(params, dict):
        raise ValueError(f'Configuration must be a mapping: {path}')
    parent = params.pop('extends', None)
    if parent:
        params = merge(load_params(path.parent / parent, ancestors=(*ancestors, path)), params)
    return params


# 校验运行期使用的目录参数，拒绝依赖进程工作目录的相对路径。
def require_absolute(label, value):
    if value and not os.path.isabs(value):
        raise ValueError(
            f'{label} must be an absolute path, got {value!r}; '
            'a relative path would silently depend on the process working directory'
        )
    return value


# 声明启动参数并构造节点启动描述。
def generate_launch_description():
    share = get_package_share_directory('srm27_zlocalization_backend')
    defaults = {
        'config_file': os.path.join(share, 'config', 'backend.yaml'),
        'use_sim_time': 'false',
        'rviz': 'false',
        'mode': 'config',
        # config 保留直接启动时显式 YAML / legacy mode 的语义；导航入口一律传 true/false。
        'prior_enabled': 'config',
        'loop_enabled': 'config',
        'uwb_enabled': 'config',
        'prior_map': '',
        # 以下三项是迁入本工作区后新增的显式覆盖：同一份算法配置要同时服务仿真
        # （front_mid360）与实车（livox_imu），且轨迹/日志不允许落到进程 CWD 下。
        'sensor_frame': '',
        'odom_frame': '',
        'trajectory_directory': ''
    }

    # 解析启动上下文，生成后端与可视化组件动作。
    def setup(context):
        args = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        params = load_params(args['config_file'])
        if args['mode'] not in ('config', 'mapping', 'localization'):
            raise ValueError('mode must be config, mapping or localization')
        if args['mode'] != 'config':
            params.setdefault('prior_map', {})['enabled'] = args['mode'] == 'localization'
        for argument, section in (('prior_enabled', 'prior_map'),
                                  ('loop_enabled', 'loop'), ('uwb_enabled', 'uwb')):
            if args[argument] != 'config':
                params.setdefault(section, {})['enabled'] = launch_bool(args[argument])
        if args['prior_map']:
            params.setdefault('prior_map', {})['directory'] = require_absolute(
                'prior_map', args['prior_map'])
        if args['sensor_frame']:
            params.setdefault('common', {})['sensor_frame'] = args['sensor_frame']
        if args['odom_frame']:
            params.setdefault('common', {})['odom_frame'] = args['odom_frame']
        if args['trajectory_directory']:
            params.setdefault('trajectory', {})['save_directory'] = require_absolute(
                'trajectory_directory', args['trajectory_directory'])
        prior = bool(params.get('prior_map', {}).get('enabled', False))
        prior_directory = params.get('prior_map', {}).get('directory', '')
        require_absolute('prior_map.directory', prior_directory)
        require_absolute(
            'trajectory.save_directory',
            params.get('trajectory', {}).get('save_directory', '')
        )
        if prior and (not prior_directory or not Path(prior_directory).is_dir()):
            raise ValueError(
                'Prior map does not exist; set prior_map:=/absolute/map or mode:=mapping'
            )
        if launch_bool(args['rviz']):
            params.setdefault('visualization', {}).update(
                enabled=True,
                publish_prior_map=prior,
                publish_runtime_map=not prior,
                publish_map_axes=True,
                publish_paths=True,
                publish_debug_registration=True
            )
        actions = [
            Node(
                package='srm27_zlocalization_backend',
                executable='srm27_zlocalization_backend_node',
                name='backend_node',
                output='screen',
                parameters=[params, {'use_sim_time': launch_bool(args['use_sim_time'])}]
            )
        ]
        if launch_bool(args['rviz']):
            actions.append(
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='backend_rviz',
                    parameters=[{'use_sim_time': launch_bool(args['use_sim_time'])}],
                    arguments=['-d', os.path.join(share, 'rviz', 'backend.rviz')],
                    remappings=[
                        (
                            '/backend_node/rviz_map',
                            '/backend_node/prior_map' if prior else '/backend_node/runtime_map'
                        )
                    ]
                )
            )
        return actions

    return LaunchDescription(
        [
            *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
            OpaqueFunction(function=setup)
        ]
    )
