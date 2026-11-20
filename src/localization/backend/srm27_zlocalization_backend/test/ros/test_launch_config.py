"""验证配置继承、参数覆盖及启动错误处理。"""

import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location(
    'backend_launch', Path(__file__).resolve().parents[2] / 'launch/backend.launch.py')
backend_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(backend_launch)


@pytest.mark.parametrize('text, expected',
                         [('true', True), ('OFF', False), ('1', True), ('0', False)])
def test_boolean_values(text, expected):
    """核对启动布尔值的受支持文本形式。"""
    assert backend_launch.launch_bool(text) is expected


def test_invalid_boolean_is_not_silently_true():
    """未知布尔文本应显式报错。"""
    with pytest.raises(ValueError):
        backend_launch.launch_bool('maybe')


def test_relative_inheritance_merges_leaves_without_losing_siblings(tmp_path):
    """相对父配置按叶项合并，子项覆盖后保留父配置的其他字段。"""
    (tmp_path / 'base.yaml').write_text('graph: {lag: 30, noise: 0.1}\n', encoding='utf-8')
    (tmp_path / 'child.yaml').write_text('extends: base.yaml\ngraph: {lag: 10}\n', encoding='utf-8')
    result = backend_launch.load_params(tmp_path / 'child.yaml')
    assert result == {'graph': {'lag': 10, 'noise': .1}}


def test_cycle_and_nonmapping_are_rejected(tmp_path):
    """拒绝循环继承和根节点类型无效的 YAML。"""
    (tmp_path / 'a.yaml').write_text('extends: b.yaml\n', encoding='utf-8')
    (tmp_path / 'b.yaml').write_text('extends: a.yaml\n', encoding='utf-8')
    with pytest.raises(ValueError):
        backend_launch.load_params(tmp_path / 'a.yaml')
    (tmp_path / 'list.yaml').write_text('[1, 2, 3]\n', encoding='utf-8')
    with pytest.raises(ValueError):
        backend_launch.load_params(tmp_path / 'list.yaml')


def launch_actions(config, **overrides):
    """解析声明与覆盖参数，执行生产启动入口并返回生成的动作。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    context = LaunchContext()
    context.launch_configurations.update({'config_file': str(config), **overrides})
    description = backend_launch.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    setup = next(action for action in description.entities if isinstance(action, OpaqueFunction))
    return context, setup.execute(context)


@pytest.mark.parametrize('overrides', [
    {'mode': 'unknown'}, {'mode': 'localization', 'prior_map': 'missing'},
    {'mode': 'mapping', 'rviz': 'perhaps'},
])
def test_invalid_launch_mode_map_or_switch_fails_before_process_creation(tmp_path, overrides):
    """非法模式、地图路径或开关应在创建进程前失败。"""
    config = tmp_path / 'configuration.yaml'
    config.write_text('{}\n', encoding='utf-8')
    overrides = dict(overrides)
    if 'prior_map' in overrides:
        overrides['prior_map'] = str(tmp_path / 'missing_map')
    with pytest.raises(ValueError):
        launch_actions(config, **overrides)


def test_localization_visualization_selects_backend_and_rviz_actions(tmp_path):
    """先验定位启用可视化时生成后端与 RViz 两个节点动作。"""
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node

    config = tmp_path / 'configuration.yaml'
    config.write_text('{}\n', encoding='utf-8')
    context, actions = launch_actions(
        config, mode='localization', prior_map=str(tmp_path), rviz='true')
    assert len(actions) == 2
    assert all(isinstance(action, Node) for action in actions)
    executables = [perform_substitutions(context, normalize_to_list_of_substitutions(
        action.node_executable)) for action in actions]
    assert executables == ['srm27_zlocalization_backend_node', 'rviz2']


@pytest.mark.parametrize('mask', range(8))
def test_independent_switches_override_yaml_without_map_implying_prior(tmp_path, mask):
    """入口的三项显式开关覆盖 YAML 真值，地图路径本身不启用先验。"""
    from launch_ros.utilities import evaluate_parameters

    config = tmp_path / 'configuration.yaml'
    config.write_text('prior_map: {enabled: true}\nloop: {enabled: true}\n'
                      'uwb: {enabled: true}\n', encoding='utf-8')
    context, actions = launch_actions(
        config, prior_map=str(tmp_path),
        prior_enabled=str(bool(mask & 1)), loop_enabled=str(bool(mask & 2)),
        uwb_enabled=str(bool(mask & 4)))
    params = evaluate_parameters(context, actions[0]._Node__parameters)[0]
    for section, bit in (('prior_map', 1), ('loop', 2), ('uwb', 4)):
        assert params[f'{section}.enabled'] is bool(mask & bit)


def test_shipping_profiles_default_all_features_off():
    root = Path(__file__).resolve().parents[2] / 'config'
    for name in ('backend.yaml', 'srm27_localization_real.yaml', 'srm27_localization_sim.yaml'):
        params = backend_launch.load_params(root / name)
        assert all(params[section]['enabled'] is False for section in ('prior_map', 'loop', 'uwb'))


def test_disabled_prior_needs_no_map_and_enabled_prior_requires_one(tmp_path):
    config = tmp_path / 'configuration.yaml'
    config.write_text('{}\n', encoding='utf-8')
    launch_actions(config, prior_enabled='false', loop_enabled='true', uwb_enabled='true')
    with pytest.raises(ValueError):
        launch_actions(config, prior_enabled='true')
    for flag in ('prior_enabled', 'loop_enabled', 'uwb_enabled'):
        with pytest.raises(ValueError):
            launch_actions(config, **{flag: 'maybe'})
