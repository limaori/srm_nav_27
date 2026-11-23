"""运行两个入口的 dry-run，验证三开关八种组合；不启动或清理 ROS 进程。"""

import os
from pathlib import Path
import shlex
import subprocess

import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[4]


def run_cli(platform, *args, **environment):
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(('ZLOC_', 'START_', 'USE_', 'MAP_', 'LIO_'))}
    env.update(DRY_RUN='1', PREKILL='0', **environment)
    return subprocess.run(
        ['bash', str(ROOT / f'scripts/bringup/start_{platform}_nav.sh'),
         '--lio', '--lio-frontend', 'zlio', '--no-rviz', *args],
        cwd=ROOT, env=env, text=True, capture_output=True, timeout=30)


def check_independent_feature_switches(platform, mask, tmp_path):
    bundle = tmp_path / 'prior bundle'
    bundle.mkdir()
    (bundle / 'keyframes.txt').write_text('test fixture\n')
    config = tmp_path / 'backend config.yaml'
    config.write_text('prior_map: {enabled: true}\nloop: {enabled: true}\nuwb: {enabled: true}\n')
    args = ['--zloc-backend', '--zloc-config', str(config)]
    for key, bit in (('prior', 1), ('loop', 2), ('uwb', 4)):
        if mask & bit:
            args.append(f'--zloc-{key}')
    if mask & 1:
        args.extend(['--zloc-map', str(bundle)])
    # 即使 shell 遗留同名环境变量，也不能改变省略开关的含义。
    result = run_cli(platform, *args, ZLOC_PRIOR='1', ZLOC_LOOP='1', ZLOC_UWB='1')
    assert result.returncode == 0, result.stdout + result.stderr
    for key, bit in (('prior', 1), ('loop', 2), ('uwb', 4)):
        assert f'zlocalization_{key}_enabled:={int(bool(mask & bit))}' in result.stdout
    assert 'zlocalization_backend_config:=' in result.stdout
    assert 'backend\\ config.yaml' in result.stdout


def check_map_path_and_dependencies(platform, tmp_path):
    result = run_cli(platform, '--zloc-backend', '--zloc-map', str(tmp_path / 'unused'))
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'zlocalization_prior_enabled:=0' in result.stdout
    for flag in ('--zloc-prior', '--zloc-loop', '--zloc-uwb'):
        result = run_cli(platform, flag)
        assert result.returncode != 0
        assert '需要 --zloc-backend' in result.stderr
    result = run_cli(platform, '--zloc-backend', '--zloc-prior')
    assert result.returncode != 0
    assert '--zloc-map' in result.stderr


class BackendCliTest(unittest.TestCase):
    def test_all_feature_combinations(self):
        for platform in ('real', 'sim'):
            for mask in range(8):
                with self.subTest(platform=platform, mask=mask), tempfile.TemporaryDirectory() as directory:
                    check_independent_feature_switches(platform, mask, Path(directory))

    def test_map_path_and_dependencies(self):
        for platform in ('real', 'sim'):
            with self.subTest(platform=platform), tempfile.TemporaryDirectory() as directory:
                check_map_path_and_dependencies(platform, Path(directory))

    def test_backend_with_decision_and_no_prior_map(self):
        trajectory = '/tmp/trajectory output'
        for platform, entry, tree in (('real', 'real', 'rmul_2027'),
                                      ('sim', 'simulation', 'rmuc_2027')):
            with self.subTest(platform=platform):
                extra = ('--no-chassis', '--gimbal-joint-topic', 'test_angle') if platform == 'real' else ()
                result = run_cli(platform, '--zloc-backend', '--zloc-loop',
                                 '--zloc-trajectory', trajectory, '--behavior', tree, *extra)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                commands = [shlex.split(line[line.index('ros2 launch '):].removesuffix('; exec bash'))
                            for line in result.stdout.splitlines() if 'ros2 launch ' in line]
                nav = next(cmd for cmd in commands if cmd[3] == f'nav_srm_{entry}_launch.py')
                behavior = next(cmd for cmd in commands if cmd[2] == 'srm27_behavior')
                self.assertFalse(any(arg.endswith(':=') for arg in nav), nav)
                self.assertIn('use_zlocalization_backend:=True', nav)
                self.assertIn('zlocalization_loop_enabled:=1', nav)
                self.assertIn(f'zlocalization_trajectory_directory:={trajectory}', nav)
                self.assertIn('decision_required:=true', nav)
                self.assertIn(f'target_tree:={tree}', behavior)
                if platform == 'real':
                    self.assertIn('gimbal_joint_topic:=test_angle', nav)


if __name__ == '__main__':
    unittest.main()
