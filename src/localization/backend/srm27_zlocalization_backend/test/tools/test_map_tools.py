"""在进程边界验证正式 CLI、目录隔离及同一地图包的派生产物。"""

import hashlib
import os
from pathlib import Path
import struct
import subprocess

import pytest

BINARY_DIR = Path(os.environ['BACKEND_BINARY_DIR'])
# 迁入本工作区后所有可执行文件都带包名前缀，测试仍以短名书写。
PREFIX = 'srm27_zlocalization_backend_'
TOOLS = ('build_pcd_map_bundle', 'filter_map_bundle', 'transform_map_bundle',
         'rebuild_scan_context_descriptors', 'clean_map_bundle')


def run(tool, *arguments, expected_returncode=0):
    """执行工具并验证精确退出码，区分受控拒绝与信号终止。"""
    command = [str(BINARY_DIR / (PREFIX + tool)), *map(str, arguments)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=30, check=False)
    assert result.returncode == expected_returncode, (
        f'command={command!r}, expected={expected_returncode}, actual={result.returncode}\n'
        f'stdout:\n{result.stdout}\nstderr:\n{result.stderr}'
    )
    return result


def write_cloud(path, points):
    """按 XYZ 与强度字段写出小型二进制点云。"""
    header = ('VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
              f'COUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\n'
              f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n')
    path.write_bytes(header.encode() + b''.join(struct.pack('<ffff', *p) for p in points))


def fingerprint(directory):
    """按相对路径记录地图包文件的内容摘要。"""
    return {str(path.relative_to(directory)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in directory.rglob('*') if path.is_file()}


@pytest.fixture
def bundle(tmp_path):
    """构造具有先前占据和后续自由证据的原始关键帧包。"""
    directory = tmp_path / 'recorded'
    (directory / 'clouds').mkdir(parents=True)
    rows = ['# backend_map 1']
    # 固定视点的后续回波穿过先前占据；强度保存原点身份。
    for index in range(6):
        rows.append(f'{index} {10 + index} 0 0 0 0 0 0 1')
        # 三维自由邻域需要一组覆盖射线，不能用一条射线代表空间清空。
        points = ([(2.05, .05, .05, 100)] if index == 0 else
                  [(6.05, .05 + .2*y, .05 + .2*z, 100 + index)
                   for y in range(-15, 16) for z in range(-15, 16)])
        write_cloud(directory / 'clouds' / f'{index:06d}.pcd', points)
    (directory / 'keyframes.txt').write_text('\n'.join(rows) + '\n', encoding='utf-8')
    return directory


@pytest.mark.parametrize('tool', TOOLS)
def test_help_and_unknown_option(tool):
    """检查所有 CLI 的帮助输出及未知选项退出码。"""
    assert 'Usage' in run(tool, '--help').stdout
    run(tool, '--not-a-supported-option', expected_returncode=1)


@pytest.mark.parametrize('arguments', [('--threads', '0'), ('--map-voxel', 'nan'),
                                       ('--voxel-resolution', '-1'), ('--lidar-origin', '0', '0')])
def test_invalid_cleanup_does_not_publish_output(bundle, tmp_path, arguments):
    """非法清理参数应拒绝执行，保留输入且不创建输出目录。"""
    output = tmp_path / 'invalid'
    before = fingerprint(bundle)
    run('clean_map_bundle', '--in', bundle, '--out', output, *arguments, expected_returncode=1)
    assert not output.exists()
    assert fingerprint(bundle) == before


def test_cleanup_commits_complete_reloadable_bundle_and_preserves_input(bundle, tmp_path):
    """核对清理后的完整地图包、重载能力和原始输入保留。"""
    output = tmp_path / 'clean'
    before = fingerprint(bundle)
    run('clean_map_bundle', '--in', bundle, '--out', output, '--map-voxel', '0')
    assert {'keyframes.txt', 'map.pcd', 'cleanup.txt'} <= {p.name for p in output.iterdir()}
    rows = [line.split() for line in (output / 'keyframes.txt').read_text().splitlines()
            if line and not line.startswith('#')]
    assert len(rows) == 5
    assert [int(row[0]) for row in rows] == list(range(5))
    assert [float(row[1]) for row in rows] == list(range(11, 16))
    assert len(list((output / 'clouds').glob('*.pcd'))) == 5
    run('clean_map_bundle', '--in', output, '--out', tmp_path / 'reloaded')
    assert fingerprint(bundle) == before
    complete = fingerprint(output)
    run('clean_map_bundle', '--in', bundle, '--out', output, expected_returncode=1)
    assert fingerprint(output) == complete


def test_alias_and_descendant_output_are_rejected(bundle, tmp_path):
    """拒绝与输入重合、通过符号链接重合或位于输入目录内的输出路径。"""
    alias = tmp_path / 'alias'
    alias.symlink_to(bundle, target_is_directory=True)
    before = fingerprint(bundle)
    for output in (bundle, alias, bundle / 'child'):
        run('clean_map_bundle', '--in', bundle, '--out', output, expected_returncode=1)
    assert fingerprint(bundle) == before


def test_corrupt_geometry_fails_atomically_then_retry_succeeds(bundle, tmp_path):
    """损坏几何使事务失败并清理暂存目录，修复输入后可重试成功。"""
    cloud = bundle / 'clouds/000003.pcd'
    valid = cloud.read_bytes()
    cloud.write_bytes(b'invalid PCD')
    output = tmp_path / 'clean'
    run('clean_map_bundle', '--in', bundle, '--out', output, expected_returncode=1)
    assert not output.exists()
    assert not list(tmp_path.glob('clean.staging.*'))
    cloud.write_bytes(valid)
    run('clean_map_bundle', '--in', bundle, '--out', output)


@pytest.fixture
def generated_bundle(tmp_path):
    """通过实际工具从参考点云生成双描述子地图包。"""
    reference = tmp_path / 'reference.pcd'
    points = [(float(x), float(y), float(z), float(x + y + z))
              for x in range(-4, 5) for y in range(-4, 5) for z in (1, 2, 3)]
    write_cloud(reference, points)
    output = tmp_path / 'generated'
    run('build_pcd_map_bundle', '--input', reference, '--out-bundle', output,
        '--x-min', '0', '--x-max', '1', '--y-min', '0', '--y-max', '0',
        '--bundle-step', '1', '--bundle-yaws', '0', '--max-keyframes', '2',
        '--map-voxel', '0', '--scan-voxel', '0', '--fov-down', '-80', '--fov-up', '80',
        '--descriptor-type', 'both')
    return output


def keyframe_rows(directory):
    """读取关键帧位姿索引，忽略格式说明行。"""
    return [list(map(float, line.split()))
            for line in (directory / 'keyframes.txt').read_text().splitlines()
            if line and not line.startswith('#')]


def test_build_transform_filter_and_descriptor_rebuild_form_consistent_workflow(
        generated_bundle, tmp_path):
    """串联生成、坐标变换、过滤和描述子重建，逐步核对地图包一致性。"""
    original = fingerprint(generated_bundle)
    expected_files = {'sc_descriptors.bin', 'gprobe_descriptors.bin', 'map.pcd', 'keyframes.txt'}
    assert expected_files <= original.keys()
    rows = keyframe_rows(generated_bundle)
    assert len(rows) == 2
    transformed = tmp_path / 'transformed'
    run('transform_map_bundle', '--in', generated_bundle, '--out', transformed,
        '--translation', '2', '-3', '1', '--quaternion', '0', '0', '0', '1')
    transformed_rows = keyframe_rows(transformed)
    assert len(transformed_rows) == len(rows), 'transformation changed the keyframe count'
    for before, after in zip(rows, transformed_rows):
        assert after[:2] == before[:2]
        assert after[2:5] == pytest.approx([before[2] + 2, before[3] - 3, before[4] + 1])
    for path in (generated_bundle / 'clouds').glob('*.pcd'):
        assert path.read_bytes() == (transformed / 'clouds' / path.name).read_bytes()
    reference = tmp_path / 'export.pcd'
    run('filter_map_bundle', '--in', transformed, '--export-reference-map', reference)
    filtered = tmp_path / 'filtered'
    run('filter_map_bundle', '--in', transformed, '--out', filtered,
        '--reference-map', reference, '--exact', '--min-points-per-keyframe', '1')
    assert keyframe_rows(filtered) == transformed_rows
    filtered_reference = tmp_path / 'filtered_export.pcd'
    run('filter_map_bundle', '--in', filtered, '--export-reference-map', filtered_reference)
    assert filtered_reference.read_bytes() == reference.read_bytes()
    assert (filtered / 'sc_descriptors.bin').is_file()
    assert (filtered / 'gprobe_descriptors.bin').is_file()
    before = fingerprint(filtered)
    run('rebuild_scan_context_descriptors', '--bundle', filtered)
    after = fingerprint(filtered)
    assert {k: v for k, v in before.items() if k != 'sc_descriptors.bin'} == {
        k: v for k, v in after.items() if k != 'sc_descriptors.bin'}
    run('clean_map_bundle', '--in', generated_bundle, '--out', tmp_path / 'clean_generated')
    assert (tmp_path / 'clean_generated/sc_descriptors.bin').is_file()
    assert (tmp_path / 'clean_generated/gprobe_descriptors.bin').is_file()
    assert fingerprint(generated_bundle) == original


@pytest.mark.parametrize('descriptor', ['sc_descriptors.bin', 'gprobe_descriptors.bin'])
@pytest.mark.parametrize('damage', ['truncated', 'dangling_symlink'])
def test_one_valid_descriptor_does_not_hide_another_invalid_descriptor(
        generated_bundle, tmp_path, descriptor, damage):
    """任一已有描述子损坏时均拒绝生成派生地图包。"""
    path = generated_bundle / descriptor
    path.unlink()
    if damage == 'truncated':
        path.write_bytes(b'broken descriptor')
    else:
        path.symlink_to(tmp_path / 'missing-descriptor')
    for tool, extra in [
        ('clean_map_bundle', ()),
        ('transform_map_bundle',
         ('--translation', '0', '0', '0', '--quaternion', '0', '0', '0', '1')),
        ('filter_map_bundle', ('--reference-map', generated_bundle / 'map.pcd')),
    ]:
        output = tmp_path / tool
        run(tool, '--in', generated_bundle, '--out', output, *extra, expected_returncode=1)
        assert not output.exists()
        assert not list(tmp_path.glob(tool + '.staging.*'))


@pytest.fixture
def room_bundle(tmp_path):
    """构造非对称房间点云和包含单帧描述子的地图包。"""
    directory = tmp_path / 'room'
    (directory / 'clouds').mkdir(parents=True)
    points = []
    for i in range(-5, 6):
        for j in range(-5, 6):
            u, v = .4*i, .4*j
            for xyz in ((-3, u, v), (5, u, v), (u, -4, v), (u, 6, v),
                        (u, v, -2), (u, v, 3)):
                points.append((*xyz, float(len(points))))
    write_cloud(directory / 'clouds/000000.pcd', points)
    write_cloud(directory / 'map.pcd', points)
    (directory / 'keyframes.txt').write_text('# backend_map 1\n0 10 0 0 0 0 0 0 1\n')
    run('rebuild_scan_context_descriptors', '--bundle', directory)
    return directory, points


@pytest.mark.parametrize('source_mode', ['bundle_map', 'keyframes', 'explicit_xyz'])
def test_geometric_alignment_recovers_known_transform(room_bundle, tmp_path, source_mode):
    """从三种源点云入口配准到参考地图，检查恢复已知刚体变换。"""
    import math

    original, points = room_bundle
    shifted = tmp_path / 'shifted'
    run('transform_map_bundle', '--in', original, '--out', shifted,
        '--translation', '.2', '-.1', '.05',
        '--quaternion', '0', '0', str(math.sin(.03)), str(math.cos(.03)))
    extra = []
    if source_mode == 'keyframes':
        (shifted / 'map.pcd').unlink()
    elif source_mode == 'explicit_xyz':
        source = tmp_path / 'source_xyz.pcd'
        c, s = math.cos(.06), math.sin(.06)
        xyz = [(c*x-s*y+.2, s*x+c*y-.1, z+.05) for x, y, z, _ in points]
        header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                  f'WIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n')
        source.write_bytes(header.encode() + b''.join(struct.pack('<fff', *p) for p in xyz))
        extra = ['--source-map', source]
    before = fingerprint(shifted)
    output = tmp_path / 'aligned'
    run('transform_map_bundle', '--in', shifted, '--out', output,
        '--target-map', original / 'map.pcd', '--initial-yaw-deg', '0',
        '--align-voxel', '.1', '--align-rmse-threshold', '.02',
        '--align-min-overlap', '.95', *extra)
    row = keyframe_rows(output)[0]
    assert row[:2] == [0, 10]
    assert row[2:8] == pytest.approx([0] * 6, abs=2e-4)
    assert abs(row[8]) == pytest.approx(1, abs=1e-8)
    assert fingerprint(shifted) == before
    assert (output / 'map.pcd').exists() == (source_mode != 'keyframes')


def test_alignment_rejects_unobservable_target_without_publishing_bundle(room_bundle, tmp_path):
    """单平面目标无法形成完整位姿锚时，拒绝发布变换后的地图包。"""
    original, _ = room_bundle
    plane = tmp_path / 'plane.pcd'
    write_cloud(plane, [(i*.2, j*.2, 0, 1) for i in range(-10, 11) for j in range(-10, 11)])
    before = fingerprint(original)
    output = tmp_path / 'invalid_alignment'
    run('transform_map_bundle', '--in', original, '--out', output,
        '--target-map', plane, '--initial-yaw-deg', '0', expected_returncode=1)
    assert not output.exists()
    assert not list(tmp_path.glob('invalid_alignment.staging.*'))
    assert fingerprint(original) == before
