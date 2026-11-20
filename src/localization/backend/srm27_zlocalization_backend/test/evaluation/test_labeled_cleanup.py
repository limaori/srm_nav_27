"""可选的完整公开序列验收；算法驱动与标签评分分进程。"""

import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess

import numpy as np
import pytest

from metrics import map_metrics, point_metrics, voxel_keys

GEOMETRY_SHA256 = 'e60c1a89d711f43803a3e0cf7e2381525fdae25c6390bdf7abd54ed289142858'
LABELS_SHA256 = '454442bea39390020f0cf0888cc6e167c6ca43671f58265ef7e3ea605e341d01'


def quaternion_rotation(values):
    """将位姿四元数转换为三维旋转矩阵。"""
    x, y, z, w = np.asarray(values) / np.linalg.norm(values)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def records(path):
    """按评估数据格式读取逐帧几何与标签记录。"""
    with path.open('rb') as stream:
        assert stream.read(8) == b'ZLIOMOS1'
        while header := stream.read(72):
            assert len(header) == 72
            stamp, sequence, count, *pose = struct.unpack('<qII7d', header)
            points = np.frombuffer(stream.read(count * 12), dtype='<f4').reshape(count, 3)
            yield stamp, sequence, np.asarray(pose), points


def read_binary_pcd(path):
    """读取二进制 PCD 的字段布局与点记录。"""
    with path.open('rb') as stream:
        header = {}
        while line := stream.readline():
            parts = line.decode('ascii').split()
            if parts and not parts[0].startswith('#'):
                header[parts[0]] = parts[1:]
                if parts[0] == 'DATA':
                    break
        assert header['DATA'] == ['binary']
        assert header['FIELDS'] == ['x', 'y', 'z', 'intensity']
        assert header['SIZE'] == ['4'] * 4 and header['TYPE'] == ['F'] * 4
        points = np.frombuffer(stream.read(), dtype='<f4').reshape(-1, 4)
        assert len(points) == int(header['POINTS'][0])
        return points[:, :3]


@pytest.mark.parametrize('stride, floors', [(1, (.90, .85, .99, .93)),
                                           (5, (.88, .80, .99, .90))])
def test_labeled_stairs(tmp_path, stride, floors):
    """在公开 stairs 标签数据上运行清理并核对逐点、地图级指标。"""
    data = Path(os.environ['LOCALIZATION_TEST_DATA_DIR'])
    for name, expected in [('geometry.bin', GEOMETRY_SHA256), ('labels.bin', LABELS_SHA256)]:
        assert hashlib.sha256((data / name).read_bytes()).hexdigest() == expected, name
    manifest = json.loads((data / 'manifest.json').read_text(encoding='utf-8'))
    assert manifest['geometry_sha256'] == GEOMETRY_SHA256
    assert manifest['labels_sha256'] == LABELS_SHA256
    assert manifest['label_mapping'] == {'9': 'static', '251': 'dynamic'}
    assert len(manifest['frames']) == 486
    output = tmp_path / 'result'
    result = subprocess.run([os.environ['CLEANUP_DATASET_DRIVER'], str(data / 'geometry.bin'),
                             str(output), str(stride)], capture_output=True, text=True, timeout=90)
    assert result.returncode == 0, result.stdout + result.stderr
    labels = np.fromfile(data / 'labels.bin', dtype='<u2')
    masks = np.fromfile(output / 'mask.bin', dtype=np.uint8)
    static_truth, dynamic_truth = set(), set()
    selected_dynamic, selected_removed = [], []
    label_offset = mask_offset = selected_frames = 0
    for _, sequence, pose, points in records(data / 'geometry.bin'):
        frame_labels = labels[label_offset:label_offset + len(points)]
        label_offset += len(points)
        if sequence % stride:
            continue
        selected_frames += 1
        mask = masks[mask_offset:mask_offset + len(points)]
        mask_offset += len(points)
        ranges = np.linalg.norm(points.astype(np.float64), axis=1)
        valid = np.isfinite(points).all(axis=1) & (ranges >= .5) & (ranges <= 100)
        assert np.array_equal(mask != 255, valid)
        assert np.isin(frame_labels[valid], [9, 251]).all()
        selected_dynamic.append(frame_labels[valid] == 251)
        selected_removed.append(mask[valid] == 0)
        world = points.astype(np.float64) @ quaternion_rotation(pose[3:]).T + pose[:3]
        static_truth.update(voxel_keys(world[valid & (frame_labels == 9)], .1))
        dynamic_truth.update(voxel_keys(world[valid & (frame_labels == 251)], .1))
    assert label_offset == len(labels) and mask_offset == len(masks)
    assert selected_frames == (486 if stride == 1 else 98)
    point_scores = point_metrics(np.concatenate(selected_dynamic), np.concatenate(selected_removed))
    output_voxels = voxel_keys(read_binary_pcd(output / 'map.pcd'), .1)
    map_scores = map_metrics(static_truth, dynamic_truth, output_voxels)
    scores = {'stride': stride, 'frames': selected_frames, 'point': point_scores, 'map': map_scores,
              'input_range_m': [.5, 100], 'output_voxel_m': .05, 'truth_voxel_m': .1,
              'geometry_sha256': GEOMETRY_SHA256, 'labels_sha256': LABELS_SHA256}
    (tmp_path / 'scores.json').write_text(json.dumps(scores, indent=2), encoding='utf-8')
    print(json.dumps(scores, sort_keys=True))
    for actual, floor in zip((point_scores['precision'], point_scores['recall'],
                              map_scores['preservation'], map_scores['rejection']), floors):
        assert actual is not None and actual >= floor, scores
