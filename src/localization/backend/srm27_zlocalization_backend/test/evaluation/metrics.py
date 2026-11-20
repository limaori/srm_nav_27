"""点分类和最终地图的体素指标；两种分母不得混用。"""

import numpy as np


def voxel_keys(points, resolution):
    """按给定分辨率将点坐标映射为唯一体素键。"""
    cells = np.floor(np.asarray(points, dtype=np.float64) / resolution).astype(np.int64)
    return set(map(tuple, np.unique(cells, axis=0)))


def point_metrics(dynamic, removed):
    """按静态与动态标签统计逐点混淆矩阵、精确率和召回率。"""
    dynamic = np.asarray(dynamic, dtype=bool)
    removed = np.asarray(removed, dtype=bool)
    if dynamic.shape != removed.shape or dynamic.ndim != 1:
        raise ValueError('classification arrays must have equal one-dimensional shapes')
    tp = int(np.count_nonzero(dynamic & removed))
    fp = int(np.count_nonzero(~dynamic & removed))
    fn = int(np.count_nonzero(dynamic & ~removed))
    tn = int(np.count_nonzero(~dynamic & ~removed))
    return {'tp': tp, 'fp': fp, 'fn': fn, 'tn': tn,
            'precision': tp / (tp + fp) if tp + fp else None,
            'recall': tp / (tp + fn) if tp + fn else None}


def map_metrics(static_voxels, dynamic_voxels, output_voxels):
    """以唯一体素集合统计最终地图的静态保留与动态残留指标。"""
    # 同时出现静态和动态真值的体素按静态保留；只在纯动态体素上计删除率。
    dynamic_only = dynamic_voxels - static_voxels
    return {
        'static_voxels': len(static_voxels), 'dynamic_only_voxels': len(dynamic_only),
        'preservation': (len(static_voxels & output_voxels) / len(static_voxels)
                         if static_voxels else None),
        'rejection': (1 - len(dynamic_only & output_voxels) / len(dynamic_only)
                      if dynamic_only else None),
    }
