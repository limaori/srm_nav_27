"""用人工可数的标签和体素集合校验评估分母。"""

from metrics import map_metrics, point_metrics, voxel_keys


def test_point_confusion_matrix_and_precision_recall_are_distinct():
    """用已知分类结果分别核对混淆矩阵、精确率与召回率。"""
    result = point_metrics(
        [True, True, True, True, False, False], [True, True, False, False, True, False])
    assert (result['tp'], result['fp'], result['fn'], result['tn']) == (2, 1, 2, 1)
    assert result['precision'] == 2 / 3
    assert result['recall'] == 1 / 2


def test_map_scores_use_unique_voxels_with_static_precedence():
    """地图指标按唯一体素计数，同格静态标签具有优先级。"""
    static = voxel_keys([[.01, 0, 0], [.02, 0, 0], [.11, 0, 0]], .1)
    dynamic = voxel_keys([[.01, 0, 0], [.21, 0, 0], [.31, 0, 0]], .1)
    output = voxel_keys([[.01, 0, 0], [.21, 0, 0]], .1)
    assert map_metrics(static, dynamic, output) == {
        'static_voxels': 2, 'dynamic_only_voxels': 2, 'preservation': .5, 'rejection': .5}


def test_empty_denominators_are_unavailable_not_perfect():
    """分母为空时指标返回不可用值，不报告满分。"""
    assert point_metrics([], [])['recall'] is None
    assert point_metrics([False], [False])['precision'] is None
    assert map_metrics(set(), set(), set())['preservation'] is None
