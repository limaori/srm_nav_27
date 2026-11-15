"""Regression checks for the actual map-injection helpers; requires ROS messages."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import MapMetaData

spec = importlib.util.spec_from_file_location(
    "inject_recovery_stuck", Path(__file__).parents[1] / "inject_recovery_stuck.py"
)
inject = importlib.util.module_from_spec(spec)
spec.loader.exec_module(inject)


def info():
    metadata = MapMetaData()
    metadata.width = metadata.height = 100
    metadata.resolution = 0.1
    metadata.origin.orientation.w = 1.0
    return metadata


def test_world_rectangle_uses_occupancy_grid_rows():
    metadata = info()
    cells = np.zeros((100, 100), dtype=bool)
    inject.stamp_rect(cells, metadata, 1.9, 1.9, 2.1, 2.1)
    ys, xs = np.where(cells)
    assert np.all(np.abs((ys + 0.5) * metadata.resolution - 2.0) < 0.3)
    assert np.all(np.abs((xs + 0.5) * metadata.resolution - 2.0) < 0.3)


def test_wall_is_injected_on_requested_side_of_actual_robot():
    metadata = info()
    cells = np.zeros((100, 100), dtype=bool)
    args = SimpleNamespace(arena_radius=1.0, wall_length=1.2, wall_gap=0.2, wall_dir="+y")
    inject.build_shape(cells, metadata, 20, 30, "wall", args)
    ys, _ = np.where(cells)
    assert np.all(ys > 30)
    assert ys.max() < 40


def test_open_map_distance_field_does_not_reference_undefined_dimensions():
    _, field = inject.clearance_field(np.zeros((20, 30), dtype=bool), info())
    assert field.shape == (20, 30)
    assert np.isfinite(field).all()
