#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""capture_gimbal_cloud_rotation.py 的回归测试；不连接 ROS，也不出图。

跑法（在 debug_tools 目录下）：

    python3 -m unittest test_capture_gimbal_cloud_rotation -v
"""

import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

import capture_gimbal_cloud_rotation as tool


def room_points(seed=7, samples=9000):
    """14x10 m 房间 + 斜墙 + 柱子的表面点，和 --demo 的场景同构。"""
    rng = np.random.default_rng(seed)
    chunks = []
    count = samples
    chunks.append(np.column_stack([
        rng.uniform(-7.0, 7.0, count), rng.uniform(-5.0, 5.0, count),
        rng.normal(0.0, 0.008, count)]))
    for side in range(4):
        count = samples // 2
        z = rng.uniform(0.0, 3.0, count)
        if side < 2:
            x = rng.uniform(-7.0, 7.0, count)
            y = np.full(count, 5.0 if side == 0 else -5.0)
        else:
            y = rng.uniform(-5.0, 5.0, count)
            x = np.full(count, 7.0 if side == 2 else -7.0)
        chunks.append(np.column_stack([x, y, z]))
    count = samples // 3
    t = rng.uniform(0.0, 1.0, count)
    chunks.append(np.column_stack([-4.0 + 3.0 * t, -2.0 + 3.5 * t,
                                   rng.uniform(0.0, 2.2, count)]))
    return np.concatenate(chunks, axis=0)


class AlignerTest(unittest.TestCase):
    def setUp(self):
        self.settings = tool.Settings(
            align_points=900, inlier_dist=0.15, max_shift=0.6,
            yaw_range=30.0, coarse_step=3.0, fine_step=0.3,
            min_range=0.0, max_range=30.0)

    def test_recovers_known_rotation_and_shift(self):
        scene = room_points()
        rng = np.random.default_rng(3)
        reference = scene[rng.choice(len(scene), 1800, replace=False)]
        angle = math.radians(12.3)
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        shift = np.array([0.05, -0.03])
        delta = reference[:, :2]
        current = reference.copy()
        current[:, 0] = cos_a * delta[:, 0] - sin_a * delta[:, 1] + shift[0]
        current[:, 1] = sin_a * delta[:, 0] + cos_a * delta[:, 1] + shift[1]

        aligner = tool.Aligner(self.settings)
        aligner.set_reference(reference, np.zeros(2))
        result = aligner.align(current, np.zeros(2))
        self.assertIsNotNone(result)
        measured = -math.degrees(result["psi"])  # 当前帧在世界系里转了 +12.3°
        self.assertAlmostEqual(measured, 12.3, delta=0.6)
        self.assertAlmostEqual(result["shift_norm"], float(np.linalg.norm(shift)), delta=0.06)
        self.assertGreater(result["inlier"], 0.6)

    def test_static_cloud_is_near_zero(self):
        scene = room_points()
        rng = np.random.default_rng(4)
        a = scene[rng.choice(len(scene), 1800, replace=False)]
        b = scene[rng.choice(len(scene), 1800, replace=False)]
        aligner = tool.Aligner(self.settings)
        aligner.set_reference(a, np.zeros(2))
        result = aligner.align(b, np.zeros(2))
        self.assertIsNotNone(result)
        self.assertLess(abs(-math.degrees(result["psi"])), 0.5)


class ClassifyTest(unittest.TestCase):
    def test_thresholds(self):
        self.assertEqual(tool.classify(0.03, 0.1, 20, 40, 10), "stable")
        self.assertEqual(tool.classify(0.98, 0.99, 20, 40, 10), "follow_pos")
        self.assertEqual(tool.classify(-0.95, -0.99, 20, 40, 10), "follow_neg")
        self.assertEqual(tool.classify(1.92, 0.99, 20, 40, 10), "follow_2x")
        self.assertEqual(tool.classify(-1.9, -0.95, 20, 40, 10), "follow_2x")
        self.assertEqual(tool.classify(0.45, 0.8, 20, 40, 10), "partial")
        self.assertEqual(tool.classify(0.9, 0.2, 20, 40, 10), "unclear")
        self.assertEqual(tool.classify(1.0, 1.0, 3, 40, 10), "insufficient")
        self.assertEqual(tool.classify(1.0, 1.0, 20, 5, 10), "insufficient")

    def test_wrap_is_nan_safe(self):
        wrapped = tool.wrap_deg_array(np.array([np.nan, 190.0, -190.0]))
        self.assertTrue(np.isnan(wrapped[0]))
        self.assertAlmostEqual(wrapped[1], -170.0)
        self.assertAlmostEqual(wrapped[2], 170.0)

    def test_translation_orbit_radius_is_recovered(self):
        angles = np.linspace(-50.0, 50.0, 101)
        theta = np.radians(angles)
        radius = 0.12
        xs = radius * np.cos(theta + 0.4) + 0.03
        ys = radius * np.sin(theta + 0.4) - 0.02
        got, residual = tool.fit_translation_orbit(angles, xs, ys)
        self.assertAlmostEqual(got, radius, delta=1e-6)
        self.assertLess(residual, 1e-9)

    def test_linear_fit_sign_is_physical(self):
        # Δ点云转角 = -Δ云台角（漏补）时，斜率必须是负的。
        angle = np.linspace(-5.0, 5.0, 21)
        slope, corr, n = tool.linear_fit(angle, -angle)
        self.assertAlmostEqual(slope, -1.0)
        self.assertAlmostEqual(corr, -1.0)
        self.assertEqual(n, 21)


class BundleTest(unittest.TestCase):
    def test_roundtrip(self):
        stamps = np.array([10.0, 10.2])
        frames = [
            np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32),
            np.array([[7, 8, 9]], dtype=np.float32),
        ]
        recording = tool.TopicRecording(
            topic="/test/cloud",
            stamps=stamps,
            frame_ids=["odom", "odom"],
            points=frames,
            intensity=[None, None],
            T_odom_frame=np.tile(np.eye(4), (2, 1, 1)),
            T_odom_lidar=np.tile(np.eye(4), (2, 1, 1)),
            T_map_odom=np.tile(np.eye(4), (2, 1, 1)),
            gimbal_deg=np.array([1.0, 2.0]),
            base_deg=np.array([0.0, 0.0]),
            tf_ok=np.array([True, True]),
        )
        samples = tool.samples_to_arrays([
            {"t_stamp": 10.0, "joint_deg": -1.0},
            {"t_stamp": 10.2, "joint_deg": -2.0},
        ])
        with tempfile.TemporaryDirectory() as folder:
            bundle = Path(folder) / "bundle"
            tool.write_bundle(bundle, samples, {recording.topic: recording},
                              {"tool": "test", "capture": {}})
            loaded_bundle, manifest, loaded_samples, recordings = tool.load_bundle(bundle)
            self.assertEqual(manifest["tool"], "test")
            self.assertEqual(len(loaded_samples["t_stamp"]), 2)
            loaded = recordings["/test/cloud"]
            self.assertEqual(len(loaded), 2)
            np.testing.assert_allclose(loaded.points[0], frames[0])
            np.testing.assert_allclose(loaded.points[1], frames[1])
            np.testing.assert_allclose(loaded.gimbal_deg, [1.0, 2.0])
            self.assertTrue(loaded.tf_ok.all())


class DemoEndToEndTest(unittest.TestCase):
    def test_demo_recovers_all_four_cases(self):
        settings = tool.Settings(no_plot=True, no_pcd=True, analysis_frame="odom")
        samples, recordings, manifest = tool.build_demo(settings)
        with tempfile.TemporaryDirectory() as folder:
            bundle = Path(folder) / "demo"
            tool.write_bundle(bundle, samples, recordings, manifest)
            analysis = tool.analyze_bundle(bundle, settings)
            self.assertIsNotNone(analysis)
            expected = manifest["expected"]
            self.assertEqual(len(analysis.results), 4)
            for result in analysis.results:
                self.assertIn(result.topic, expected)
                self.assertAlmostEqual(result.slope, expected[result.topic], delta=0.25,
                                       msg=f"{result.topic} 斜率 {result.slope}")
                if expected[result.topic] == 0.0:
                    self.assertEqual(result.kind, "stable")
                elif expected[result.topic] > 0:
                    self.assertEqual(result.kind, "follow_pos")
                else:
                    self.assertEqual(result.kind, "follow_neg")
            self.assertTrue((bundle / "report.md").exists())
            self.assertTrue((bundle / "analysis" / "pairs.csv").exists())


if __name__ == "__main__":
    unittest.main()
