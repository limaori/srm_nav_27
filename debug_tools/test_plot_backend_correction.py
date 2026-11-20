"""先验/回环可共存后的诊断归因回归；不连接 ROS。"""

import unittest

from plot_backend_correction import Recording, parse_backend_log


class CorrectionSourcesTest(unittest.TestCase):
    def test_joint_correction_is_one_separate_event(self):
        for sources, expected in (
            ('prior_scan_to_map', 'prior'), ('uwb', 'uwb'),
            ('prior_scan_to_map+uwb', 'prior_uwb'),
        ):
            with self.subTest(sources=sources):
                event = parse_backend_log(
                    f'global correction committed: runtime=5, sources={sources}, '
                    'joint_NIS=0.1/9, delta_t=0.02m, delta_r_deg=0.1')
                self.assertEqual(event.source, expected)
                self.assertEqual(event.delta_t, .02)

    def test_prior_event_does_not_prove_loop_disabled(self):
        recording = Recording(None)
        event = parse_backend_log(
            'global correction committed: runtime=1, sources=prior_scan_to_map, '
            'delta_t=0m, delta_r_deg=0')
        recording.add_event(event)
        self.assertIsNone(recording.loop_override)
        self.assertFalse(recording.observe_backend_configuration(event.raw))

    def test_explicit_capabilities_replace_previous_disabled_status(self):
        recording = Recording(None)
        recording.observe_backend_configuration(
            'configuration override: loop.enabled requested=true -> effective=false (no descriptor)')
        self.assertIsNotNone(recording.loop_override)
        recording.observe_backend_configuration(
            'unified backend ready: prior_map=enabled, loop=enabled, uwb=enabled')
        self.assertIsNone(recording.loop_override)
        recording.observe_backend_configuration(
            'unified backend ready: prior_map=disabled, loop=disabled, uwb=disabled')
        self.assertIsNotNone(recording.loop_override)


if __name__ == '__main__':
    unittest.main()
