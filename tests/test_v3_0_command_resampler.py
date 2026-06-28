# -*- coding: utf-8 -*-
from __future__ import division
import unittest
from lily_motion_v3.command_resampler import resample_command_records, moving_average_command_records, full_command_diagnostics

class CommandResamplerTest(unittest.TestCase):
    def test_resample_inserts_intermediate_frames(self):
        records = [
            {'frame_index': 0, 'phase_name': 'A', 'joint_command_rad': [0.0, 0.0]},
            {'frame_index': 1, 'phase_name': 'A', 'joint_command_rad': [1.0, 2.0]},
        ]
        out = resample_command_records(records, factor=4)
        self.assertEqual(len(out), 5)
        self.assertAlmostEqual(out[1]['joint_command_rad'][0], 0.25)
        self.assertAlmostEqual(out[2]['joint_command_rad'][1], 1.0)
        self.assertAlmostEqual(out[-1]['joint_command_rad'][0], 1.0)

    def test_moving_average_reduces_center_spike(self):
        records = [
            {'frame_index': 0, 'joint_command_rad': [0.0]},
            {'frame_index': 1, 'joint_command_rad': [3.0]},
            {'frame_index': 2, 'joint_command_rad': [0.0]},
        ]
        out = moving_average_command_records(records, window=3)
        self.assertAlmostEqual(out[1]['joint_command_rad'][0], 1.0)

    def test_adjacent_diagnostics_detects_worst_transition(self):
        records = [
            {'frame_index': 0, 'phase_name': 'A', 'joint_command_rad': [0.0, 0.0]},
            {'frame_index': 1, 'phase_name': 'B', 'joint_command_rad': [0.1, 2.0]},
            {'frame_index': 2, 'phase_name': 'B', 'joint_command_rad': [0.2, 2.1]},
        ]
        d = full_command_diagnostics(records)
        self.assertAlmostEqual(d['max_adjacent_delta_rad'], 2.0)
        self.assertEqual(d['worst_transition']['joint_index'], 1)
        self.assertTrue(d['phase_summary'])

if __name__ == '__main__':
    unittest.main()
