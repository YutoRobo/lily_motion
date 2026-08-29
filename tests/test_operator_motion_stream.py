# -*- coding: utf-8 -*-
from __future__ import division

import imp
import math
import os
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODULE_PATH = os.path.join(ROOT, 'tools', 'operator_ui', 'motion_stream.py')
AIR = os.path.join(
    ROOT, 'data', 'reference_candidates',
    'v3_0_44_candidate_022_wide_urdf0p075', 'staged',
    'air_entry_and_hold_only_commands.jsonl')
ROLL4 = os.path.join(
    ROOT, 'data', 'reference_candidates',
    'v3_0_44_candidate_022_wide_urdf0p075', 'staged',
    'roll_to_4of4_commands.jsonl')

motion_stream = imp.load_source('operator_motion_stream_under_test', MODULE_PATH)


class OperatorMotionStreamTest(unittest.TestCase):
    def test_air_entry_starts_from_home_zero(self):
        loaded = motion_stream.load_motion_stream(AIR, 2)
        self.assertTrue(loaded['source_frame_count'] > 0)
        self.assertTrue(loaded['transport_frame_count'] >= loaded['source_frame_count'])
        self.assertEqual(24, len(loaded['first_position']))
        self.assertLess(max(abs(v) for v in loaded['first_position']), 1e-12)
        result = motion_stream.continuity([0.0] * 24, loaded['first_position'])
        self.assertTrue(result['pass'])

    def test_air_entry_to_full_roll_boundary_is_continuous(self):
        air = motion_stream.load_motion_stream(AIR, 2)
        roll = motion_stream.load_motion_stream(ROLL4, 2)
        result = motion_stream.continuity(
            air['last_position'], roll['first_position'])
        self.assertTrue(
            result['pass'],
            'air-entry -> 4of4 boundary jump %.6f rad axis%s' % (
                result['max_delta_rad'], result['axis']))

    def test_four_degree_or_larger_boundary_is_rejected(self):
        first = [0.0] * 24
        first[10] = math.radians(4.0)
        result = motion_stream.continuity([0.0] * 24, first)
        self.assertFalse(result['pass'])
        self.assertEqual(10, result['axis'])

    def test_bad_resample_factor_is_rejected(self):
        with self.assertRaises(motion_stream.MotionStreamError):
            motion_stream.load_motion_stream(AIR, 0)


if __name__ == '__main__':
    unittest.main()
