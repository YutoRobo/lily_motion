from __future__ import print_function
import unittest
from lily_motion_v3.command_resampler import resample_command_records, moving_average_command_records


def rec(i, roll, q):
    return {'frame_index': i, 'roll_index': roll, 'phase_name': 'p', 'joint_command_rad': [float(q)]}


class TestSegmentedResampler(unittest.TestCase):
    def test_resample_does_not_interpolate_across_roll_boundary(self):
        records = [rec(0, 0, 0.0), rec(1, 0, 1.0), rec(2, 1, 100.0), rec(3, 1, 101.0)]
        out = resample_command_records(records, factor=2, segment_key='roll_index')
        vals = [r['joint_command_rad'][0] for r in out]
        # Without segmentation, 50.5 would appear between 1 and 100.  It must not.
        self.assertNotIn(50.5, vals)
        self.assertIn(0.5, vals)
        self.assertIn(100.5, vals)

    def test_moving_average_resets_at_roll_boundary(self):
        records = [rec(0, 0, 0.0), rec(1, 0, 0.0), rec(2, 1, 100.0), rec(3, 1, 100.0)]
        out = moving_average_command_records(records, window=3, segment_key='roll_index')
        vals = [r['joint_command_rad'][0] for r in out]
        self.assertEqual(vals, [0.0, 0.0, 100.0, 100.0])


if __name__ == '__main__':
    unittest.main()
