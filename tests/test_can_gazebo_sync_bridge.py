# -*- coding: utf-8 -*-
from __future__ import division

import os
import struct
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GAZEBO_TOOLS = os.path.join(ROOT, 'tools', 'gazebo')
for path in (ROOT, GAZEBO_TOOLS):
    if path not in sys.path:
        sys.path.insert(0, path)

from can_position_sync_bridge import (
    POSITION_LENGTH,
    PositionBurstAccumulator,
    parse_candump_line,
)


def position_payload(value):
    return [0, 0, 0, 0] + list(bytearray(struct.pack('<f', float(value))))


def hash_line(axis, value):
    data = position_payload(value)
    return '(123.456789) can0 %03X#%s\n' % (
        0x400 + int(axis), ''.join('%02X' % int(x) for x in data))


class CanGazeboSyncBridgeTest(unittest.TestCase):
    def test_parse_hash_position_frame(self):
        axis, value = parse_candump_line(hash_line(12, 1.25))
        self.assertEqual(12, axis)
        self.assertAlmostEqual(1.25, value, places=6)

    def test_parse_bracket_position_frame(self):
        data = position_payload(-0.5)
        line = '(123.0) can0 405 [8] %s\n' % (
            ' '.join('%02X' % int(x) for x in data))
        axis, value = parse_candump_line(line)
        self.assertEqual(5, axis)
        self.assertAlmostEqual(-0.5, value, places=6)

    def test_ignore_non_position_can_id(self):
        line = '(123.0) can0 500#0000000000000000\n'
        self.assertIsNone(parse_candump_line(line))

    def test_ignore_position_frame_with_nonzero_prefix(self):
        line = '(123.0) can0 400#0100000000000000\n'
        self.assertIsNone(parse_candump_line(line))

    def test_burst_coalesces_after_quiet_interval(self):
        acc = PositionBurstAccumulator(quiet_sec=0.002)
        acc.accept(0, 0.1, 1.0000)
        acc.accept(1, 0.2, 1.0005)
        self.assertIsNone(acc.take_if_quiet(1.0010))
        target = acc.take_if_quiet(1.0030)
        self.assertEqual(POSITION_LENGTH, len(target))
        self.assertAlmostEqual(0.1, target[0])
        self.assertAlmostEqual(0.2, target[1])
        self.assertAlmostEqual(0.0, target[2])
        self.assertEqual(1, acc.target_count)

    def test_axes_not_in_later_burst_hold_previous_command(self):
        acc = PositionBurstAccumulator(quiet_sec=0.001)
        acc.accept(3, 0.3, 1.0)
        first = acc.take_if_quiet(1.002)
        self.assertAlmostEqual(0.3, first[3])

        acc.accept(4, 0.4, 2.0)
        second = acc.take_if_quiet(2.002)
        self.assertAlmostEqual(0.3, second[3])
        self.assertAlmostEqual(0.4, second[4])


if __name__ == '__main__':
    unittest.main()
