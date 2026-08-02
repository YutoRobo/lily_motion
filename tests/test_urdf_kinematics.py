# -*- coding: utf-8 -*-
from __future__ import division

import math
import os
import unittest

from lily_motion_v3.geometry import segment_segment_distance
from lily_motion_v3.urdf_kinematics import (
    UrdfLegKinematics,
    axis_rotation_matrix,
)


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURE = os.path.join(ROOT, "tests", "fixtures", "urdf", "minimal_lily_leg.urdf")


def assert_vector_close(test_case, actual, expected, places=9):
    test_case.assertEqual(len(expected), len(actual))
    for actual_value, expected_value in zip(actual, expected):
        test_case.assertAlmostEqual(actual_value, expected_value, places=places)


class UrdfKinematicsTest(unittest.TestCase):
    def setUp(self):
        self.kinematics = UrdfLegKinematics.from_robot_description(FIXTURE)

    def test_parses_topology_order_origins_axes_and_links(self):
        self.assertEqual(("BLF", "BLH"), self.kinematics.leg_names)
        chain = self.kinematics.leg_joint_summary("BLF")
        self.assertEqual([
            "BLF_coxa_joint", "BLF_base_clause_joint",
            "BLF_thigh_joint", "BLF_tibia_joint",
            "BLF_appendix_joint",
        ], [item["joint_name"] for item in chain])
        self.assertEqual("BLF_base", chain[2]["parent_link"])
        self.assertEqual("BLF_thigh", chain[2]["child_link"])
        self.assertEqual([1.0, 0.0, 0.0], chain[2]["origin_xyz"])
        self.assertAlmostEqual(math.pi / 2.0, chain[2]["origin_rpy"][2])
        self.assertEqual([0.0, 1.0, 0.0], chain[2]["axis"])

    def test_home_position_uses_origin_rpy(self):
        points = self.kinematics.link_positions_body("BLF", [0.0, 0.0, 0.0])
        assert_vector_close(self, points["body_root"], [0.0, 0.0, 0.0])
        assert_vector_close(self, points["root"], [1.0, 0.0, 0.0])
        assert_vector_close(self, points["coxa_end"], [2.0, 0.0, 0.0])
        assert_vector_close(self, points["knee"], [2.0, 1.0, 0.0])
        assert_vector_close(self, points["foot"], [2.0, 2.0, 0.0])
        thigh = [
            item for item in points["joint_positions"]
            if item["joint_name"] == "BLF_thigh_joint"][0]
        assert_vector_close(self, thigh["axis_body"], [-1.0, 0.0, 0.0])

    def test_axis_rotations_x_y_z_and_arbitrary(self):
        half_pi = math.pi / 2.0
        assert_vector_close(self,
            _apply(axis_rotation_matrix([1, 0, 0], half_pi), [0, 1, 0]),
            [0, 0, 1])
        assert_vector_close(self,
            _apply(axis_rotation_matrix([0, 1, 0], half_pi), [0, 0, 1]),
            [1, 0, 0])
        assert_vector_close(self,
            _apply(axis_rotation_matrix([0, 0, 1], half_pi), [1, 0, 0]),
            [0, 1, 0])
        arbitrary = axis_rotation_matrix([1, 1, 0], math.pi)
        assert_vector_close(self, _apply(arbitrary, [1, 0, 0]), [0, 1, 0])
        with self.assertRaises(ValueError):
            axis_rotation_matrix([0, 0, 0], 1.0)

    def test_command_order_mapping_and_size_validation(self):
        mapped = self.kinematics.command_values_by_leg(list(range(24)))
        self.assertEqual([0.0, 1.0, 2.0], mapped["BRF"])
        self.assertEqual([3.0, 4.0, 5.0], mapped["BLF"])
        self.assertEqual([9.0, 10.0, 11.0], mapped["BRH"])
        self.assertEqual([21.0, 22.0, 23.0], mapped["TRH"])
        with self.assertRaises(ValueError):
            self.kinematics.command_values_by_leg(list(range(23)))
        with self.assertRaises(ValueError):
            self.kinematics.command_values_by_leg(list(range(25)))

    def test_segments_have_expected_endpoints_and_continuity(self):
        segments = self.kinematics.leg_segments_body("BLF", [0, 0, 0])
        self.assertEqual(3, len(segments))
        assert_vector_close(self, segments[0]["a"], [1, 0, 0])
        assert_vector_close(self, segments[-1]["b"], [2, 2, 0])
        assert_vector_close(self, segments[0]["b"], segments[1]["a"])
        assert_vector_close(self, segments[1]["b"], segments[2]["a"])

    def test_missing_file_has_clear_error(self):
        with self.assertRaisesRegex(ValueError, "does not exist"):
            UrdfLegKinematics.from_robot_description(FIXTURE + ".missing")


def _apply(matrix, vector):
    return [sum(matrix[row][column] * vector[column] for column in range(3))
            for row in range(3)]


class SegmentDistanceRegressionTest(unittest.TestCase):
    def assert_distance(self, a0, a1, b0, b1, expected):
        actual, unused_a, unused_b = segment_segment_distance(a0, a1, b0, b1)
        self.assertAlmostEqual(expected, actual, places=9)

    def test_parallel(self):
        self.assert_distance([0, 0, 0], [1, 0, 0], [0, 2, 0], [1, 2, 0], 2)

    def test_intersection(self):
        self.assert_distance([-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], 0)

    def test_endpoint_nearest(self):
        self.assert_distance([0, 0, 0], [1, 0, 0], [2, 1, 0], [2, 2, 0], math.sqrt(2))

    def test_degenerate_and_same_point(self):
        self.assert_distance([0, 0, 0], [0, 0, 0], [1, 0, 0], [2, 0, 0], 1)
        self.assert_distance([1, 2, 3], [1, 2, 3], [1, 2, 3], [1, 2, 3], 0)


if __name__ == "__main__":
    unittest.main()
