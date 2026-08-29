#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import json
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class LilyCliProfileTest(unittest.TestCase):
    def setUp(self):
        path = os.path.join(ROOT, 'config', 'lily_cli_profile.json')
        with open(path, 'r') as f:
            self.profile = json.load(f)

    def test_profile_schema(self):
        self.assertEqual(self.profile['schema_version'], 1)
        self.assertEqual(self.profile['can_interface'], 'can0')
        self.assertGreaterEqual(self.profile['transport']['resample_factor'], 1)
        self.assertGreater(self.profile['transport']['rate_hz'], 0)

    def test_all_configured_stage_files_exist(self):
        candidate = self.profile['candidate']
        for name, relpath in self.profile['stages'].items():
            path = os.path.join(ROOT, candidate, relpath)
            self.assertTrue(os.path.isfile(path), '%s missing: %s' % (name, path))

    def test_expected_operator_stage_names(self):
        expected = set([
            'air-entry',
            'risk-0-50', 'risk-50-100', 'risk-100-300', 'risk-300-end',
            'roll-1of4', 'roll-2of4', 'roll-3of4', 'roll-4of4',
            'combined',
        ])
        self.assertEqual(set(self.profile['stages']), expected)


if __name__ == '__main__':
    unittest.main()
