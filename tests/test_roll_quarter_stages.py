#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import imp
import json
import os
import shutil
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODULE_PATH = os.path.join(
    ROOT, 'tools', 'command_generation', 'build_roll_quarter_stages.py')
quarter_builder = imp.load_source('quarter_builder', MODULE_PATH)


class QuarterStageBuilderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='lily_quarter_stage_test_')

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _write_source(self, name, blocks, position_key='joint_command_rad',
                      position_length=24):
        path = os.path.join(self.tmp, name)
        frame_index = 0
        with open(path, 'w') as f:
            for roll_index, count in blocks:
                for _unused in range(count):
                    record = {
                        'frame_index': frame_index,
                        'roll_index': roll_index,
                        'phase_name': 'RF_TEST',
                        position_key: [frame_index * 0.001] * position_length,
                    }
                    f.write(json.dumps(record, sort_keys=True))
                    f.write('\n')
                    frame_index += 1
        return path

    def test_builds_cumulative_prefixes_from_uneven_semantic_blocks(self):
        source = self._write_source(
            'source.jsonl', [(0, 2), (1, 3), (2, 1), (3, 4)])
        output_dir = os.path.join(self.tmp, 'out')

        manifest = quarter_builder.build(source, output_dir)

        self.assertEqual(
            [2, 5, 6, 10],
            [stage['frame_count'] for stage in manifest['stages']])
        self.assertEqual(
            [(0, 0, 1), (1, 2, 4), (2, 5, 5), (3, 6, 9)],
            [(b['roll_index'], b['start_index'], b['end_index'])
             for b in manifest['roll_blocks']])

        with open(source, 'rb') as f:
            source_lines = f.readlines()
        for quarter, expected_count in enumerate([2, 5, 6, 10], start=1):
            path = os.path.join(
                output_dir,
                'roll_to_%dof4_commands.jsonl' % quarter)
            with open(path, 'rb') as f:
                self.assertEqual(source_lines[:expected_count], f.readlines())

    def test_accepts_supported_alternate_position_key(self):
        source = self._write_source(
            'position.jsonl', [(0, 1), (1, 1), (2, 1), (3, 1)],
            position_key='position')
        manifest = quarter_builder.build(
            source, os.path.join(self.tmp, 'out_position'))
        self.assertEqual(4, manifest['source_frame_count'])

    def test_rejects_wrong_position_length(self):
        source = self._write_source(
            'bad_length.jsonl', [(0, 1), (1, 1), (2, 1), (3, 1)],
            position_length=23)
        with self.assertRaises(ValueError):
            quarter_builder.build(source, os.path.join(self.tmp, 'out_bad'))

    def test_rejects_missing_roll_index(self):
        source = os.path.join(self.tmp, 'missing_roll.jsonl')
        with open(source, 'w') as f:
            f.write(json.dumps({'joint_command_rad': [0.0] * 24}) + '\n')
        with self.assertRaises(ValueError):
            quarter_builder.build(source, os.path.join(self.tmp, 'out_missing'))

    def test_rejects_reappearing_roll_index(self):
        source = self._write_source(
            'reappear.jsonl', [(0, 1), (1, 1), (0, 1), (2, 1)])
        with self.assertRaises(ValueError):
            quarter_builder.build(source, os.path.join(self.tmp, 'out_reappear'))

    def test_rejects_unexpected_roll_count(self):
        source = self._write_source(
            'three_rolls.jsonl', [(0, 1), (1, 1), (2, 1)])
        with self.assertRaises(ValueError):
            quarter_builder.build(source, os.path.join(self.tmp, 'out_three'))

    def test_dry_run_does_not_write_output(self):
        source = self._write_source(
            'dry.jsonl', [(0, 1), (1, 2), (2, 3), (3, 4)])
        output_dir = os.path.join(self.tmp, 'dry_out')
        manifest = quarter_builder.build(
            source, output_dir, dry_run=True)
        self.assertFalse(os.path.exists(output_dir))
        self.assertEqual([1, 3, 6, 10],
                         [s['frame_count'] for s in manifest['stages']])

    def test_refuses_to_overwrite_generated_files_by_default(self):
        source = self._write_source(
            'overwrite.jsonl', [(0, 1), (1, 1), (2, 1), (3, 1)])
        output_dir = os.path.join(self.tmp, 'overwrite_out')
        quarter_builder.build(source, output_dir)
        with self.assertRaises(ValueError):
            quarter_builder.build(source, output_dir)
        quarter_builder.build(source, output_dir, overwrite=True)


if __name__ == '__main__':
    unittest.main()
