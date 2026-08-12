#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import hashlib
import json
import os
import sys

POSITION_KEYS = ('joint_command_rad', 'position', 'joint_positions_rad')
POSITION_LENGTH = 24


def _sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            chunk = f.read(1024 * 1024)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def _position(record, index):
    for key in POSITION_KEYS:
        if key not in record:
            continue
        value = record[key]
        if not isinstance(value, list) or len(value) != POSITION_LENGTH:
            raise ValueError(
                'record %d: %s must contain exactly %d positions' %
                (index, key, POSITION_LENGTH))
        return value
    raise ValueError('record %d: no supported position key' % index)


def load_records(path):
    records = []
    raw_lines = []
    with open(path, 'rb') as f:
        for physical_line, raw in enumerate(f, start=1):
            if not raw.strip():
                continue
            try:
                text = raw.decode('utf-8')
                record = json.loads(text)
            except Exception as exc:
                raise ValueError(
                    'line %d: invalid JSON: %s' % (physical_line, exc))
            _position(record, len(records))
            if 'roll_index' not in record:
                raise ValueError(
                    'record %d: roll_index is required' % len(records))
            try:
                roll_index = int(record['roll_index'])
            except Exception:
                raise ValueError(
                    'record %d: roll_index must be integer-compatible' %
                    len(records))
            normalized = dict(record)
            normalized['_semantic_roll_index'] = roll_index
            records.append(normalized)
            raw_lines.append(raw)
    if not records:
        raise ValueError('command log contains no records')
    return records, raw_lines


def detect_blocks(records, expected_roll_count):
    blocks = []
    seen = set()
    current = None
    start = 0
    for index, record in enumerate(records):
        roll_index = record['_semantic_roll_index']
        if current is None:
            current = roll_index
            seen.add(roll_index)
            start = index
            continue
        if roll_index == current:
            continue
        blocks.append({
            'roll_index': current,
            'start_index': start,
            'end_index': index - 1,
        })
        if roll_index in seen:
            raise ValueError(
                'roll_index %d reappears after its block ended; '
                'semantic blocks are not contiguous' % roll_index)
        seen.add(roll_index)
        current = roll_index
        start = index

    blocks.append({
        'roll_index': current,
        'start_index': start,
        'end_index': len(records) - 1,
    })

    if len(blocks) != int(expected_roll_count):
        raise ValueError(
            'expected %d semantic roll blocks, found %d: %s' %
            (int(expected_roll_count), len(blocks),
             [b['roll_index'] for b in blocks]))

    for block in blocks:
        block['frame_count'] = (
            block['end_index'] - block['start_index'] + 1)
    return blocks


def _stage_filename(quarter, total):
    return 'roll_to_%dof%d_commands.jsonl' % (quarter, total)


def _planned_paths(output_dir, expected_roll_count):
    paths = [
        os.path.join(output_dir, _stage_filename(i, expected_roll_count))
        for i in range(1, expected_roll_count + 1)
    ]
    paths.append(os.path.join(output_dir, 'quarter_stage_manifest.json'))
    return paths


def _check_output_paths(output_dir, expected_roll_count, overwrite):
    if overwrite:
        return
    existing = [
        path for path in _planned_paths(output_dir, expected_roll_count)
        if os.path.exists(path)
    ]
    if existing:
        raise ValueError(
            'refusing to overwrite existing quarter-stage output(s): %s; '
            'choose a new --output-dir or pass --overwrite intentionally' %
            ', '.join(existing))


def build(source, output_dir, expected_roll_count=4,
          dry_run=False, overwrite=False):
    expected_roll_count = int(expected_roll_count)
    if expected_roll_count <= 0:
        raise ValueError('expected_roll_count must be positive')

    records, raw_lines = load_records(source)
    blocks = detect_blocks(records, expected_roll_count)
    source_sha = _sha256(source)

    if not dry_run:
        _check_output_paths(output_dir, expected_roll_count, overwrite)
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)

    stages = []
    for quarter, block in enumerate(blocks, start=1):
        count = block['end_index'] + 1
        name = _stage_filename(quarter, expected_roll_count)
        out_path = os.path.join(output_dir, name)
        stage = {
            'quarter': quarter,
            'roll_index': block['roll_index'],
            'source_end_index_inclusive': block['end_index'],
            'frame_count': count,
            'path': name,
        }
        if not dry_run:
            with open(out_path, 'wb') as f:
                for raw in raw_lines[:count]:
                    f.write(raw)
            stage['sha256'] = _sha256(out_path)
        stages.append(stage)

    manifest = {
        'schema_version': 1,
        'source_command_log': source,
        'source_sha256': source_sha,
        'source_frame_count': len(records),
        'expected_roll_count': expected_roll_count,
        'roll_blocks': blocks,
        'stages': stages,
        'rule': (
            'Each stage is a cumulative prefix ending at the final frame '
            'of the corresponding contiguous roll_index block.'),
    }

    if not dry_run:
        manifest_path = os.path.join(
            output_dir, 'quarter_stage_manifest.json')
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2, sort_keys=True)
            f.write('\n')
    return manifest


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            'Build cumulative semantic 1/N..N/N roll JSONL stages from '
            'contiguous roll_index blocks. This tool only generates files; '
            'it does not publish ROS or open CAN.'))
    parser.add_argument('--command-log', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--expected-roll-count', type=int, default=4)
    parser.add_argument(
        '--overwrite', action='store_true',
        help='Intentionally replace existing generated quarter-stage outputs')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='Validate semantic boundaries and print the manifest without writing files')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        manifest = build(
            args.command_log,
            args.output_dir,
            expected_roll_count=args.expected_roll_count,
            dry_run=args.dry_run,
            overwrite=args.overwrite)
    except Exception as exc:
        print('error: %s' % exc, file=sys.stderr)
        return 2
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
