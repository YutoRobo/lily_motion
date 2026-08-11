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
        if key in record:
            value = record[key]
            if not isinstance(value, list) or len(value) != POSITION_LENGTH:
                raise ValueError('record %d: %s must contain exactly %d positions' % (index, key, POSITION_LENGTH))
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
                text = raw.decode('utf-8') if not isinstance(raw, str) else raw
                record = json.loads(text)
            except Exception as exc:
                raise ValueError('line %d: invalid JSON: %s' % (physical_line, exc))
            _position(record, len(records))
            if 'roll_index' not in record:
                raise ValueError('record %d: roll_index is required' % len(records))
            try:
                roll_index = int(record['roll_index'])
            except Exception:
                raise ValueError('record %d: roll_index must be integer-compatible' % len(records))
            record['_semantic_roll_index'] = roll_index
            records.append(record)
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
        ri = record['_semantic_roll_index']
        if current is None:
            current = ri
            seen.add(ri)
            start = index
            continue
        if ri != current:
            blocks.append({'roll_index': current, 'start_index': start, 'end_index': index - 1})
            if ri in seen:
                raise ValueError('roll_index %d reappears after its block ended; semantic blocks are not contiguous' % ri)
            seen.add(ri)
            current = ri
            start = index
    blocks.append({'roll_index': current, 'start_index': start, 'end_index': len(records) - 1})
    if len(blocks) != expected_roll_count:
        raise ValueError('expected %d semantic roll blocks, found %d: %s' % (
            expected_roll_count, len(blocks), [b['roll_index'] for b in blocks]))
    for block in blocks:
        block['frame_count'] = block['end_index'] - block['start_index'] + 1
    return blocks


def build(source, output_dir, expected_roll_count=4, dry_run=False):
    records, raw_lines = load_records(source)
    blocks = detect_blocks(records, expected_roll_count)
    source_sha = _sha256(source)
    stages = []
    for quarter, block in enumerate(blocks, start=1):
        count = block['end_index'] + 1
        name = 'roll_to_%dof%d_commands.jsonl' % (quarter, expected_roll_count)
        out_path = os.path.join(output_dir, name)
        stage = {
            'quarter': quarter,
            'roll_index': block['roll_index'],
            'source_end_index_inclusive': block['end_index'],
            'frame_count': count,
            'path': name,
        }
        if not dry_run:
            if not os.path.isdir(output_dir):
                os.makedirs(output_dir)
            with open(out_path, 'wb') as f:
                for raw in raw_lines[:count]:
                    f.write(raw)
            stage['sha256'] = _sha256(out_path)
        stages.append(stage)

    manifest = {
        'schema_version': 1,
        'source_command_log': os.path.abspath(source),
        'source_sha256': source_sha,
        'source_frame_count': len(records),
        'expected_roll_count': expected_roll_count,
        'roll_blocks': blocks,
        'stages': stages,
        'rule': 'Each stage is a cumulative prefix ending at the final frame of the corresponding contiguous roll_index block.',
    }
    if not dry_run:
        manifest_path = os.path.join(output_dir, 'quarter_stage_manifest.json')
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2, sort_keys=True)
            f.write('\n')
    return manifest


def parse_args(argv):
    ap = argparse.ArgumentParser(description='Build cumulative 1/4..4/4 roll command logs from semantic roll_index blocks.')
    ap.add_argument('--command-log', required=True)
    ap.add_argument('--output-dir', required=True)
    ap.add_argument('--expected-roll-count', type=int, default=4)
    ap.add_argument('--dry-run', action='store_true')
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.expected_roll_count <= 0:
        raise SystemExit('--expected-roll-count must be positive')
    try:
        manifest = build(args.command_log, args.output_dir, args.expected_roll_count, args.dry_run)
    except Exception as exc:
        print('error: %s' % exc, file=sys.stderr)
        return 2
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
