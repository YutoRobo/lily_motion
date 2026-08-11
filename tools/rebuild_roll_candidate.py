#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import json
import os
import subprocess
import sys


def _repo_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def _inside(path, parent):
    path = os.path.realpath(path)
    parent = os.path.realpath(parent)
    return path == parent or path.startswith(parent + os.sep)


def load_profile(path):
    with open(path) as f:
        profile = json.load(f)
    required = ('name', 'generator_argv', 'output_dir', 'generated_command_log')
    for key in required:
        if key not in profile:
            raise ValueError('profile missing required key: %s' % key)
    if not isinstance(profile['generator_argv'], list) or not profile['generator_argv']:
        raise ValueError('generator_argv must be a non-empty JSON list')
    return profile


def resolve(root, value):
    if os.path.isabs(value):
        return value
    return os.path.join(root, value)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Run an explicit roll-candidate generation profile and build semantic quarter stages.')
    ap.add_argument('--profile', required=True)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)

    root = _repo_root()
    try:
        profile = load_profile(args.profile)
    except Exception as exc:
        print('error: %s' % exc, file=sys.stderr)
        return 2

    output_dir = resolve(root, profile['output_dir'])
    reference_root = os.path.join(root, 'data', 'reference_candidates')
    testdata_root = os.path.join(root, 'testdata')
    if _inside(output_dir, reference_root):
        print('error: rebuild output must not overwrite data/reference_candidates', file=sys.stderr)
        return 2
    if not _inside(output_dir, testdata_root):
        print('error: rebuild output must stay under testdata/', file=sys.stderr)
        return 2

    substitutions = {
        '{repo}': root,
        '{output_dir}': output_dir,
        '{python}': sys.executable,
    }
    cmd = []
    for item in profile['generator_argv']:
        value = str(item)
        for token, replacement in substitutions.items():
            value = value.replace(token, replacement)
        cmd.append(value)

    command_log = resolve(root, profile['generated_command_log'].replace('{output_dir}', output_dir))
    quarter_dir_value = profile.get('quarter_stage_output_dir', '{output_dir}/staged')
    quarter_dir = resolve(root, quarter_dir_value.replace('{output_dir}', output_dir))
    expected = int(profile.get('expected_roll_count', 4))

    print('profile:', profile['name'])
    print('output_dir:', output_dir)
    print('generator:', ' '.join(cmd))
    print('generated_command_log:', command_log)
    print('quarter_stage_output_dir:', quarter_dir)
    if args.dry_run:
        return 0

    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    status = subprocess.call(cmd, cwd=root)
    if status != 0:
        print('error: generator exited with status %d' % status, file=sys.stderr)
        return status
    if not os.path.isfile(command_log):
        print('error: configured generated command log was not created: %s' % command_log, file=sys.stderr)
        return 2

    builder = os.path.join(root, 'tools', 'build_roll_quarter_stages.py')
    stage_cmd = [sys.executable, builder,
                 '--command-log', command_log,
                 '--output-dir', quarter_dir,
                 '--expected-roll-count', str(expected)]
    status = subprocess.call(stage_cmd, cwd=root)
    if status != 0:
        return status

    manifest = {
        'schema_version': 1,
        'profile': os.path.abspath(args.profile),
        'profile_name': profile['name'],
        'generator_argv_resolved': cmd,
        'generated_command_log': command_log,
        'quarter_stage_output_dir': quarter_dir,
        'note': 'A generated candidate remains under testdata until separately reviewed and promoted to data/reference_candidates.',
    }
    with open(os.path.join(output_dir, 'generation_manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write('\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
