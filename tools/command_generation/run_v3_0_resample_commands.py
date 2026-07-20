#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse, json, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
from lily_motion_v3.command_resampler import (
    load_command_records, write_command_records, resample_command_records,
    moving_average_command_records, full_command_diagnostics, unwrap_continuous_command_records, boundary_transition_diagnostics)


def main():
    ap = argparse.ArgumentParser(description='Resample/smooth an existing joint_command_rad JSONL for Gazebo preview.')
    ap.add_argument('--input', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--resample-factor', type=int, default=1)
    ap.add_argument('--smooth-window', type=int, default=1)
    ap.add_argument('--segment-key', default='', help='Optional record key such as roll_index. If set, resampling and smoothing are reset at segment boundaries.')
    ap.add_argument('--unwrap-continuous-angles', action='store_true', help='Before resampling/smoothing, replace each joint angle by the 2*pi-equivalent value closest to the previous frame. This does not average across boundaries.')
    ap.add_argument('--diagnose-boundaries', action='store_true', help='Print roll/surface boundary jump diagnostics before and after processing.')
    args = ap.parse_args()
    records = load_command_records(args.input)
    before = full_command_diagnostics(records)
    segment_key = args.segment_key.strip() or None
    before_boundary = boundary_transition_diagnostics(records, segment_key=segment_key or 'roll_index') if args.diagnose_boundaries else None
    if args.unwrap_continuous_angles:
        records = unwrap_continuous_command_records(records)
    out = resample_command_records(records, factor=args.resample_factor, segment_key=segment_key)
    out = moving_average_command_records(out, window=args.smooth_window, segment_key=segment_key)
    write_command_records(out, args.output)
    after = full_command_diagnostics(out)
    summary = {
        'version_note': 'v3.0.34: segmented smoothing plus optional continuous angle unwrapping for Gazebo preview; gait state machine is unchanged.',
        'input': args.input,
        'output': args.output,
        'resample_factor': args.resample_factor,
        'smooth_window': args.smooth_window,
        'segment_key': args.segment_key,
        'unwrap_continuous_angles': args.unwrap_continuous_angles,
        'before': {
            'frame_count': before.get('frame_count'),
            'max_adjacent_delta_deg': before.get('max_adjacent_delta_deg'),
            'max_delta_deg': before.get('max_delta_deg'),
            'worst_transition': before.get('worst_transition'),
        },
        'boundary_before': before_boundary,
        'boundary_after': boundary_transition_diagnostics(out, segment_key=segment_key or 'roll_index') if args.diagnose_boundaries else None,
        'after': {
            'frame_count': after.get('frame_count'),
            'max_adjacent_delta_deg': after.get('max_adjacent_delta_deg'),
            'max_delta_deg': after.get('max_delta_deg'),
            'worst_transition': after.get('worst_transition'),
        }
    }
    print(json.dumps(summary, indent=2, sort_keys=True))

if __name__ == '__main__':
    main()
