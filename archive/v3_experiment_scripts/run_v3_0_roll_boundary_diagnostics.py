#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse, json, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
from lily_motion_v3.command_resampler import load_command_records, boundary_transition_diagnostics, unwrap_continuous_command_records

def main():
    ap = argparse.ArgumentParser(description='Diagnose command jumps at roll/surface boundaries.')
    ap.add_argument('--command-log', required=True)
    ap.add_argument('--segment-key', default='roll_index')
    ap.add_argument('--top-joints', type=int, default=12)
    ap.add_argument('--unwrap-continuous-angles', action='store_true', help='Diagnose after continuous angle unwrapping as well.')
    ap.add_argument('--output', default='')
    args = ap.parse_args()
    recs = load_command_records(args.command_log)
    raw = boundary_transition_diagnostics(recs, segment_key=args.segment_key, top_joints=args.top_joints)
    report = {
        'command_log': args.command_log,
        'segment_key': args.segment_key,
        'raw_boundary_diagnostics': raw,
        'version_note': 'v3.0.34: boundary diagnostics for RF-6 -> next RF-1 command jumps.'
    }
    if args.unwrap_continuous_angles:
        unwrapped = unwrap_continuous_command_records(recs)
        report['unwrapped_boundary_diagnostics'] = boundary_transition_diagnostics(unwrapped, segment_key=args.segment_key, top_joints=args.top_joints)
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        d=os.path.dirname(args.output)
        if d and not os.path.isdir(d): os.makedirs(d)
        open(args.output,'w').write(text+'\n')
    print(text)
if __name__ == '__main__':
    main()
