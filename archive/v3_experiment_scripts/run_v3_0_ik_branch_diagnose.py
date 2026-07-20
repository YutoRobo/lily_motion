#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse, json, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
from lily_motion_v3.command_resampler import load_command_records
from lily_motion_v3.legacy_ik_branch_diagnostics import LegacyIKBranchDiagnostics


def main():
    ap = argparse.ArgumentParser(description='Diagnose IK branch alternatives at a worst or specified legacy command frame.')
    ap.add_argument('--command-log', required=True)
    ap.add_argument('--frame-index', type=int, default=None)
    ap.add_argument('--leg-id', type=int, default=None)
    ap.add_argument('--surface-id', type=int, default=1)
    ap.add_argument('--legacy-body-z', type=float, default=0.35)
    ap.add_argument('--second-joint-abs-max-deg', type=float, default=95.0)
    ap.add_argument('--output', default=None)
    args = ap.parse_args()
    records = load_command_records(args.command_log)
    diag = LegacyIKBranchDiagnostics(default_body_z=args.legacy_body_z, second_joint_limit_deg=args.second_joint_abs_max_deg)
    report = diag.diagnose_frame(records, frame_index=args.frame_index, leg_id=args.leg_id, surface_id=args.surface_id)
    report['version_note'] = 'v3.0.26: worst-frame IK branch diagnostic using vendored legacy FK/IK formulas.'
    if args.output:
        d = os.path.dirname(args.output)
        if d and not os.path.isdir(d): os.makedirs(d)
        with open(args.output, 'w') as f: json.dump(report, f, indent=2, sort_keys=True)
    print(json.dumps(report, indent=2, sort_keys=True))

if __name__ == '__main__':
    main()
