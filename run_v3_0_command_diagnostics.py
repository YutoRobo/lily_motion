#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse, json, math, os, sys
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
from lily_motion_v3.interface_config import JOINT_STATE_ORDER, LEG_NAMES_BY_ID
from lily_motion_v3.command_resampler import load_command_records, full_command_diagnostics


def main():
    ap=argparse.ArgumentParser(description='Diagnose command range and adjacent-frame joint jumps.')
    ap.add_argument('--command-log', required=True)
    ap.add_argument('--top-joints', type=int, default=8)
    args=ap.parse_args()
    rows=load_command_records(args.command_log)
    if not rows:
        print(json.dumps({'frame_count':0}, indent=2)); return
    diag=full_command_diagnostics(rows)
    n=len(rows[0]['joint_command_rad'])
    mins=diag.get('mins_rad', [0.0]*n)
    maxs=diag.get('maxs_rad', [0.0]*n)
    deltas=diag.get('deltas_rad', [0.0]*n)
    adj=diag.get('per_joint_max_adjacent_delta_rad', [0.0]*n)
    joints=[]
    for i,(legacy_leg_id,joint_index) in enumerate(JOINT_STATE_ORDER):
        joint_name=['base_clause','thigh','tibia'][joint_index]
        joints.append({
            'index':i,
            'legacy_leg_id':legacy_leg_id,
            'leg_name':LEG_NAMES_BY_ID[legacy_leg_id],
            'joint':joint_name,
            'min_rad':mins[i], 'max_rad':maxs[i], 'delta_rad':deltas[i],
            'min_deg':math.degrees(mins[i]), 'max_deg':math.degrees(maxs[i]), 'delta_deg':math.degrees(deltas[i]),
            'max_adjacent_delta_rad':adj[i],
            'max_adjacent_delta_deg':math.degrees(adj[i]),
            'worst_transition_index': diag.get('per_joint_worst_transition_index', [None]*n)[i],
        })
    top_by_adj=sorted(joints, key=lambda x: x['max_adjacent_delta_rad'], reverse=True)[:max(0,args.top_joints)]
    summary={
        'command_log':args.command_log,
        'frame_count':len(rows),
        'nonzero_joint_count':diag.get('nonzero_joint_count'),
        'max_delta_rad':diag.get('max_delta_rad'),
        'max_delta_deg':diag.get('max_delta_deg'),
        'max_adjacent_delta_rad':diag.get('max_adjacent_delta_rad'),
        'max_adjacent_delta_deg':diag.get('max_adjacent_delta_deg'),
        'worst_transition':diag.get('worst_transition'),
        'phase_summary':diag.get('phase_summary'),
        'top_joints_by_adjacent_delta':top_by_adj,
        'joints':joints,
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
if __name__=='__main__': main()
