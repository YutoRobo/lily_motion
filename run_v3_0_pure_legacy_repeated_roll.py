#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse, json, os, sys, warnings, math
warnings.filterwarnings("ignore", category=DeprecationWarning)
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator, write_jsonl, command_diagnostics
from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator


def parse_int_list(s):
    return [int(x) for x in str(s).split(',') if str(x).strip()]


def servo_snapshot(emu):
    out = {}
    for leg_id in range(8):
        lg = emu.legs_by_legacy_id[leg_id]
        vals = lg.getServosDeg()
        out[str(leg_id)] = [float(vals[0][0]), float(vals[1][0]), float(vals[2][0])]
    return out


def summarize_rolls(records):
    by = {}
    for r in records:
        ri = int(r.get('roll_index', -1))
        if ri < 0:
            continue
        if ri not in by:
            by[ri] = {'start_frame': r.get('frame_index'), 'surface_start': r.get('surface_start'), 'phases': []}
        by[ri]['end_frame'] = r.get('frame_index')
        by[ri]['surface_after'] = r.get('surface_after')
        by[ri]['transition'] = r.get('roll_surface_transition')
        ph = r.get('phase_name')
        if ph not in by[ri]['phases']:
            by[ri]['phases'].append(ph)
        by[ri]['end_base_pose'] = r.get('base_pose')
    return [dict([('roll_index', k)] + list(v.items())) for k, v in sorted(by.items())]


def main():
    ap = argparse.ArgumentParser(description='v3.0.31 pure legacy repeated roll: run the same vendored emulator instance through 4 forward rolls without optimization.')
    ap.add_argument('--surface-sequence', default='1,5,6,2,1')
    ap.add_argument('--move-dist', type=float, default=0.4)
    ap.add_argument('--support-dist', type=float, default=0.7)
    ap.add_argument('--max-step', type=int, default=30)
    ap.add_argument('--legacy-body-z', type=float, default=0.35)
    ap.add_argument('--initialize-step', type=int, default=100)
    ap.add_argument('--include-initialize', action='store_true')
    ap.add_argument('--goal2-dist-front', type=float, default=0.4)
    ap.add_argument('--goal2-x-scale', type=float, default=1.0)
    ap.add_argument('--goal2-pitch-scale', type=float, default=1.0)
    ap.add_argument('--goal2-landing-z', type=float, default=0.0)
    ap.add_argument('--goal3-lift-z', type=float, default=0.05)
    ap.add_argument('--goal3-target-x', type=float, default=0.2)
    ap.add_argument('--goal4-target-x', type=float, default=0.05)
    ap.add_argument('--goal5-x-scale', type=float, default=1.0)
    ap.add_argument('--goal5-pitch-scale', type=float, default=1.0)
    ap.add_argument('--rf1-current-angle-anchor', action='store_true',
                    help='v3.0.36: emit one current-servo frame at the beginning of each RF-1 phase before preswing interpolation.')
    ap.add_argument('--skip-constraints', action='store_true')
    ap.add_argument('--constraint-stride', type=int, default=4)
    ap.add_argument('--second-joint-abs-max-deg', type=float, default=95.0)
    ap.add_argument('--inter-leg-limit', type=float, default=0.04)
    ap.add_argument('--ground-tolerance', type=float, default=1e-4)
    ap.add_argument('--output-command-log', default='testdata/v3_0_31_pure_legacy_repeated_commands.jsonl')
    ap.add_argument('--report-output', default='testdata/v3_0_31_pure_legacy_repeated_report.json')
    args = ap.parse_args()

    # Keep stdout JSON clean; legacy runtime can print warnings/debug text.
    json_stdout = sys.stdout
    sys.stdout = sys.stderr

    seq = parse_int_list(args.surface_sequence)
    cfg = LegacyStateMachineConfig(
        move_dist=args.move_dist,
        support_dist=args.support_dist,
        max_step=args.max_step,
        surface_id=seq[0],
        z=args.legacy_body_z,
        initialize_step=args.initialize_step,
        include_initialize=False,
        goal2_dist_front=args.goal2_dist_front,
        goal2_x_scale=args.goal2_x_scale,
        goal2_pitch_scale=args.goal2_pitch_scale,
        goal2_landing_z=args.goal2_landing_z,
        goal3_lift_z=args.goal3_lift_z,
        goal3_target_x=args.goal3_target_x,
        goal4_target_x=args.goal4_target_x,
        goal5_x_scale=args.goal5_x_scale,
        goal5_pitch_scale=args.goal5_pitch_scale,
        rf1_current_angle_anchor=args.rf1_current_angle_anchor,
    )
    emu = LegacyStateMachineEmulator(cfg)
    generation_error = None
    candidate_completed = True
    records = []
    try:
        records = emu.run_forward_repeated(surface_sequence=seq, include_initialize=args.include_initialize)
    except Exception as e:
        candidate_completed = False
        generation_error = {'type': e.__class__.__name__, 'message': str(e)}
        records = list(getattr(emu, '_records', []))

    if args.output_command_log:
        write_jsonl(records, args.output_command_log)

    constraints = {'skipped': True}
    if not args.skip_constraints:
        try:
            stride = max(1, int(args.constraint_stride))
            eval_records = records[::stride]
            if records and (not eval_records or eval_records[-1] is not records[-1]):
                eval_records = eval_records + [records[-1]]
            ev = LegacyConstraintEvaluator(
                second_joint_limit_deg=args.second_joint_abs_max_deg,
                ground_tol=args.ground_tolerance,
                inter_leg_limit_m=args.inter_leg_limit,
                default_body_z=args.legacy_body_z)
            constraints = ev.evaluate(eval_records)
            constraints['constraint_stride'] = stride
            constraints['evaluated_frame_count'] = len(eval_records)
        except Exception as e:
            constraints = {'error': {'type': e.__class__.__name__, 'message': str(e)}}

    report = {
        'version_note': 'v3.0.36: RF-1 current-angle anchor; pure legacy repeated roll remains one emulator instance, cumulative controller_x/controller_pitch, no constrained parameterizer.',
        'profile': 'pure_legacy_repeated_roll',
        'surface_sequence': seq,
        'quarter_roll_count': max(0, len(seq)-1),
        'candidate_completed': candidate_completed,
        'generation_error': generation_error,
        'frame_count': len(records),
        'command_diagnostics': command_diagnostics(records),
        'roll_summaries': summarize_rolls(records),
        'terminal_updates': getattr(emu, '_boundary_summaries', []),
        'final_lily_posture': {
            'x': float(emu.lily.posture.x),
            'y': float(emu.lily.posture.y),
            'z': float(emu.lily.posture.z),
            'roll': float(emu.lily.posture.roll),
            'pitch': float(emu.lily.posture.pitch),
            'yaw': float(emu.lily.posture.yaw),
        },
        'final_servo_deg_by_legacy_id': servo_snapshot(emu),
        'constraints': constraints,
        'parameters': vars(args),
        'output_command_log': args.output_command_log,
    }
    if args.report_output:
        d = os.path.dirname(args.report_output)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(args.report_output, 'w') as f:
            json.dump(report, f, indent=2, sort_keys=True)
    sys.stdout = json_stdout
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
