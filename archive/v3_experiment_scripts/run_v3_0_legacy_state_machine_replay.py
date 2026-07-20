#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse, json, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator, command_diagnostics, write_jsonl, FORWARD_NEXT_SURFACE


def main():
    ap = argparse.ArgumentParser(description='Generate Gazebo command JSONL using a vendored legacy roll state-machine emulator.')
    ap.add_argument('--surface-id', type=int, default=1)
    ap.add_argument('--direction', choices=['forward'], default='forward')
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
    ap.add_argument('--output', default='testdata/v3_0_22_legacy_state_machine_commands.jsonl')
    args = ap.parse_args()
    cfg = LegacyStateMachineConfig(
        move_dist=args.move_dist, support_dist=args.support_dist, max_step=args.max_step,
        surface_id=args.surface_id, z=args.legacy_body_z, initialize_step=args.initialize_step,
        include_initialize=args.include_initialize,
        goal2_dist_front=args.goal2_dist_front, goal2_x_scale=args.goal2_x_scale,
        goal2_pitch_scale=args.goal2_pitch_scale, goal2_landing_z=args.goal2_landing_z,
        goal3_lift_z=args.goal3_lift_z, goal3_target_x=args.goal3_target_x,
        goal4_target_x=args.goal4_target_x)
    emu = LegacyStateMachineEmulator(cfg)
    records = emu.run_forward_roll()
    write_jsonl(records, args.output)
    diag = command_diagnostics(records)
    summary = {
        'version_note': 'v3.0.25: vendored legacy Servo/Leg/EndEfectorManager/LilyRobot state-machine replay; no ROS/catkin import.',
        'profile': 'legacy_state_machine',
        'output': args.output,
        'surface_start': args.surface_id,
        'surface_after': FORWARD_NEXT_SURFACE.get(args.surface_id),
        'move_dist': args.move_dist,
        'support_dist': args.support_dist,
        'max_step': args.max_step,
        'include_initialize': args.include_initialize,
        'initialize_step': args.initialize_step,
        'goal2_dist_front': args.goal2_dist_front,
        'goal2_x_scale': args.goal2_x_scale,
        'goal2_pitch_scale': args.goal2_pitch_scale,
        'goal2_landing_z': args.goal2_landing_z,
        'goal3_lift_z': args.goal3_lift_z,
        'goal3_target_x': args.goal3_target_x,
        'goal4_target_x': args.goal4_target_x,
    }
    summary.update(diag)
    print(json.dumps(summary, indent=2, sort_keys=True))

if __name__ == '__main__':
    main()
