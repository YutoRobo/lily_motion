#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Run the project-contained v3 one-roll concept generator."""
from __future__ import print_function
import argparse
import json

from lily_motion_v3.v3_roll_candidate_generator import V3RollCandidateGenerator, V3RollGenerationConfig


def _parse_float_list(text):
    out = []
    for part in str(text).split(','):
        part = part.strip()
        if part:
            out.append(float(part))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--surface-id", type=int, default=1)
    ap.add_argument("--contact-plan-variant", default="default")
    ap.add_argument("--steps-per-phase", type=int, default=8)
    ap.add_argument("--lift-height", type=float, default=0.08)
    ap.add_argument("--clearance-height", type=float, default=0.06)
    ap.add_argument("--candidate-support-shift-x", type=float, default=0.04)
    ap.add_argument("--candidate-support-drop-z", type=float, default=-0.02)
    ap.add_argument("--body-roll-pitch-deg", type=float, default=90.0)
    ap.add_argument("--body-roll-x-shift", type=float, default=0.0)
    ap.add_argument("--body-roll-z-shift", type=float, default=0.0)
    ap.add_argument("--disable-body-roll-pose-search", action="store_true")
    ap.add_argument("--body-roll-search-x-offsets", default="-0.20,-0.10,0.0,0.10,0.20")
    ap.add_argument("--body-roll-search-z-offsets", default="-0.10,0.0,0.10,0.20,0.30,0.40")
    ap.add_argument("--ground-z", type=float, default=0.0)
    ap.add_argument("--no-auto-align-initial-ground", action="store_true")
    ap.add_argument("--min-inter-leg-clearance", type=float, default=0.05)
    ap.add_argument("--min-target-point-clearance", type=float, default=0.04)
    ap.add_argument("--no-contact-lock-generation", action="store_true")
    ap.add_argument("--summary-only", action="store_true")
    ap.add_argument("--output", default="")
    args = ap.parse_args()

    cfg = V3RollGenerationConfig(
        steps_per_phase=args.steps_per_phase,
        lift_height=args.lift_height,
        clearance_height=args.clearance_height,
        candidate_support_shift_x=args.candidate_support_shift_x,
        candidate_support_drop_z=args.candidate_support_drop_z,
        body_roll_pitch_rad=__import__("math").radians(args.body_roll_pitch_deg),
        body_roll_x_shift=args.body_roll_x_shift,
        body_roll_z_shift=args.body_roll_z_shift,
        enable_body_roll_pose_search=not args.disable_body_roll_pose_search,
        body_roll_search_x_offsets=_parse_float_list(args.body_roll_search_x_offsets),
        body_roll_search_z_offsets=_parse_float_list(args.body_roll_search_z_offsets),
        ground_z=args.ground_z,
        auto_align_initial_ground=not args.no_auto_align_initial_ground,
        min_inter_leg_clearance_m=args.min_inter_leg_clearance,
        min_target_point_clearance_m=args.min_target_point_clearance,
        enable_contact_lock_generation=not args.no_contact_lock_generation,
        contact_plan_variant=args.contact_plan_variant,
    )
    gen = V3RollCandidateGenerator(config=cfg)
    cand = gen.generate_forward_one_roll(surface_id=args.surface_id)
    data = cand.to_dict()
    if args.summary_only:
        data = {
            "direction": data["direction"],
            "phase_count": data["phase_count"],
            "frame_count": data["frame_count"],
            "phase_names": [p["name"] for p in data["phases"]],
            "report": data["report"],
        }
    text = json.dumps(data, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w") as f:
            f.write(text)
            f.write("\n")
    print(text)


if __name__ == "__main__":
    main()
