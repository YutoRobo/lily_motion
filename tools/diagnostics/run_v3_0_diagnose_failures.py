#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Generate a v3 candidate and print the failure-diagnosis report."""
from __future__ import print_function
import argparse
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from run_v3_0_whole_roll_eval import build_generation_config
from lily_motion_v3.v3_roll_candidate_generator import V3RollCandidateGenerator
from lily_motion_v3.synchronized_roll_generator import SynchronizedRollCandidateGenerator
from lily_motion_v3.whole_roll_evaluator import WholeRollEvaluator, WholeRollEvaluationConfig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--surface-id", type=int, default=1)
    ap.add_argument("--trajectory-mode", choices=["phase", "synchronized"], default="phase")
    ap.add_argument("--synchronized-steps", type=int, default=72)
    ap.add_argument("--roll-start-s", type=float, default=0.35)
    ap.add_argument("--roll-end-s", type=float, default=0.85)
    ap.add_argument("--contact-plan-variant", default="front_pair_roll")
    ap.add_argument("--steps-per-phase", type=int, default=6)
    ap.add_argument("--lift-height", type=float, default=0.12)
    ap.add_argument("--clearance-height", type=float, default=0.06)
    ap.add_argument("--candidate-support-shift-x", type=float, default=0.04)
    ap.add_argument("--candidate-support-drop-z", type=float, default=-0.02)
    ap.add_argument("--body-roll-pitch-deg", type=float, default=60.0)
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
    ap.add_argument("--filter-window", type=int, default=3)
    ap.add_argument("--contact-preserving-filter", action="store_true")
    ap.add_argument("--contact-drift-soft-limit", type=float, default=0.05)
    ap.add_argument("--contact-drift-hard-limit", type=float, default=0.15)
    ap.add_argument("--output", default="")
    args = ap.parse_args()

    gen_cls = SynchronizedRollCandidateGenerator if args.trajectory_mode == "synchronized" else V3RollCandidateGenerator
    gen = gen_cls(config=build_generation_config(args))
    cand = gen.generate_forward_one_roll(surface_id=args.surface_id)
    whole = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(
        filter_window=args.filter_window,
        ground_z=args.ground_z,
        min_inter_leg_clearance_m=args.min_inter_leg_clearance,
        contact_drift_soft_limit_m=args.contact_drift_soft_limit,
        contact_drift_hard_limit_m=args.contact_drift_hard_limit,
        second_joint_abs_max_deg=gen.robot_model.leg_config.second_joint_abs_max_deg,
        contact_preserving_filter=args.contact_preserving_filter,
    )).evaluate(cand)
    data = whole["failure_diagnosis"]
    text = json.dumps(data, indent=2, sort_keys=True)
    if args.output:
        d = os.path.dirname(args.output)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(args.output, "w") as f:
            f.write(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
