# v3.0.8 Whole-Roll Filtered Contact-Lock Evaluation

## Purpose

v3.0.8 changes the evaluation axis from single-frame feasibility to whole-roll feasibility.

The key assumptions are:

- Raw joint commands may contain large flip-like changes near singular postures.
- Raw discontinuity is not automatically a failure.
- The command actually applied to the robot should be judged after filtering.
- Once a support foot contacts the ground, its world contact point should remain locked until the foot leaves support.
- One roll must be judged as a full trajectory, not as independent frame-wise poses.

## Added modules

```text
lily_motion_v3/command_filter.py
lily_motion_v3/contact_lock.py
lily_motion_v3/whole_roll_evaluator.py
tools/diagnostics/run_v3_0_whole_roll_eval.py
tools/diagnostics/run_v3_0_parameter_sweep.py
```

## Main command

```bash
python tools/diagnostics/run_v3_0_whole_roll_eval.py --summary-only
```

Example output fields:

```text
candidate_completed
whole_roll_success_by_filtered_geometry
raw_max_joint_delta_deg
filtered_max_joint_delta_deg
filtered_penetration_count
filtered_min_clearance_m
filtered_near_count
filtered_max_second_joint_deg
contact_drift_violation_count
max_contact_drift_m
generator_ik_failure_count
```

## Parameter sweep

```bash
python tools/diagnostics/run_v3_0_parameter_sweep.py \
  --steps-per-phase 6,8 \
  --lift-heights 0.06,0.08,0.10 \
  --clearance-heights 0.05,0.08 \
  --candidate-support-shift-xs 0.02,0.04,0.06 \
  --candidate-support-drop-zs=-0.04,-0.02,0.0 \
  --output testdata/v3_0_8_parameter_sweep_summary.json
```

## Current interpretation

The present v3 generator is still not a valid full roll gait.  The evaluator makes this explicit:

- Raw 180 deg jumps can be reduced by the moving-average filter.
- Filtered geometry can still penetrate the floor.
- Contact-lock drift can become very large, which means the filtered command violates the assumption that a support foot stays fixed on the floor.
- Generator IK failures still show that the current contact state and target placement are not sufficient.

This version is therefore not a final gait generator.  It is the correct evaluation basis for the next generator revision.
