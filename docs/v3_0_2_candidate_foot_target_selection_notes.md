# v3.0.2 Candidate Foot Target Selection Notes

## Purpose

v3.0.2 moves the v3 roll generator one step beyond fixed foot targets.
The generator now creates multiple candidate foot targets for role-dependent legs and selects a feasible target using project-contained kinematics and local constraints.

This version still does **not** depend on the legacy LilyRobot or legacy IK.

## Added files

- `lily_motion_v3/geometry.py`
- `lily_motion_v3/foot_target_candidate.py`

Updated files:

- `lily_motion_v3/leg_kinematics.py`
- `lily_motion_v3/robot_model.py`
- `lily_motion_v3/v3_roll_candidate_generator.py`
- `archive/v3_experiment_scripts/run_v3_0_concept_roll.py`
- `tests/test_v3_0_kinematics.py`

## Main additions

### 1. Link-point and segment primitives

`LegKinematics.link_positions()` returns representative points:

- mount
- coxa_end
- knee
- foot

`RobotModel.leg_segments_body()` converts these into body-frame segments:

- mount_to_coxa
- coxa_to_knee
- knee_to_foot

These are not exact collision geometries. They are portable primitives for early v3 planning and evaluation.

### 2. Candidate foot target generation

`CandidateFootTargetGenerator` generates role-dependent candidates:

- `LIFT`: upward lift candidates with small x/y offsets
- `CLEARANCE`: upward/side clearance candidates
- `CANDIDATE_SUPPORT`: shifted landing/support candidates
- other roles: current target is kept

Each candidate is evaluated by:

- IK feasibility
- second-joint limit through the v3 IK selector
- minimum point distance to other current foot targets
- continuity from previous joint angles
- second-joint margin

### 3. Candidate selection reporting

`MotionEvaluationReport.support_consistency` now includes:

- `target_selection_failure_count`
- `max_candidate_count_per_leg`
- `top_selection_failures`
- `top_selection_records`

This is intentionally reported as support/role consistency rather than as a narrow RF-phase-specific diagnostic.

### 4. Project-contained inter-leg segment clearance

The v3 roll generator now computes a simple segment-to-segment inter-leg clearance for every generated frame.

`MotionEvaluationReport.inter_leg_clearance` includes:

- `threshold_m`
- `min_distance_m`
- `near_count`
- `top_near_records`

This remains a first-stage geometric screening, not exact Gazebo collision geometry.

## Example command

```bash
python archive/v3_experiment_scripts/run_v3_0_concept_roll.py --summary-only
```

With custom thresholds:

```bash
python archive/v3_experiment_scripts/run_v3_0_concept_roll.py \
  --steps-per-phase 8 \
  --min-inter-leg-clearance 0.05 \
  --min-target-point-clearance 0.04 \
  --output testdata/v3_0_2_concept_roll_forward.json
```

## Current status

This is still a concept generator, not a finalized roll gait.

The important progress is architectural:

1. v3 remains project-contained.
2. Foot targets are no longer single fixed points.
3. IK candidate selection is explicit.
4. Constraint-related diagnostics are reported in one motion report.
5. The design is reusable for future contact-state and gait redesign work.

## Test result

```text
Ran 44 tests
OK
```
