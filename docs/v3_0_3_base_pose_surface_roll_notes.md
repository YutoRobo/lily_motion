# v3.0.3 Base Pose / Surface Roll Notes

## Purpose

v3.0.2 could run without legacy dependencies, but the summary was too optimistic because the body pose did not meaningfully rotate.  v3.0.3 adds a frame-wise `base_pose` trajectory and converts fixed world foot targets into body-frame IK targets during `ConstrainedBodyRoll`.

This is still a concept generator, not a finalized gait.

## Main changes

- Added base/world transform utilities.
- Added `RobotModel.body_point_to_world()` and `RobotModel.world_point_to_body()`.
- Added world-space link segment evaluation.
- Added `foot_targets_world` to each `V3MotionFrame`.
- Added `base_pose_enabled`, `surface_after`, `max_abs_base_pitch_deg`, and `initial_base_z` to the task report.
- Added coarse ground clearance reporting.
- `ConstrainedBodyRoll` now changes pitch by `body_roll_pitch_rad` while preserving world foot targets.

## Run

```bash
python archive/v3_experiment_scripts/run_v3_0_concept_roll.py --summary-only
```

Save full output:

```bash
python archive/v3_experiment_scripts/run_v3_0_concept_roll.py \
  --steps-per-phase 8 \
  --output testdata/v3_0_3_concept_roll_forward.json
```

Try a smaller roll angle for diagnosis:

```bash
python archive/v3_experiment_scripts/run_v3_0_concept_roll.py \
  --summary-only \
  --body-roll-pitch-deg 45
```

## Interpretation

After base-pose rotation is enabled, the v3 generator is expected to expose harder failures than v3.0.2.  That is intentional.  If the report shows IK failures or ground penetration, it means the current role/target design is not yet a physically valid roll.

Important fields:

- `report.task_success.max_abs_base_pitch_deg`
- `report.task_success.surface_after`
- `report.ik_reachability.ik_failure_count`
- `report.ground_clearance.min_clearance_m`
- `report.joint_limit.max_abs_second_joint_deg`
- `frames[].base_pose`
- `frames[].foot_targets_world`

## Known limitation

The ground and inter-leg checks still use representative points and line segments, not exact Gazebo collision geometry.  The purpose is to expose gross geometric failures before connecting Gazebo or optimization.
