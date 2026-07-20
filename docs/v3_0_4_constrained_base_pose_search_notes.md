# v3.0.4 constrained base-pose search notes

## Purpose

v3.0.3 introduced an explicit base pitch trajectory, but the body roll still used a fixed base pose trajectory.  That made the generator closer to a diagnostic than a constrained motion generator: it could reveal IK failures and ground penetration, but it did not try to choose a better base translation.

v3.0.4 adds a small base-pose candidate search for `ConstrainedBodyRoll`.

## What changed

During each `ConstrainedBodyRoll` frame, the generator now:

1. keeps the requested pitch for that frame,
2. samples multiple `base_x` / `base_z` offsets around the nominal pose,
3. converts fixed world foot targets to body-frame targets for each candidate pose,
4. evaluates IK reachability, second-joint limit, ground clearance, inter-leg clearance, and joint discontinuity,
5. selects the least-violating base pose.

This is still not a final constrained optimizer.  It is a deterministic, project-contained candidate selector intended to show whether base translation can reduce the dominant failures.

## New CLI options

```bash
python archive/v3_experiment_scripts/run_v3_0_concept_roll.py --summary-only
```

Optional controls:

```bash
--disable-body-roll-pose-search
--body-roll-search-x-offsets '-0.20,-0.10,0.0,0.10,0.20'
--body-roll-search-z-offsets '-0.10,0.0,0.10,0.20,0.30,0.40'
```

Example with a wider search:

```bash
python archive/v3_experiment_scripts/run_v3_0_concept_roll.py \
  --summary-only \
  --body-roll-search-x-offsets '-0.30,-0.20,-0.10,0.0,0.10,0.20,0.30' \
  --body-roll-search-z-offsets '0.0,0.10,0.20,0.30,0.40,0.50,0.60'
```

## New report field

`MotionEvaluationReport` now includes:

```text
base_pose_search:
  enabled
  failure_count
  top_records
  top_failure_records
```

`task_success` also includes:

```text
base_pose_search_enabled
base_pose_search_failure_count
```

## Interpretation

A lower penetration count or lower IK failure count after enabling search means that base translation is helping.  If failures remain, the result should not be interpreted as a bug in the search itself.  It means the current contact-state transition and fixed foot targets are still geometrically too strict for a 90 deg roll.

In that case, the next design step is not just more search range.  The support set, support foot targets, and contact transfer policy need to be redesigned.

## Limitations

- The pitch trajectory is still prescribed.
- Search is grid-based and local.
- Support feet are not yet optimized as contact targets.
- This does not yet publish to Gazebo.
- Collision checking remains representative-segment based, not exact Gazebo collision geometry.
