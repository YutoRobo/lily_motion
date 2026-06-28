# v3.0.5 Base Pose Handoff Notes

## Purpose

v3.0.4 introduced base-pose search during `ConstrainedBodyRoll`, but the following phases recomputed their base pose from the nominal roll progress.  This could reset `base_x/base_z` at `SupportTransfer` and `PostureNormalization`, breaking the consistency of the pose selected during the roll phase.

v3.0.5 fixes this by handing off the selected terminal `ConstrainedBodyRoll` base pose to the subsequent phases.

## What changed

- `ConstrainedBodyRoll` stores the latest selected `base_pose` as `_post_roll_base_pose`.
- `SupportTransfer` and `PostureNormalization` use this stored pose instead of recomputing nominal progress=1.0.
- The change is intentionally small: it does not yet redesign support legs or contact targets.

## Why this matters

This prevents an artificial discontinuity where body roll finishes with a searched pose such as `z=0.73`, but the next phase falls back to the nominal `z=0.33`.  That reset caused ground penetration and inconsistent IK results in the previous report.

## Remaining limitations

Even with pose handoff, the current one-roll concept can still fail if the selected support set and foot targets are not geometrically compatible with a 90 degree roll.  The next major step is support/contact redesign, not merely widening the base-pose search grid.
