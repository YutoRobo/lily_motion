# v3.0.10 Contact-Preserving Filter Notes

## Purpose

v3.0.9 showed the key split clearly:

- Raw trajectory with `front_pair_roll` can preserve SUPPORT foot contact nearly exactly.
- Naive moving-average filtering reduces raw flip-like joint jumps, but it also moves planted feet.

v3.0.10 adds an optional filtered-command reprojection step so that smoothing no longer silently violates the contact-lock assumption.

## Added behavior

New filter mode:

```text
moving_average_unwrapped_angles_contact_reproject
```

Pipeline:

```text
raw joint trajectory
  -> moving average on unwrapped joint angles
  -> for each SUPPORT leg, re-solve IK to the locked world contact point
  -> evaluate filtered geometry and contact drift
```

This is not yet the final actuator command filter. It is a diagnostic / bridge filter that separates these two effects:

1. command smoothing effect
2. planted-foot constraint preservation

## Command

```bash
python run_v3_0_whole_roll_eval.py \
  --summary-only \
  --contact-plan-variant front_pair_roll \
  --steps-per-phase 6 \
  --contact-preserving-filter
```

## Observed quick check

For `front_pair_roll`, `steps_per_phase=6`:

Without contact-preserving filter:

```text
filtered_contact_drift_violation_count = 15
filtered_max_contact_drift_m           = 0.1635 m
filtered_max_joint_delta_deg           = 36.0 deg
filtered_penetration_count             = 2
```

With contact-preserving filter:

```text
filtered_contact_drift_violation_count = 0
filtered_max_contact_drift_m           ~ 0
filter_projected_count                 = 108
filter_projection_failure_count         = 12
filtered_max_joint_delta_deg           = 180.0 deg
filtered_penetration_count             = 2
```

## Interpretation

The new filter proves that the contact-lock drift after moving average was not unavoidable. It can be eliminated by reprojecting SUPPORT legs back to their locked foot positions.

However, this also reveals the next problem:

```text
When contact lock is enforced after filtering,
large IK branch changes can reappear in the filtered trajectory.
```

Therefore, the next step is not to return to naive moving average. The next step is to make the reprojection branch-aware and smoothness-aware.

## Next direction

v3.0.11 should improve the filtered-contact reprojection by adding:

```text
1. branch continuity cost during reprojection
2. max allowed reprojected joint delta
3. optional partial reprojection / transition window
4. separate smoothing policies for SUPPORT and non-SUPPORT legs
```

The core lesson is:

```text
A pure moving average is not physically valid for planted legs.
A pure contact projection preserves feet but may reintroduce branch jumps.
The final filter must combine both constraints.
```
