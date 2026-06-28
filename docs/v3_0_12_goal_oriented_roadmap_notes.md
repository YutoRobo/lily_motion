# v3.0.12 Goal-oriented roadmap and staged search

## Purpose

v3.0.12 does not replace the current v3 generator. It adds a roadmap-controlled
exploration layer so that development does not drift away from the real goal:

- generate a project-contained one-roll motion,
- allow singular-pose / flip-like raw commands when necessary,
- smooth raw commands with a filter,
- tolerate small filtered contact drift,
- keep ground penetration, inter-leg interference, and the second-joint limit within bounds,
- later compare and import legacy-style roll parameters.

## Why this was added

The current v3 generator is not yet a successful gait. However, the evaluation
axis is now close to the intended design philosophy. The remaining work should
be managed as staged search and model improvement rather than ad-hoc local fixes.

## Roadmap

### Stage 1: v3-native whole-roll search

Search v3-native contact plans and generator parameters using the full filtered
trajectory evaluation. Raw 180 deg-like flips are not immediately rejected; the
filtered trajectory is the main geometry candidate.

Primary metrics:

- generator_ik_failure_count
- filtered_penetration_count
- filtered_near_count
- filtered_max_second_joint_deg
- filtered_max_joint_delta_deg
- filtered_max_contact_drift_m
- filtered_contact_drift_hard_violation_count

### Stage 2: Contact plan redesign

If no Stage 1 case reduces IK failures enough, redesign the support set / lift
set / candidate support set. This is likely necessary because current variants
are still rough approximations.

### Stage 3: Legacy-style profile compatibility

Add a compatibility layer that maps legacy RF-style parameters into v3 internal
frames. This remains on the roadmap, but it is not the immediate next task unless
v3-native search stalls.

Target future interface:

```bash
python run_v3_0_whole_roll_eval.py --profile legacy_style \
  --step-scale 1.5 \
  --splited-num 10 \
  --rf2-pitch-scale 1.0 \
  --rf2-x-scale 1.0
```

### Stage 4: Gazebo replay

Gazebo replay should be used when the filtered geometry has at least a plausible
partial trajectory. Until then, Gazebo is a failure-visualization tool, not the
main design loop.

## v3.0.12 addition

`run_v3_0_goal_oriented_sweep.py` runs a staged search with explicit roadmap
metadata and richer failure ranking. It can sweep body-roll pitch angles as well
as the existing contact/gait/filter parameters.
