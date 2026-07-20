# v3.0.16 Visualization Notes

## Purpose

v3.0.16 adds a ROS/Gazebo-independent visual inspection path for v3-core roll candidates.
The aim is to make the whole-body behavior visible before relying on Gazebo replay.

This is not a physics simulation. It renders the self-contained v3 `RobotModel` using FK:

- base wireframe
- leg representative segments
- foot points
- active SUPPORT contact lock points
- key frames around phase transitions and the first dominant failure

## Why this was added

The numerical evaluation is useful, but it does not show whether the robot motion is globally meaningful.
The visualizer helps inspect whether the candidate is actually rolling, which legs are moving, and where the first failure happens.

## Command

```bash
python tools/diagnostics/run_v3_0_visualize_roll.py \
  --contact-plan-variant front_pair_roll \
  --steps-per-phase 6 \
  --lift-height 0.12 \
  --clearance-height 0.06 \
  --candidate-support-drop-z=-0.02 \
  --filter-window 3 \
  --body-roll-pitch-deg 60 \
  --command-source filtered \
  --output-dir testdata/v3_0_16_visualization
```

Open:

```bash
testdata/v3_0_16_visualization/index.html
```

## Raw vs filtered

Use `--command-source raw` to inspect the raw trajectory before the moving-average filter.
Use `--command-source filtered` to inspect the command sequence being evaluated after filtering.

## Interpretation limits

The plot is kinematic only:

- no Gazebo collision geometry
- no contact force
- no dynamics
- no actuator response

It is meant to catch large-scale mistakes before Gazebo replay.
