# v3.0.19 Legacy RF-style reproduction scaffold

## Purpose

v3.0.19 moves the legacy-style adapter from a generic role scaffold toward a
legacy RF-style representation inside v3-core.  It still does **not** import or
call the old project, old `LilyRobot`, old IK, `legacy-src-path`, or xacro.

The goal is to represent the old qualitative motion in the common v3 schema:

1. RF-1: six-contact starting condition
2. RF-2: next-surface pre-shape using `rf2_pitch_scale` and `rf2_x_scale`
3. RF-3: lift/step the middle transition pair while six-contact context is preserved
4. RF-4: body roll through the singular/flip-prone region
5. RF-5: support transfer
6. RF-6: posture normalization for the next roll

This is still a scaffold, not a byte-for-byte reproduction of the old RF code.
The important improvement is that the v3-core report, visualizer, and Gazebo
export now see RF-named phases, making the behavior easier to compare with the
old algorithm.

## Evaluate

```bash
python run_v3_0_legacy_style_eval.py \
  --summary-only \
  --step-scale 1.5 \
  --splited-num 10 \
  --rf2-pitch-scale 1.0 \
  --rf2-x-scale 1.0 \
  --filter-window 3
```

## Export for Gazebo

```bash
python run_v3_0_export_commands.py \
  --profile legacy_style \
  --step-scale 1.5 \
  --splited-num 10 \
  --rf2-pitch-scale 1.0 \
  --rf2-x-scale 1.0 \
  --filter-window 3 \
  --command-source filtered \
  --output testdata/v3_0_19_legacy_rf_filtered_commands.jsonl
```

Then dry-run:

```bash
python run_v3_0_gazebo_replay.py \
  --dry-run \
  --command-log testdata/v3_0_19_legacy_rf_filtered_commands.jsonl \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0
```

Gazebo replay:

```bash
python run_v3_0_gazebo_replay.py \
  --rate 20 \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/v3_0_19_legacy_rf_filtered_commands.jsonl \
  --gazebo-link-state-log testdata/v3_0_19_legacy_rf_gazebo_link_states.jsonl \
  --verbose-publish
```

## Interpretation

This version is meant for comparison and debugging.  If the Gazebo motion does
not yet match the old program, that means the RF profile still needs fidelity
improvement, not that v3-core is invalid.
