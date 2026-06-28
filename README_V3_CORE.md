# Lily Motion v3-core independent package

This package contains the project-contained v3 rolling-gait core.  It is not the
old legacy-connected evaluator.  The v3 evaluation pipeline is intended to run
without `legacy-src-path`, legacy `LilyRobot`, hidden legacy IK, or xacro loading.

## Quick evaluation

```bash
python run_v3_0_whole_roll_eval.py --summary-only
```

A useful current candidate example:

```bash
python run_v3_0_whole_roll_eval.py \
  --summary-only \
  --contact-plan-variant front_pair_roll \
  --steps-per-phase 6 \
  --lift-height 0.12 \
  --clearance-height 0.06 \
  --candidate-support-drop-z=-0.02 \
  --filter-window 3 \
  --body-roll-pitch-deg 60
```

## Failure diagnosis

```bash
python run_v3_0_diagnose_failures.py
```

## Sweep

```bash
python run_v3_0_parameter_sweep.py \
  --trajectory-modes phase,synchronized \
  --synchronized-steps 48,72 \
  --contact-plan-variants front_pair_roll,diagonal_front_roll \
  --steps-per-phase 6 \
  --lift-heights 0.08,0.12,0.16 \
  --clearance-heights 0.06,0.10,0.14 \
  --candidate-support-shift-xs 0.04 \
  --candidate-support-drop-zs=-0.02,0.0,0.02 \
  --filter-windows 3,5 \
  --body-roll-pitch-deg 60 \
  --output testdata/v3_0_15_sweep.json
```

## Optional Gazebo dry run

```bash
python run_v3_0_gazebo_replay.py --dry-run --command-log testdata/v3_0_15_commands.jsonl
```

Live Gazebo replay requires ROS/Gazebo, but not the older `lily_motion` package.

## v3.0.16: Standalone visualization

Use the visualizer before Gazebo when you need to inspect the overall kinematic motion.

```bash
python run_v3_0_visualize_roll.py \
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

Then open `testdata/v3_0_16_visualization/index.html`.

## v3.0.17 legacy-style adapter

A legacy-style adapter is included as `run_v3_0_legacy_style_eval.py` and
`lily_motion_v3/legacy_style_generator.py`.  It maps legacy-like parameters
(`step_scale`, `splited_num`, `rf2_pitch_scale`, `rf2_x_scale`) into the common
v3 `WholeRollCandidate` / `WholeRollEvaluator` path without importing the old
project.

This is a scaffold for comparison, not an exact RF-1..RF-6 reproduction.

## v3.0.18 Gazebo preview profiles

`run_v3_0_export_commands.py` now supports both v3-native and legacy-style candidates:

```bash
python run_v3_0_export_commands.py --profile native --command-source filtered --output testdata/native.jsonl
python run_v3_0_export_commands.py --profile legacy_style --command-source filtered --output testdata/legacy_style.jsonl
```

Replay is always a separate step:

```bash
python run_v3_0_gazebo_replay.py --dry-run --command-log testdata/native.jsonl
python run_v3_0_gazebo_replay.py --rate 20 --frame-hold-sec 0.25 --command-log testdata/native.jsonl
```

Gazebo preview is currently diagnostic.  It shows what the current candidate does up to the first invalid frame unless `--include-invalid-frame` or `--allow-invalid-frames` is used.

## v3.0.19 legacy RF-style adapter

`legacy_style` now uses RF-named phases (`RF-1_StableSixContact` ..
`RF-6_PostureNormalization`) so the v3-core reports and Gazebo previews can be
compared more directly with the old six-contact / middle-pair step-over roll
idea.  This remains project-contained and does not call old LilyRobot/IK/xacro.


## v3.0.20: Imported legacy/reference trajectories

Use `run_v3_0_import_legacy_reference.py` when you have an actual legacy joint command log. This is now the preferred route for reproducing the real legacy roll motion, because it avoids guessing the old motion from videos.

Example:

```bash
python run_v3_0_import_legacy_reference.py \
  --input testdata/legacy_published_commands.jsonl \
  --input-format auto \
  --input-unit rad \
  --filter-window 3 \
  --candidate-output testdata/legacy_reference_candidate.json \
  --command-output testdata/legacy_reference_commands.jsonl \
  --command-source raw

python run_v3_0_gazebo_replay.py \
  --rate 20 \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/legacy_reference_commands.jsonl \
  --gazebo-link-state-log testdata/legacy_reference_gazebo_link_states.jsonl \
  --verbose-publish
```
