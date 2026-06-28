# v3.0.18 Gazebo export profiles

## Purpose

v3.0.18 makes Gazebo preview a first-class check for the current state of the project.
The exporter can now emit command JSONL for both:

- `--profile native`: v3-native candidates (`front_pair_roll`, `diagonal_front_roll`, etc.)
- `--profile legacy_style`: v3-core legacy-style scaffold, still independent of old LilyRobot/legacy IK/xacro

It also supports:

- `--command-source raw`
- `--command-source filtered`

The recommended default for visual checking is `--command-source filtered`, because the current design explicitly allows raw flip-like changes and evaluates the smoothed command as the actuator-side candidate.

## Important limitations

The legacy-style profile is not yet an exact numerical reproduction of the old RF implementation. It is a v3-core scaffold that exposes legacy-like parameters (`step_scale`, `splited_num`, `rf2_pitch_scale`, `rf2_x_scale`) through the common candidate/evaluator/exporter interface.

Current Gazebo preview is still a **visual diagnosis step**, not a success demo. The candidate may stop at the first invalid frame unless `--include-invalid-frame` or `--allow-invalid-frames` is specified.

## Native Gazebo preview

Export:

```bash
python run_v3_0_export_commands.py \
  --profile native \
  --contact-plan-variant front_pair_roll \
  --steps-per-phase 6 \
  --lift-height 0.12 \
  --clearance-height 0.06 \
  --candidate-support-drop-z=-0.02 \
  --filter-window 3 \
  --body-roll-pitch-deg 60 \
  --command-source filtered \
  --output testdata/v3_0_18_native_filtered_commands.jsonl
```

Dry-run replay:

```bash
python run_v3_0_gazebo_replay.py \
  --dry-run \
  --command-log testdata/v3_0_18_native_filtered_commands.jsonl \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0
```

Gazebo replay after starting Gazebo:

```bash
python run_v3_0_gazebo_replay.py \
  --rate 20 \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/v3_0_18_native_filtered_commands.jsonl \
  --gazebo-link-state-log testdata/v3_0_18_native_gazebo_link_states.jsonl \
  --verbose-publish
```

## Legacy-style Gazebo preview

Export:

```bash
python run_v3_0_export_commands.py \
  --profile legacy_style \
  --step-scale 1.5 \
  --splited-num 10 \
  --rf2-pitch-scale 1.0 \
  --rf2-x-scale 1.0 \
  --filter-window 3 \
  --command-source filtered \
  --output testdata/v3_0_18_legacy_style_filtered_commands.jsonl
```

Dry-run replay:

```bash
python run_v3_0_gazebo_replay.py \
  --dry-run \
  --command-log testdata/v3_0_18_legacy_style_filtered_commands.jsonl \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0
```

Gazebo replay after starting Gazebo:

```bash
python run_v3_0_gazebo_replay.py \
  --rate 20 \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/v3_0_18_legacy_style_filtered_commands.jsonl \
  --gazebo-link-state-log testdata/v3_0_18_legacy_style_gazebo_link_states.jsonl \
  --verbose-publish
```

## Suggested visual checks

1. Confirm the body visibly follows the expected roll direction.
2. Confirm whether the preview stops before the intended roll because of invalid frames.
3. Compare raw vs filtered only after confirming filtered commands move the model.
4. Compare native vs legacy_style as different generator families, not as equivalent motions yet.
5. When the preview looks too short, inspect `exported_command_count`; it may stop at the first invalid frame by design.
