# v3.0.20 Legacy Reference Trajectory Importer

## Purpose

v3.0.20 adds a safer route for reproducing the old rolling motion: import the actual joint command trajectory emitted by the legacy program instead of guessing the motion from videos or hand-written RF scaffolds.

The importer does **not** call the old project, old `LilyRobot`, old IK, `legacy-src-path`, or xacro. It only reads a log file and converts the command sequence into the v3-core `WholeRollCandidate` schema.

## Supported inputs

Recommended JSONL record:

```json
{"frame_index":0,"phase_name":"RF-1","joint_command_rad":[0.0, ... 24 values ...]}
```

Also accepted:

- `joint_command_deg`: 24 values in existing Gazebo/JointState order
- `positions`, `position`, `command`, or `joint_command`: 24 values, unit selected by `--input-unit`
- `joint_angles`: `{ "TRF": [q0,q1,q2], ... }` or `{ "0": [q0,q1,q2], ... }`
- CSV with `joint_command_0` ... `joint_command_23`, `q0` ... `q23`, or `joint_0` ... `joint_23`

If base pose columns are present (`base_x`, `base_y`, `base_z`, `base_roll`, `base_pitch`, `base_yaw`) they are used. If not, v3 uses a constant auto-aligned base pose. In that case, geometry evaluation is approximate, but Gazebo replay still uses the imported joint commands exactly.

## Import and evaluate

```bash
python tools/command_generation/run_v3_0_import_legacy_reference.py \
  --input testdata/legacy_published_commands.jsonl \
  --input-format auto \
  --input-unit rad \
  --filter-window 3 \
  --candidate-output testdata/legacy_reference_candidate.json \
  --command-output testdata/legacy_reference_commands.jsonl \
  --command-source raw
```

Use `--command-source raw` to replay the exact imported command sequence. Use `--command-source filtered` only when you deliberately want v3 to apply a moving-average filter.

## Export later from the imported candidate

```bash
python tools/command_generation/run_v3_0_export_commands.py \
  --profile imported_reference \
  --candidate testdata/legacy_reference_candidate.json \
  --command-source raw \
  --output testdata/legacy_reference_commands.jsonl
```

## Gazebo preview

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 20 \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/legacy_reference_commands.jsonl \
  --gazebo-link-state-log testdata/legacy_reference_gazebo_link_states.jsonl \
  --verbose-publish
```

## Why this matters

The v3.0.19 RF scaffold was not a faithful reproduction of the legacy motion. v3.0.20 changes the workflow: first import the actual legacy trajectory, then use v3-core to evaluate, visualize, and replay it. This prevents us from debugging a guessed motion instead of the real one.
