# v3.0.32 segmented roll-boundary filtering

## Purpose

The current good baseline is the pure legacy repeated roll motion:

```text
surface_sequence = 1,5,6,2,1
move_dist        = 0.4
support_dist     = 0.7
legacy_body_z    = 0.35
max_step         = 30 or higher
profile          = pure_legacy_repeated_roll
```

This motion is close to the original legacy behavior.  The remaining visible issue is a steep singular-posture avoidance motion around the quarter-roll boundary, especially from the end of the first quarter roll to the start of the second.

## Important design decision

Surface/quarter-roll switching is not part of the same local moving-average neighborhood.  A moving average across a surface switch can blend two different roll-coordinate contexts and can make the intended singular-posture avoidance look worse or physically ambiguous.

Therefore v3.0.32 adds segmented resampling/smoothing.  Use:

```bash
--segment-key roll_index
```

when resampling or replaying repeated-roll command logs.

This resets interpolation and moving-average windows at each `roll_index` boundary.

## Recommended generation

```bash
python archive/v3_experiment_scripts/run_v3_0_pure_legacy_repeated_roll.py \
  --surface-sequence 1,5,6,2,1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --legacy-body-z 0.35 \
  --max-step 30 \
  --skip-constraints \
  --output-command-log testdata/v3_0_32_good_pure_legacy_m30_commands.jsonl \
  --report-output testdata/v3_0_32_good_pure_legacy_m30_report.json
```

## Recommended Gazebo-preview smoothing

```bash
python tools/command_generation/run_v3_0_resample_commands.py \
  --input testdata/v3_0_32_good_pure_legacy_m30_commands.jsonl \
  --resample-factor 8 \
  --smooth-window 3 \
  --segment-key roll_index \
  --output testdata/v3_0_32_good_pure_legacy_m30_segmented_x8_sw3.jsonl
```

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 10 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 3.0 \
  --hold-end-sec 5.0 \
  --command-log testdata/v3_0_32_good_pure_legacy_m30_segmented_x8_sw3.jsonl \
  --verbose-publish
```

Alternatively, replay can apply segmentation directly:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 10 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 3.0 \
  --hold-end-sec 5.0 \
  --command-log testdata/v3_0_32_good_pure_legacy_m30_commands.jsonl \
  --resample-factor 8 \
  --smooth-window 3 \
  --segment-key roll_index \
  --verbose-publish
```

## What changed

- `lily_motion_v3/command_resampler.py`
  - `resample_command_records(..., segment_key=None)`
  - `moving_average_command_records(..., segment_key=None)`
- `tools/command_generation/run_v3_0_resample_commands.py`
  - added `--segment-key`
- `tools/gazebo/run_v3_0_gazebo_replay.py`
  - added `--segment-key` for replay-time resampling/smoothing

## What did not change

The gait state machine is unchanged.  This version only changes the preview/export filtering behavior around quarter-roll boundaries.
