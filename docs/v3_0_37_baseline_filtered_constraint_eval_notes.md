# v3.0.37 baseline filtered constraint evaluation

This version fixes the current baseline for Gazebo preview:

- `pure_legacy_repeated_roll`
- `surface_sequence=1,5,6,2,1`
- `move_dist=0.4`
- `support_dist=0.7`
- `legacy_body_z=0.35`
- `max_step>=30`
- RF-1 current-angle anchor enabled
- resample factor 8
- moving average `smooth_window=40`
- no `segment_key` by default, so the moving average is applied across the whole 4-roll command stream.

The goal is not to create a new gait.  The goal is to evaluate the exact command stream that will be sent to Gazebo.

## Basic command

```bash
python run_v3_0_baseline_filtered_constraint_eval.py \
  --surface-sequence 1,5,6,2,1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --legacy-body-z 0.35 \
  --max-step 30 \
  --resample-factor 8 \
  --smooth-window 40 \
  --constraint-stride 8 \
  --output-raw-command-log testdata/v3_0_37_baseline_raw_commands.jsonl \
  --output-filtered-command-log testdata/v3_0_37_baseline_x8_sw40_commands.jsonl \
  --report-output testdata/v3_0_37_baseline_filtered_constraint_report.json
```

## Gazebo replay

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --strict-command-log-input \
  --rate 80 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 3.0 \
  --hold-end-sec 5.0 \
  --command-log testdata/v3_0_37_baseline_x8_sw40_commands.jsonl
```

## What to inspect

The report contains both `raw` and `filtered` sections:

- `max_adjacent_delta_deg`
- roll-boundary jumps
- max second joint angle
- second joint violation count
- ground penetration count
- inter-leg near count

The filtered section is the important one for Gazebo because it corresponds to the actual replay command.
