# v3.0.34 continuous angle unwrap for Gazebo preview

This version does **not** change the pure legacy repeated-roll state machine.
It addresses a replay-side issue: legacy IK may output angle-equivalent values
that differ by approximately 2π.  Gazebo joint controllers see those values as
large command jumps.  `--unwrap-continuous-angles` rewrites each joint command to
the 2π-equivalent value closest to the previous frame.

This is not moving-average across a surface boundary and not boundary preblend.
It preserves the represented posture while avoiding avoidable representation
jumps.

Recommended baseline:

```bash
python archive/v3_experiment_scripts/run_v3_0_pure_legacy_repeated_roll.py \
  --surface-sequence 1,5,6,2,1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --legacy-body-z 0.35 \
  --max-step 30 \
  --skip-constraints \
  --output-command-log testdata/v3_0_34_pure_legacy_m30_commands.jsonl \
  --report-output testdata/v3_0_34_pure_legacy_m30_report.json
```

Diagnose boundary jumps:

```bash
python archive/v3_experiment_scripts/run_v3_0_roll_boundary_diagnostics.py \
  --command-log testdata/v3_0_34_pure_legacy_m30_commands.jsonl \
  --segment-key roll_index \
  --unwrap-continuous-angles \
  --output testdata/v3_0_34_boundary_report.json
```

Generate Gazebo preview commands:

```bash
python tools/command_generation/run_v3_0_resample_commands.py \
  --input testdata/v3_0_34_pure_legacy_m30_commands.jsonl \
  --unwrap-continuous-angles \
  --resample-factor 8 \
  --smooth-window 3 \
  --segment-key roll_index \
  --diagnose-boundaries \
  --output testdata/v3_0_34_pure_legacy_m30_x8_sw3_unwrap.jsonl
```

Replay:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 80 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 3.0 \
  --hold-end-sec 5.0 \
  --command-log testdata/v3_0_34_pure_legacy_m30_x8_sw3_unwrap.jsonl
```
