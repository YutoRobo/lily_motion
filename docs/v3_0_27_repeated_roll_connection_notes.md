# v3.0.27 repeated legacy roll connection evaluation

This version adds a repeated-roll evaluator for the legacy state-machine emulator.
It runs a forward surface sequence such as `1,5,6,2,1` on one continuous vendored runtime instance and evaluates:

- second-joint limit over all quarter rolls,
- ground penetration,
- approximate inter-leg tibia distance,
- terminal body position relative to the next surface's support set center.

The connection metric is deliberately simple: body XY projection versus the centroid of the next support legs. It is not a full dynamic stability proof, but it catches the failure mode where one quarter-roll is locally safe while ending in a poor state for the next quarter-roll.

Gazebo check example:

```bash
python archive/v3_experiment_scripts/run_v3_0_repeated_legacy_roll_eval.py \
  --surface-sequence 1,5,6,2,1 \
  --support-dist 0.76 \
  --legacy-body-z 0.41 \
  --move-dist 0.32 \
  --goal2-pitch-scale 0.85 \
  --output-command-log testdata/v3_0_27_repeated_roll_commands.jsonl \
  --report-output testdata/v3_0_27_repeated_roll_report.json

python tools/command_generation/run_v3_0_resample_commands.py \
  --input testdata/v3_0_27_repeated_roll_commands.jsonl \
  --resample-factor 8 \
  --smooth-window 3 \
  --output testdata/v3_0_27_repeated_roll_resampled_x8.jsonl

python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 10 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 3.0 \
  --hold-end-sec 5.0 \
  --command-log testdata/v3_0_27_repeated_roll_resampled_x8.jsonl \
  --verbose-publish
```
