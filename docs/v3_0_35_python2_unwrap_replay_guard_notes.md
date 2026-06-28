# v3.0.35 Python2 unwrap fix and replay guard

Fixes `math.isfinite` usage that fails on Python 2.7.
Adds `--strict-command-log-input` to `run_v3_0_gazebo_replay.py` so a missing processed command log does not silently fall back to generated native preview replay.

Recommended baseline remains pure legacy repeated roll:

```bash
python run_v3_0_pure_legacy_repeated_roll.py \
  --surface-sequence 1,5,6,2,1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --legacy-body-z 0.35 \
  --max-step 30 \
  --skip-constraints \
  --output-command-log testdata/v3_0_35_pure_legacy_repeated_m30_commands.jsonl \
  --report-output testdata/v3_0_35_pure_legacy_repeated_m30_report.json
```

Then process commands:

```bash
python run_v3_0_resample_commands.py \
  --input testdata/v3_0_35_pure_legacy_repeated_m30_commands.jsonl \
  --unwrap-continuous-angles \
  --resample-factor 8 \
  --smooth-window 3 \
  --segment-key roll_index \
  --diagnose-boundaries \
  --output testdata/v3_0_35_pure_legacy_m30_x8_sw3_unwrap.jsonl
```

Replay strictly:

```bash
python run_v3_0_gazebo_replay.py \
  --strict-command-log-input \
  --rate 80 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 3.0 \
  --hold-end-sec 5.0 \
  --command-log testdata/v3_0_35_pure_legacy_m30_x8_sw3_unwrap.jsonl
```
