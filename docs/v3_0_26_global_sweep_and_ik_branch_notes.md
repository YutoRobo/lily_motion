# v3.0.26 Global posture sweep and IK branch diagnostics

## Purpose

v3.0.25 showed that a narrow RF-2 landing-target sweep did not reduce the main second-joint violation. The worst case remained around BRF thigh ≈ 109 deg in `RF-2_Goal2_UpperLegLanding`, with violations persisting into RF-3/RF-4/RF-5.

v3.0.26 adds two tools:

1. `run_v3_0_legacy_global_sweep.py`
   - Sweeps broad posture parameters: `support_dist`, `legacy_body_z`, `move_dist`, and `goal2_pitch_scale`.
   - Keeps the legacy state-machine structure intact.
   - Exports the best command log for Gazebo replay.

2. `run_v3_0_ik_branch_diagnose.py`
   - Looks at the worst or specified frame and leg.
   - Recomputes four legacy analytical IK branches for the same foot target.
   - Reports whether any branch satisfies the 95 deg second-joint limit.

## Interpretation

If the IK branch diagnostic reports a feasible branch, the next issue is branch selection.
If no branch satisfies the second-joint limit, the next issue is geometry: body height, support footprint, roll displacement, or phase target positions.

## Recommended first run

```bash
python run_v3_0_legacy_global_sweep.py \
  --support-dists 0.60,0.65,0.70,0.75 \
  --legacy-body-zs 0.30,0.35,0.40 \
  --move-dists 0.30,0.35,0.40 \
  --goal2-pitch-scales 0.70,0.85,1.0 \
  --output testdata/v3_0_26_global_sweep.json \
  --best-command-output testdata/v3_0_26_global_best_commands.jsonl \
  --best-report-output testdata/v3_0_26_global_best_report.json
```

Then diagnose the worst frame:

```bash
python run_v3_0_ik_branch_diagnose.py \
  --command-log testdata/v3_0_26_global_best_commands.jsonl \
  --surface-id 1 \
  --output testdata/v3_0_26_global_best_ik_branch_report.json
```

Gazebo replay:

```bash
python run_v3_0_resample_commands.py \
  --input testdata/v3_0_26_global_best_commands.jsonl \
  --resample-factor 4 \
  --smooth-window 3 \
  --output testdata/v3_0_26_global_best_commands_resampled.jsonl

python run_v3_0_gazebo_replay.py \
  --rate 60 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/v3_0_26_global_best_commands_resampled.jsonl \
  --verbose-publish
```
