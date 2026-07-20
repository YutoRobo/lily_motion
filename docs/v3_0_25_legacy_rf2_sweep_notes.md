# v3.0.25 Legacy RF-2 focused sweep notes

## Purpose

v3.0.24 showed that the reproduced legacy roll exceeds the second-joint 95 deg constraint, with the worst frame usually in `RF-2_Goal2_UpperLegLanding`.

v3.0.25 keeps the legacy state-machine structure intact and adds narrow diagnostic parameters around RF-2 and adjacent middle-pair phases.

## Added parameters

Defaults reproduce the supplied legacy controller.

- `goal2_dist_front` default `0.4`
- `goal2_x_scale` default `1.0`
- `goal2_pitch_scale` default `1.0`
- `goal2_landing_z` default `0.0`
- `goal3_lift_z` default `0.05`
- `goal3_target_x` default `0.2`
- `goal4_target_x` default `0.05`

These are diagnostic knobs. They are not yet a final gait design.

## Basic evaluation

```bash
python archive/v3_experiment_scripts/run_v3_0_legacy_constraint_eval.py \
  --surface-id 1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --max-step 30 \
  --goal2-dist-front 0.35 \
  --goal2-pitch-scale 0.8 \
  --goal2-landing-z 0.03 \
  --output-command-log testdata/v3_0_25_candidate_commands.jsonl \
  --report-output testdata/v3_0_25_candidate_report.json
```

## Small RF-2 sweep

The evaluator uses the vendored legacy FK and is intentionally heavy. Start with a small sweep.

```bash
python archive/v3_experiment_scripts/run_v3_0_legacy_rf2_sweep.py \
  --goal2-dist-fronts 0.30,0.40 \
  --goal2-x-scales 0.8,1.0 \
  --goal2-pitch-scales 0.8,1.0 \
  --goal2-landing-zs 0.0,0.03 \
  --output testdata/v3_0_25_rf2_small_sweep.json \
  --best-command-output testdata/v3_0_25_rf2_best_commands.jsonl \
  --best-report-output testdata/v3_0_25_rf2_best_report.json
```

## Gazebo check

```bash
python tools/command_generation/run_v3_0_resample_commands.py \
  --input testdata/v3_0_25_rf2_best_commands.jsonl \
  --resample-factor 4 \
  --smooth-window 3 \
  --output testdata/v3_0_25_rf2_best_commands_resampled.jsonl

python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 60 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/v3_0_25_rf2_best_commands_resampled.jsonl \
  --verbose-publish
```

## Interpretation

Do not trust a single best score blindly. Confirm in Gazebo that:

1. RF-2 still resembles the original roll sequence.
2. RF-3/RF-4 middle-pair step does not become unnatural.
3. second-joint improvement does not introduce leg collision or ground penetration.
