# v3.0.29 Constrained parameterization

This version changes the search order.

Previous sweeps treated `move_dist` as a free parameter and selected candidates
mostly by second-joint margin. That can create a one-quarter-roll motion that
looks safe but cannot connect to the next 90 degree roll.

v3.0.29 introduces a constrained parameterization:

1. Choose `support_dist`, `legacy_body_z`, and a named pitch profile.
2. Search `move_dist` internally.
3. Select the `move_dist` that minimizes the periodicity/connection error.
4. Reject the candidate before constraint scoring if the periodicity thresholds
   are not satisfied.
5. Evaluate only the remaining admissible candidates for second-joint limit,
   ground penetration, inter-leg proximity, and repeated-roll generation.

This is still a first implementation. The periodicity gate currently reuses the
repeated-roll connection centroid metric. It is not a full support polygon or
dynamics proof, but it prevents the most obvious error: choosing a `move_dist`
that improves only the first quarter roll while destroying repeated-roll
connection.

Example:

```bash
python archive/v3_experiment_scripts/run_v3_0_constrained_roll_sweep.py \
  --support-dists 0.72,0.74,0.76 \
  --legacy-body-zs 0.38,0.40,0.42 \
  --pitch-profiles legacy,balanced,late_roll \
  --move-dist-range 0.25,0.45 \
  --move-dist-samples 41 \
  --periodicity-max-error 0.08 \
  --periodicity-mean-error 0.04 \
  --second-joint-abs-max-deg 95 \
  --output testdata/v3_0_29_constrained_sweep.json \
  --best-command-output testdata/v3_0_29_constrained_best_commands.jsonl \
  --best-report-output testdata/v3_0_29_constrained_best_report.json
```

Gazebo check:

```bash
python tools/command_generation/run_v3_0_resample_commands.py \
  --input testdata/v3_0_29_constrained_best_commands.jsonl \
  --resample-factor 8 \
  --smooth-window 3 \
  --output testdata/v3_0_29_constrained_best_resampled_x8.jsonl

python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 10 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 3.0 \
  --hold-end-sec 5.0 \
  --command-log testdata/v3_0_29_constrained_best_resampled_x8.jsonl \
  --verbose-publish
```
