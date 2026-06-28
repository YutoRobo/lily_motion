# v3.0.42E effort replay plot - fixed

## Issue observed

The uploaded CSV files contain headers only and no data rows. This means the plot code did not receive usable `/joint_states.effort` samples during the replay.

This can happen when:

1. `/joint_states` is not being published while the script runs.
2. The topic name differs from `/joint_states`.
3. `/joint_states` messages are published, but `effort[]` is empty.
4. The script starts replay before the subscriber connection is ready.
5. The script is running under a different `ROS_MASTER_URI`/terminal environment than Gazebo.

## Fixes added

- The script is now placed inside the project directory as well as provided standalone.
- If `--plot-dir` is omitted but `--output` is given, a plot directory is created automatically next to the JSON output.
- A preflight `/joint_states` check was added.
- The script now waits briefly after subscriber setup before replay.
- The script waits after replay before unregistering the subscriber.
- If no effort samples are recorded, the JSON report includes `data_collection_warning` and `preflight_joint_states` diagnostics.
- Empty-effort cases are now reported explicitly instead of silently producing header-only CSVs.

## Recommended pre-checks

```bash
rostopic list | grep joint_states
rostopic echo -n 1 /joint_states/name
rostopic echo -n 1 /joint_states/effort
```

If `effort: []` is shown, the Gazebo/controller side is not publishing effort values to `/joint_states`.

## Recommended command

```bash
python run_v3_0_42e_effort_replay_plot.py \
  --command-log testdata/v3_0_42c_case27/candidate_01_x8_sw40_commands.jsonl \
  --strict-command-log-input \
  --rate 15 \
  --hold-start-sec 2.0 \
  --hold-end-sec 2.0 \
  --pre-replay-wait-sec 2.0 \
  --post-replay-wait-sec 2.0 \
  --wait-for-joint-states-sec 10.0 \
  --diagnose-command-log \
  --effort-limit 40 \
  --output testdata/v3_0_42e_effort_case27.json \
  --plot-dir testdata/v3_0_42e_effort_case27_plots
```

## Expected outputs

- JSON summary
- `effort_time_series.csv`
- `effort_joint_series.csv`
- `effort_time_series.png`
- `effort_top_joints.png`
- `effort_top_phases.png`
- `effort_heatmap_top_joints.png`

If only CSV headers are produced again, inspect `data_collection_warning` and `preflight_joint_states` in the JSON report.
