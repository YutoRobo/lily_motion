# v3.0.7 Gazebo replay timing fix

v3.0.6 exported only valid preview frames by default. With the current candidate,
this is 33 frames. At 30 Hz, this is only about 1.1 seconds, so Gazebo can look
as if it finishes instantly.

v3.0.7 keeps sequential publishing, but adds explicit visual-preview timing:

- `--frame-hold-sec`: republishes each generated frame for multiple controller ticks.
- `--hold-start-sec`: holds the first command before replay.
- `--hold-end-sec`: holds the final command after replay.
- `--verbose-publish`: prints each generated frame as it is streamed.
- `--dry-run-sleep`: makes dry-run actually wait, useful for timing checks.

Default preview timing is now:

```text
rate = 30 Hz
frame_hold_sec = 0.10 s
hold_start_sec = 1.0 s
hold_end_sec = 2.0 s
```

For the current 33-frame preview, this yields about 6.3 seconds of playback.

Example:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 30 \
  --frame-hold-sec 0.20 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --verbose-publish \
  --command-log testdata/v3_0_7_gazebo_preview_commands.jsonl
```

For timing-only check without ROS:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --dry-run \
  --dry-run-sleep \
  --frame-hold-sec 0.20
```

The command log remains one record per generated frame, while `published_count`
counts actual repeated publishes.
