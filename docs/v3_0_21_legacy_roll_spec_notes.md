# v3.0.21 legacy_roll_spec

This version adds a table-based reproduction of the old `LilyRobotController.roll(Direction.FORWARD)` structure.

It does **not** import the old project.  The old code was read and converted into checked-in tables:

- RF-1_Goal1_UpperLegPreSwing
- RF-2_Goal2_UpperLegLanding
- RF-3_Goal3_LiftMiddlePair
- RF-4_Goal4_LandMiddlePair
- RF-5_Goal5_MainBodyRoll
- RF-6_StopAdjustment

The reproduced ideas are:

- surface sequence: 1 → 5 → 6 → 2 → 1 for forward roll;
- legacy leg ids: 0 BLF, 1 BLH, 2 BRF, 3 BRH, 4 TLF, 5 TLH, 6 TRF, 7 TRH;
- support legs keep phase-start absolute/world foot points;
- landing legs linearly interpolate toward absolute/world targets;
- swing legs use direct joint-angle commands corresponding to old `swing_leg_type`;
- base motion is pitch-only plus x translation, matching `util.py`.

Known limitations:

- v3 still uses its project-contained FK/IK model, not the exact symbolic `leg.py` model.
- old `solve_type=-1` branch selection and `±2π` singular flip behavior are approximated.
- This is the first useful bridge toward faithful legacy reproduction, not the final reproduction.

## Evaluate

```bash
python archive/v3_experiment_scripts/run_v3_0_legacy_roll_spec_eval.py --summary-only \
  --surface-id 1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --max-step 30 \
  --filter-window 3
```

## Export for Gazebo

```bash
python tools/command_generation/run_v3_0_export_commands.py \
  --profile legacy_roll_spec \
  --surface-id 1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --max-step 30 \
  --filter-window 3 \
  --command-source filtered \
  --output testdata/v3_0_21_legacy_roll_spec_commands.jsonl
```

The first spec-level candidate can be marked invalid at frame 0 by the portable v3 geometry checker because the v3 model is not the exact legacy symbolic leg model.  To visually inspect the full current command sequence in Gazebo, export all frames:

```bash
python tools/command_generation/run_v3_0_export_commands.py \
  --profile legacy_roll_spec \
  --surface-id 1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --max-step 30 \
  --filter-window 3 \
  --command-source filtered \
  --allow-invalid-frames \
  --output testdata/v3_0_21_legacy_roll_spec_all_commands.jsonl
```

## Replay in Gazebo

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 20 \
  --frame-hold-sec 0.25 \
  --hold-start-sec 2.0 \
  --hold-end-sec 3.0 \
  --command-log testdata/v3_0_21_legacy_roll_spec_all_commands.jsonl \
  --gazebo-link-state-log testdata/v3_0_21_legacy_roll_spec_gazebo_link_states.jsonl \
  --verbose-publish
```

Use `--allow-invalid-frames` if you want to inspect the entire command sequence even when the evaluator marks frames as invalid.
