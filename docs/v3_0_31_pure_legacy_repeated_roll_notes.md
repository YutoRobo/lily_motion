v3.0.31 pure legacy repeated roll
===================================

Purpose
-------
This version deliberately stops optimizing parameters. It verifies whether the
vendored legacy state machine can reproduce repeated forward rolls using one
continuous runtime instance.

Critical fix
------------
Previous repeated-roll attempts reused the quarter-roll generator, but landing
absolute x targets inside the emulator were still local to each quarter roll
(`x = 0`). The supplied legacy controller uses cumulative controller state
(`self.__x`) in RF-2/RF-3/RF-4 target positions, then after each roll snaps
`lily.posture` to `(self.__x, self.__pitch)`. v3.0.31 adds equivalent
`_controller_x` and `_controller_pitch` state to the vendored emulator.

This is not an optimization version. It is a migration/debug version.

Run
---

python run_v3_0_pure_legacy_repeated_roll.py \
  --surface-sequence 1,5,6,2,1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --legacy-body-z 0.35 \
  --max-step 30 \
  --constraint-stride 4 \
  --output-command-log testdata/v3_0_31_pure_legacy_repeated_commands.jsonl \
  --report-output testdata/v3_0_31_pure_legacy_repeated_report.json

For quick debugging, use `--max-step 10 --skip-constraints` first. The full
vendored legacy IK can be slow under Python 3/SymPy for repeated rolls.

Gazebo preview
--------------

python run_v3_0_resample_commands.py \
  --input testdata/v3_0_31_pure_legacy_repeated_commands.jsonl \
  --resample-factor 8 \
  --smooth-window 3 \
  --output testdata/v3_0_31_pure_legacy_repeated_resampled_x8.jsonl

python run_v3_0_gazebo_replay.py \
  --rate 10 \
  --frame-hold-sec 0.0 \
  --hold-start-sec 3.0 \
  --hold-end-sec 5.0 \
  --command-log testdata/v3_0_31_pure_legacy_repeated_resampled_x8.jsonl \
  --verbose-publish

What to inspect
---------------
- candidate_completed
- generation_error
- roll_summaries
- terminal_updates
- final_lily_posture
- constraints.max_second_joint_deg
- constraints.inter_leg_near_count

If this pure legacy repeated roll fails, the remaining issue is still migration
or state carry-over, not parameter optimization.
