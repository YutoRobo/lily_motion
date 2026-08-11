# Lily Motion v3-core

更新日: 2026-08-12

`lily_motion_v3/` はLily回転歩容のプロジェクト内完結型coreである。旧 `LilyRobot`、hidden legacy IK、xacro loadをruntime依存にしない。

この文書は **motion生成・評価の開発入口** を説明する。実機操作の正本ではない。

現行状態:

- [`README.md`](README.md)
- [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md)
- [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md)

## Current geometry

```text
lily_motion_v3/robot_geometry.py
```

```text
coxa  = 0.075 m
thigh = 0.300 m
tibia = 0.300 m
```

旧0.05 m coxaを用いたgeometry-derived reportは、正確な現行geometry判断には使用しない。

## Current frozen pre-hardware candidate

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

```text
command count:                 2233
maximum second-joint angle:    94.8 deg
violations over 95 deg:        0
Gazebo full-roll review:       PASS
hardware full roll:            NOT TESTED
```

開発中に生成したJSONLが自動的に実機承認されることはない。実機はreview済みの `data/reference_candidates/` 配下を使用する。

## Core responsibilities

主な責務:

- robot geometry
- FK / IK
- contact plan
- RF phase handling
- repeated roll generation
- legacy/reference comparison
- command resampling utilities
- constraint evaluation support

現行runtime transportに追加された共通module:

```text
lily_motion_v3/command_stream.py
lily_motion_v3/command_timing.py
```

Gazebo専用MCU-equivalent logic:

```text
lily_motion_v3/gazebo_actuator_interpolator.py
```

`command_stream.py` / `command_timing.py` はGazeboやCANを知らない。MCU simulationはGazebo側に分離する。

## Quick evaluation

```bash
python tools/diagnostics/run_v3_0_whole_roll_eval.py --summary-only
```

Failure diagnosis:

```bash
python tools/diagnostics/run_v3_0_diagnose_failures.py
```

Visualization:

```bash
python tools/diagnostics/run_v3_0_visualize_roll.py --help
```

Parameter sweep:

```bash
python tools/diagnostics/run_v3_0_parameter_sweep.py --help
```

探索結果は `testdata/` に置き、review後にだけ正式候補へ昇格する。

## Command export

```bash
python tools/command_generation/run_v3_0_export_commands.py --help
python tools/command_generation/run_v3_0_import_legacy_reference.py --help
python tools/command_generation/run_v3_0_resample_commands.py --help
```

生成・変換したcommand logは、diagnosticsとGazebo reviewを通してからfreezeする。

## Gazebo: two different purposes

### Development direct replay

```text
tools/gazebo/run_v3_0_gazebo_replay.py
```

これは軌道開発、目視、診断、履歴再現に使う。

例:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --dry-run \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/commands.jsonl \
  --strict-command-log-input \
  --rate 15
```

### Hardware-equivalent runtime verification

実機とのcommand path比較にはdirect replayではなく、次を使う。

```text
staged JSONL
→ tools/publish_cmdforjetson_jsonl.py
→ /cmdForJetson
→ tools/gazebo/mcu_position_interpolator_node.py
→ Gazebo
```

詳細:

- [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md)

現行pre-hardware profile:

```text
transport resample factor = 2
transport rate            = 10 Hz
MCU interpolation         = 0.100 s
MCU update period         = 0.002 s
```

この経路でair-entryから全split rollまでPASS済み。

## Runtime boundary

v3-coreはCANを直接送信しない。

```text
frozen/staged JSONL
→ tools/publish_cmdforjetson_jsonl.py
→ /cmdForJetson
```

以降は:

```text
REAL:
  StateMachine → CAN → real MCU

GAZEBO:
  mcu_position_interpolator_node.py → Gazebo
```

## Archive boundary

`archive/` は履歴・再現用。現行実機操作のentry pointではない。

`docs/v3_0_*` の古い開発noteは、その時点の設計判断を記録する履歴資料であり、現在のruntime仕様を上書きしない。

## Development rules

- exploratory outputは `testdata/`
- frozen candidateは直接編集しない
- geometry判断は現行0.075 m model/URDFを基準にする
- `/cmdForJetson` より上流にhardware/Gazebo別trajectory pathを作らない
- transport resamplingとtrajectory smoothingを混同しない
- source trajectory、transport timing、MCU interpolation timingを独立管理する
