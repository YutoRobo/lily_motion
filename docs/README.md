# Documentation Status

更新日: 2026-08-12

`docs/` には現行の運用文書と、v3開発過程の履歴noteが共存している。

## 初見者の推奨読書順

### ソフトウェア全体を理解する

```text
../README.md
  ↓
RUNTIME_ARCHITECTURE.md
  ↓
COMMAND_DATA_FORMAT.md
  ↓
../README_V3_CORE.md
```

### current candidateを理解する

```text
../README.md
  ↓
COMMAND_DATA_FORMAT.md
  ↓
../data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/README.md
  ↓
manifest.json / summary.json
  ↓
pre_hardware_decision.md
```

### 実機試験を行う

```text
../README.md
  ↓
BASELINE.md
  ↓
HARDWARE_PRETEST_STATUS.md
  ↓
HARDWARE_OPERATION_PROCEDURE.md
  ↓
Lily_8leg_Robot_Command_Reference.md
```

## Current authoritative documents

優先順位の高い現行文書:

1. [`../README.md`](../README.md) — Lily / software / current stateの全体像
2. [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) — 実機/Gazebo共通runtimeとtiming layer
3. [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md) — JSON / JSONL / candidate directory / command record仕様
4. [`BASELINE.md`](BASELINE.md) — current baseline入口
5. [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) — 実機操作正本
6. [`HARDWARE_PRETEST_STATUS.md`](HARDWARE_PRETEST_STATUS.md) — latest verification status
7. [`Lily_8leg_Robot_Command_Reference.md`](Lily_8leg_Robot_Command_Reference.md) — command集
8. [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md) — joint hard gate
9. [`kinematics_link_length_update_0p075.md`](kinematics_link_length_update_0p075.md) — geometry判断記録

Frozen evidence:

- [`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md)

## 文書の役割分担

```text
README.md
  = 初見者向け全体像 / current entry point

RUNTIME_ARCHITECTURE.md
  = program boundary / shared path / timing architecture

COMMAND_DATA_FORMAT.md
  = data boundary / JSON / JSONL / source-vs-transport record

BASELINE.md
  = current frozen conditionへの入口

HARDWARE_PRETEST_STATUS.md
  = 現在どこまで試験済みか

HARDWARE_OPERATION_PROCEDURE.md
  = 実機をどう動かすか

Lily_8leg_Robot_Command_Reference.md
  = exact command lookup
```

## Historical development notes

`v3_0_*` のnoteは、そのversion時点の検討履歴である。

それらに書かれた:

- current baseline
- old link length
- old Gazebo replay rate
- old smoothing/resampling setting
- old candidate path

は、現行operationを上書きしない。

履歴noteは再現性のため当面残すが、実機操作時は上記authoritative documentsだけを使用する。

## Archived operational documents

重複・staleなoperation文書とexperiment logは `archive/docs_legacy/` に保存する。

Git履歴があるため、削除済みbackup fileも復元可能。
