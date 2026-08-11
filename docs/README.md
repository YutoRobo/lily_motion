# Documentation Status

更新日: 2026-08-12

`docs/` には現行の運用文書と、v3開発過程の履歴noteが共存している。

## Current authoritative documents

優先順位の高い現行文書:

1. [`../README.md`](../README.md) — software全体像
2. [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) — 実機/Gazebo共通runtime
3. [`BASELINE.md`](BASELINE.md) — current baseline入口
4. [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) — 実機操作正本
5. [`HARDWARE_PRETEST_STATUS.md`](HARDWARE_PRETEST_STATUS.md) — 最新verification status
6. [`Lily_8leg_Robot_Command_Reference.md`](Lily_8leg_Robot_Command_Reference.md) — command集
7. [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md) — joint hard gate
8. [`kinematics_link_length_update_0p075.md`](kinematics_link_length_update_0p075.md) — geometry判断記録

Frozen evidence:

- [`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md)

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

2026-08-12の整理で、重複・staleなoperation文書とexperiment logは `archive/docs_legacy/` へ移す。

Git履歴があるため、削除されたbackup fileも復元可能。
