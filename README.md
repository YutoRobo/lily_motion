# Lily 8脚ロボット ソフトウェア概要

更新日: 2026-08-12  
対象: `master`（pre-hardware baseline以降）

本リポジトリは、Lily 8脚ロボットの回転歩容生成、運動学、制約評価、Gazebo検証、ROS指令配信、CAN状態機械、操作UI、段階実機試験をまとめたソフトウェア一式である。

このREADMEは「現在どの経路を使うか」を示す入口である。実機操作は必ず [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md) に従う。

## 1. 現在の基準

現在のpre-hardware回転候補:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

主要値:

```text
command count             2233
coxa                      0.075 m
thigh                     0.300 m
tibia                     0.300 m
maximum second joint      94.8 deg
violations over 95 deg    0
Gazebo full-roll review   PASS
hardware full roll        NOT TESTED
```

2026-08-12の実機直前ソフトウェアbaseline:

```text
commit:
3ff47e223c2ba67b3f6bf62de327f71de5226d86

branch:
baseline/pre-hardware-gazebo-pass-20260812

record:
docs/BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md
```

現在の実機試験候補transport profile:

```text
resample-factor = 2
transport rate  = 10 Hz
```

Gazeboで実MCU相当として使用した補間profile:

```text
interpolation duration = 0.100 s
update period          = 0.002 s
```

これらは別々の設定であり、将来のMCU変更時にも独立して変更できる設計とする。

## 2. 最重要: 現行runtime architecture

実機とGazeboは、`/cmdForJetson` まで同じプログラム・同じ指令列を使う。

```text
                 ┌──────────────────────────────────┐
                 │          completely shared       │
                 │                                  │
staged/frozen JSONL
        ↓
tools/publish_cmdforjetson_jsonl.py
        ↓
lily_motion_v3.command_stream
        ↓
linear transport resampling
        ↓
/cmdForJetson
sensor_msgs/JointState
position: exactly 24 [rad]
                 └────────────────┬─────────────────┘
                                  │
                   ┌──────────────┴──────────────┐
                   │                             │
                 REAL                         GAZEBO
                   │                             │
tools/can_interface/                 tools/gazebo/
statemachine/StateMachine            mcu_position_interpolator_node.py
                   │                             │
                  CAN                    MCU-equivalent interpolation
                   │                             │
                real MCU                    Gazebo joints
                   │
                 motor
```

正式なarchitecture詳細:

- [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md)

重要:

- canonical publisherにはGazebo backend分岐を作らない。
- Gazebo試験時はCAN StateMachineを起動しない。
- 実機試験時はGazebo MCU補間nodeを起動しない。
- `/cmdForJetson` に複数の意図しないconsumerを同時接続しない。

## 3. Gazeboの2種類の使い方

### 3.1 実機等価経路の検証

実機へ送るのと同じ `/cmdForJetson` streamを検証するときは、次の2プログラムを使う。

Terminal A:

```bash
python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

Terminal B:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log <staged.jsonl> \
  --resample-factor 2 \
  --rate 10
```

2026-08-12にこの経路で、

```text
HOME → air-entry
     → roll_0_50
     → roll_50_100
     → roll_100_300
     → roll_300_end
```

を順に確認し、全区間PASSした。

### 3.2 開発・診断用direct Gazebo replay

```text
tools/gazebo/run_v3_0_gazebo_replay.py
```

は、軌道生成中の目視、診断、履歴再現に使用できる。

ただしこれは `/cmdForJetson → MCU-equivalent interpolation` の正式な実機等価経路とは別である。実機との対応確認では3.1を使用する。

## 4. 本番位置指令interface

```text
Topic: /cmdForJetson
Message: sensor_msgs/JointState
position length: exactly 24
unit: rad
```

canonical JSONL publisher:

```text
tools/publish_cmdforjetson_jsonl.py
```

主な機能:

- `joint_command_rad` / `position` / `joint_positions_rad` を24要素へ正規化
- source frameを保持したlinear transport resampling
- `--resample-factor`
- `--rate`
- `--start-index`
- `--max-frames`
- `--dry-run`
- 最初のpublish前にsubscriber接続待ち
- transport stream SHA256表示
- CANを直接開かない

## 5. ロボット表現

```text
8 legs × 3 DOF = 24 axes
```

| joint index | name | current hard gate |
|---:|---|---:|
| 0 | base / yaw | ±360 deg |
| 1 | thigh / pitch | ±95 deg |
| 2 | tibia / pitch | ±150 deg |

```text
axis = 3 × leg_index + joint_index
```

例:

```text
axis9  = leg3 base
axis10 = leg3 thigh
axis11 = leg3 tibia
```

現在の共有geometry:

```text
coxa  = 0.075 m
thigh = 0.300 m
tibia = 0.300 m
```

source:

- [`lily_motion_v3/robot_geometry.py`](lily_motion_v3/robot_geometry.py)
- [`docs/HARDWARE_LIMITS.md`](docs/HARDWARE_LIMITS.md)

## 6. 現行コード

### Motion / evaluation

```text
lily_motion_v3/
tools/command_generation/
tools/diagnostics/
```

### Canonical command transport

```text
lily_motion_v3/command_stream.py
lily_motion_v3/command_timing.py
tools/publish_cmdforjetson_jsonl.py
```

### Gazebo

```text
tools/gazebo/mcu_position_interpolator_node.py
lily_motion_v3/gazebo_actuator_interpolator.py

development / diagnostics:
tools/gazebo/run_v3_0_gazebo_replay.py
tools/gazebo/run_v3_0_gazebo_touchdown_pose_check.py
```

### Hardware CAN

```text
tools/can_interface/statemachine/main.py
tools/can_interface/initUI/ui.py
tools/can_interface/emulator/
```

### Staged hardware test publishers

```text
tools/publish_cmdforjetson_single_axis_test.py
tools/publish_cmdforjetson_one_leg_test.py
tools/publish_cmdforjetson_mapped_axis_replay.py
tools/publish_cmdforjetson_jsonl.py
```

## 7. Current vs historical

現行実機運用では次を直接実行しない。

```text
archive/
external/can_interface/
```

`archive/` は再現・履歴参照用、`external/` は外部由来・旧snapshotである。

正式な候補は:

```text
data/reference_candidates/
```

生成途中・評価結果は主に:

```text
testdata/
```

に置く。

## 8. 実機試験の基本順

既知正常の旧6脚歩行系と、現行masterのCAN＋回転系は別ソフトとして扱う。

現行回転系:

```text
single axis
→ one leg / three axes
→ suspended air-entry
→ controlled touchdown
→ roll 0–50
→ roll 50–100
→ roll 100–300
→ roll 300–end
→ final combined sequence
```

full rollから開始しない。

詳細:

- [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md)

## 9. 現在の検証状態

### Software / Gazebo

```text
shared stream tests                PASS
transport resampling tests         PASS
online MCU interpolator tests      PASS
canonical-path air-entry           PASS
canonical-path roll 0–50           PASS
canonical-path roll 50–100         PASS
canonical-path roll 100–300        PASS
canonical-path roll 300–end        PASS
final target hold in Gazebo        PASS
```

### Hardware

```text
real axis10 +0.002 rad              visually provisional PASS
real axis10 negative 0.002 rad      not yet confirmed
real axis10 ±0.005 rad              not yet confirmed
one-leg three-axis                  not yet confirmed
air-entry                           not yet confirmed
staged roll                         not yet confirmed
full roll                           not yet confirmed
```

最新状態:

- [`docs/HARDWARE_PRETEST_STATUS.md`](docs/HARDWARE_PRETEST_STATUS.md)

## 10. 安全原則

- 実機時は物理非常停止を即時操作できる状態にする。
- 可動範囲へ人を入れない。
- 対象外軸は `Use=False`。
- ALIGN / HOME方向 / SET HOME姿勢を確認する。
- single-axis → one-leg → staged rollの順序を飛ばさない。
- Gazebo試験時にCAN StateMachineを同時起動しない。
- 実機試験時にGazebo MCU nodeを同時起動しない。
- `archive/` の旧runnerを現行操作に使わない。
- frozen candidate JSONLを直接編集しない。
- transport profileを変更した場合は新baselineとして記録する。

## 11. 文書

現行の正本:

- [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md): runtime境界とタイミング設計
- [`docs/BASELINE.md`](docs/BASELINE.md): current baseline入口
- [`docs/BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](docs/BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md): 凍結証跡
- [`docs/HARDWARE_LIMITS.md`](docs/HARDWARE_LIMITS.md): 関節limit
- [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md): 実機操作
- [`docs/HARDWARE_PRETEST_STATUS.md`](docs/HARDWARE_PRETEST_STATUS.md): 現在の試験状態
- [`docs/Lily_8leg_Robot_Command_Reference.md`](docs/Lily_8leg_Robot_Command_Reference.md): コマンド集
- [`docs/kinematics_link_length_update_0p075.md`](docs/kinematics_link_length_update_0p075.md): geometry変更記録
- [`README_V3_CORE.md`](README_V3_CORE.md): v3 motion core開発・評価入口
- [`tools/can_interface/README.md`](tools/can_interface/README.md): CAN StateMachine

`docs/v3_0_*` の開発noteは履歴資料であり、現在の運用仕様を上書きしない。

## 12. 開発ルール

- `master` は統合済み現行基準。
- 実験変更はbranchで分離する。
- frozen candidateは直接変更しない。
- `/cmdForJetson` より上流のcanonical pathをGazebo/実機で分岐させない。
- source trajectory、transport rate、MCU interpolationを別概念として管理する。
- 実機でprofile変更が必要なら、理由と結果を新baselineへ記録する。
