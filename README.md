# Lily 8脚ロボット ソフトウェア概要

更新日: 2026-08-12  
対象: `master`（pre-hardware baseline以降）

本リポジトリは、Lily 8脚ロボットの**回転移動**を中心に、運動生成、運動学、制約評価、Gazebo検証、ROS指令配信、CAN状態機械、操作UI、段階実機試験をまとめたソフトウェア一式である。

このREADMEは、Lilyを初めて見る人が

```text
何をするロボットか
→ どのデータが軌道か
→ どのプログラムが実行経路か
→ Gazeboと実機がどこまで共通か
→ 現在どこまで検証済みか
→ 次にどの文書を読むべきか
```

を把握するための入口である。

実機を操作する場合は、READMEだけで判断せず、必ず [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md) に従う。

---

## 1. Lilyとは

Lilyは、**8脚 × 各3自由度 = 24軸**を持つロボットである。

各脚は概念上:

```text
body
 └─ base / yaw
     └─ thigh / pitch
         └─ tibia / pitch
```

の3関節で構成される。

軸番号は基本的に:

```text
axis = 3 × leg_index + joint_index
```

で対応する。

例:

```text
axis9  = leg3 base
axis10 = leg3 thigh
axis11 = leg3 tibia
```

現在の実機hard gate:

| joint index | name | hard gate |
|---:|---|---:|
| 0 | base / yaw | ±360 deg |
| 1 | thigh / pitch | ±95 deg |
| 2 | tibia / pitch | ±150 deg |

現在の共有geometry:

```text
coxa  = 0.075 m
thigh = 0.300 m
tibia = 0.300 m
```

詳細:

- [`lily_motion_v3/robot_geometry.py`](lily_motion_v3/robot_geometry.py)
- [`docs/HARDWARE_LIMITS.md`](docs/HARDWARE_LIMITS.md)

---

## 2. このプロジェクトでいう「回転」とは

本プロジェクトの主対象は通常の脚歩行だけではなく、**脚を使って本体を次の面へ倒し、連続的に転がる回転移動**である。

概念的には:

```text
現在の支持姿勢
   ↓
支持脚・遊脚を切り替える
   ↓
次の面へ移るための脚姿勢を作る
   ↓
本体をrollさせる
   ↓
次の支持姿勢
```

を1つのroll blockとして扱い、それらを接続して連続回転を構成する。

command recordには、動作を追跡するために例えば:

```text
roll_index
phase_name
surface_start
surface_after
```

等のmetadataが含まれることがある。

これらは「どの回転・どのphaseか」を示す情報であり、**実機へ送る24軸position値そのものは `joint_command_rad`** である。

過去のRF-1〜RF-6や各種sweep/constraint検討の詳細は `docs/v3_0_*` の履歴noteに残っているが、現在の実機operationはcurrent authoritative documentsを優先する。

---

## 3. 開発から実行までの全体像

Lilyの回転アルゴリズムは、最終的に24軸のcommand sequenceへ変換して実行する。

```text
motion algorithm / parameters
        ↓
command generation
        ↓
candidate command sequence
        ↓
diagnostics / geometry / Gazebo evaluation
        ↓
reference candidate freeze
        ↓
staged command files
        ↓
canonical command transport
        ↓
/cmdForJetson
        ↓
real hardware or Gazebo MCU-equivalent path
```

重要なのは、**軌道生成・candidate評価**と、**freeze後の実行transport**を分けて考えることである。

---

## 4. 現在のreference candidate

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

candidate directoryの主要構造:

```text
v3_0_44_candidate_022_wide_urdf0p075/
├── README.md
├── commands.jsonl
├── manifest.json
├── summary.json
├── pre_hardware_decision.md
├── staged/
│   ├── air_entry_and_hold_only_commands.jsonl
│   ├── combined_with_hold_commands.jsonl
│   ├── roll_0_50_commands.jsonl
│   ├── roll_50_100_commands.jsonl
│   ├── roll_100_300_commands.jsonl
│   └── roll_300_end_commands.jsonl
└── reports/
```

### 主要ファイルの意味

```text
commands.jsonl
  = frozen source trajectory / source keyframes

manifest.json
  = candidateのidentity / provenance / 採用根拠

summary.json
  = candidate packageの索引 / staged file範囲 / checksum要約

pre_hardware_decision.md
  = 人間向けの採用判断と実機試験順序

staged/*.jsonl
  = 段階実機試験用の実行単位

reports/
  = 診断・比較・評価証跡
```

JSON / JSONLのfield、source recordとtransport recordの違い、checksumの意味は:

- [`docs/COMMAND_DATA_FORMAT.md`](docs/COMMAND_DATA_FORMAT.md)

を参照する。

---

## 5. JSONL command recordの最小理解

`.jsonl` は**1行 = 1 command record**である。

代表例:

```json
{
  "frame_index": 0,
  "joint_command_rad": [0.0, 0.0, "... 24 axes ..."],
  "joint_command_deg": [0.0, 0.0, "..."],
  "roll_index": -1,
  "phase_name": "AIR_ENTRY_HOME_TO_CANDIDATE02_START"
}
```

実行上の最重要field:

```text
joint_command_rad
```

- exactly 24 values
- unit: rad
- `/cmdForJetson.position` へ送るposition source

`joint_command_deg`, `phase_name`, `roll_index`, `base_pose` 等は主にhuman-readable / traceability / diagnostics用metadataである。

canonical publisherは互換性のため:

```text
joint_command_rad
position
joint_positions_rad
```

のいずれかを受理し、内部で24要素の `joint_command_rad` へ正規化する。

---

## 6. Source trajectory と transportは別

frozen `commands.jsonl` / `staged/*.jsonl` はsource command recordsである。

現在の実機試験候補transport profile:

```text
resample-factor = 2
transport rate  = 10 Hz
```

factor 2では、隣接source target間にlinear midpointを1つ追加する。

```text
source:
q0 -------- q1 -------- q2

transport:
q0 -- m01 -- q1 -- m12 -- q2
```

これはfrozen JSONLを書き換える処理ではない。

さらに、transportの下流で実MCUがactuator-side interpolationを行う。

Gazeboで実MCU相当として使用したprofile:

```text
interpolation duration = 0.100 s
update period          = 0.002 s
```

したがって:

```text
source trajectory resolution
transport resampling / target rate
MCU interpolation duration
MCU internal update period
```

は**別々の概念・別々の設定**である。

---

## 7. 2026-08-12 pre-hardware baseline

実機直前ソフトウェアbaseline:

```text
commit:
3ff47e223c2ba67b3f6bf62de327f71de5226d86

branch:
baseline/pre-hardware-gazebo-pass-20260812

record:
docs/BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md
```

current baseline入口:

- [`docs/BASELINE.md`](docs/BASELINE.md)

このbaselineはtrajectory candidateそのものを変更せず、実機直前のsoftware / transport / Gazebo validation条件を固定した記録である。

---

## 8. 最重要: 現行runtime architecture

実機とGazeboは、**`/cmdForJetson` まで同じprogram path・同じtransport command stream**を使う。

```text
                 ┌──────────────────────────────────┐
                 │          completely shared       │
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

原則:

- canonical publisherにGazebo backend分岐を作らない。
- Gazebo試験時はCAN StateMachineを起動しない。
- 実機試験時はGazebo MCU補間nodeを起動しない。
- `/cmdForJetson` に意図しない複数consumerを同時接続しない。

---

## 9. 本番位置指令interface

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

- JSONL source positionの24要素validation / normalization
- linear transport resampling
- `--resample-factor`
- `--rate`
- `--start-index`
- `--max-frames`
- `--segment-key`
- `--dry-run`
- first publish前のsubscriber待ち
- transport stream SHA256表示
- CANを直接開かない
- Gazebo固有処理を持たない

---

## 10. Gazeboの2種類の使い方

### 10.1 実機等価経路の検証

実機へ送るのと同じ `/cmdForJetson` streamを検証するときは次を使う。

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

2026-08-12にこの経路で:

```text
HOME → air-entry
     → roll_0_50
     → roll_50_100
     → roll_100_300
     → roll_300_end
```

を順に確認し、全区間PASSした。

### 10.2 開発・診断用direct Gazebo replay

```text
tools/gazebo/run_v3_0_gazebo_replay.py
```

はtrajectory開発・目視・診断・履歴再現に使用する。

これはformal hardware-equivalent pathではない。

---

## 11. 現行コードの場所

### Motion / kinematics / evaluation

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
```

開発・診断:

```text
tools/gazebo/run_v3_0_gazebo_replay.py
tools/gazebo/run_v3_0_gazebo_touchdown_pose_check.py
```

### Hardware CAN / UI

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

---

## 12. Current vs historical data

現行実機operationでは次を直接実行しない。

```text
archive/
external/can_interface/
```

用途:

```text
data/reference_candidates/
  = freeze済みreference candidate

testdata/
  = 生成途中・評価結果・比較・診断・試験証跡

archive/
  = 過去実験・旧runner・旧資料・再現用

external/
  = 外部由来・旧snapshot
```

`testdata/` にあるファイルが自動的に正式candidateという意味ではない。

---

## 13. 実機試験の基本順

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

---

## 14. 現在の検証状態

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

---

## 15. 安全原則

- 実機時はphysical emergency isolationを即時操作できる状態にする。
- 可動範囲へ人を入れない。
- 対象外axisは `Use=False`。
- ALIGN / HOME方向 / SET HOME姿勢を確認する。
- single-axis → one-leg → staged rollの順序を飛ばさない。
- Gazebo試験時にCAN StateMachineを同時起動しない。
- 実機試験時にGazebo MCU nodeを同時起動しない。
- `archive/` の旧runnerを現行操作に使わない。
- frozen candidate JSONLを直接編集しない。
- transport profileを変更した場合はnew baselineとして記録する。

---

## 16. 初見者向け文書の読み順

### 全体を理解したい

```text
README.md
  ↓
docs/RUNTIME_ARCHITECTURE.md
  ↓
docs/COMMAND_DATA_FORMAT.md
  ↓
README_V3_CORE.md
```

### current candidateを理解したい

```text
README.md
  ↓
docs/COMMAND_DATA_FORMAT.md
  ↓
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/README.md
  ↓
manifest.json / summary.json
  ↓
pre_hardware_decision.md
```

### 実機を動かしたい

```text
README.md
  ↓
docs/BASELINE.md
  ↓
docs/HARDWARE_PRETEST_STATUS.md
  ↓
docs/HARDWARE_OPERATION_PROCEDURE.md
  ↓
docs/Lily_8leg_Robot_Command_Reference.md
```

---

## 17. 現行authoritative documents

- [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md): runtime境界とtiming設計
- [`docs/COMMAND_DATA_FORMAT.md`](docs/COMMAND_DATA_FORMAT.md): JSON / JSONL / candidate data仕様
- [`docs/BASELINE.md`](docs/BASELINE.md): current baseline入口
- [`docs/BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](docs/BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md): frozen baseline evidence
- [`docs/HARDWARE_LIMITS.md`](docs/HARDWARE_LIMITS.md): joint hard gate
- [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md): 実機操作正本
- [`docs/HARDWARE_PRETEST_STATUS.md`](docs/HARDWARE_PRETEST_STATUS.md): latest verification status
- [`docs/Lily_8leg_Robot_Command_Reference.md`](docs/Lily_8leg_Robot_Command_Reference.md): command集
- [`docs/kinematics_link_length_update_0p075.md`](docs/kinematics_link_length_update_0p075.md): geometry判断記録
- [`README_V3_CORE.md`](README_V3_CORE.md): v3 motion core開発・評価入口
- [`tools/can_interface/README.md`](tools/can_interface/README.md): CAN StateMachine

`docs/v3_0_*` は開発履歴であり、現行operation仕様を上書きしない。

---

## 18. 開発・変更管理ルール

- `master` は統合済みcurrent baseline。
- 実験変更はbranchで分離する。
- frozen candidateをsilent editしない。
- `/cmdForJetson` より上流のcanonical pathをGazebo/実機で分岐させない。
- source trajectory、transport timing、MCU interpolationを別概念として管理する。
- 実機でprofile変更が必要なら、理由と結果をnew candidate / baselineへ記録する。
- candidate名が同じままchecksumだけ変わるような運用を避ける。
