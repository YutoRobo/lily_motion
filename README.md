# Lily 8脚ロボット ソフトウェア

更新日: 2026-08-29  
対象: `master`

Lilyは **8脚 × 3自由度 = 24軸** のロボットで、本リポジトリではmotion生成、Gazebo検証、実機CAN runtime、MCU Config調整を扱う。

このREADMEはプロジェクト全体の入口であり、詳細は目的別の正本文書へ分ける。

## Quick start: `./lily`

日常操作の入口はrepository rootの `./lily` に統一する。

```bash
./lily status
./lily doctor real
./lily viewer --leg-index 3
./lily config --axes 11
```

staged motionは、引数を省略するとcurrent CLI profileのcandidate / transport設定を使用する。

```bash
./lily play roll-1of4
```

`play` は既定で **dry-run** であり、`/cmdForJetson` へpublishしない。実行する場合だけ明示的に `--execute` を付ける。

```bash
./lily play roll-1of4 --execute
```

単軸・1脚試験も既定ではcommand previewのみで、`--execute` を付けた場合だけpublisherを起動する。

```bash
./lily test axis 10
./lily test axis 10 --execute

./lily test leg 3 --mode individual
./lily test leg 3 --mode individual --execute
```

CLIのmachine-readable defaultは [`config/lily_cli_profile.json`](config/lily_cli_profile.json) に置く。安全な実機試験順序・STOP条件は引き続き [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md) を優先する。

---

## 1. 最初にどこを読むか

まず、やりたいことを次の大分類から選ぶ。

```text
A. 実機を動かす
   └─ CAN接続 / Config確認 / 実機試験 / 実行コマンド

B. Gazeboで確認する
   └─ Gazebo起動後のcommand path / staged replay / MCU相当補間

C. 設定値や通信仕様を調べる
   └─ Jetson引数 / MCU parameter / CAN ID / StateMachine

D. Motionを開発する
   └─ motion生成 / JSONL作成 / runtime構成

E. 現在の状態を確認する
   └─ current baseline / validation status / hardware limit

F. 文書全体を探す
   └─ docs全体の索引
```

### A. 実機を動かす

| やりたいこと | 最初に読む文書 |
|---|---|
| CANを接続する / MCUパラメータを確認・変更する | [`docs/CAN_MCU_CONFIG_GUIDE.md`](docs/CAN_MCU_CONFIG_GUIDE.md) |
| 実機試験を行う | [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md) |
| コマンドをそのままコピーする | [`docs/COPY_PASTE_COMMANDS.md`](docs/COPY_PASTE_COMMANDS.md) |
| MCU Config GUIを使う | [`tools/mcu_config/README.md`](tools/mcu_config/README.md) |

### B. Gazeboで確認する

| やりたいこと | 最初に読む文書 |
|---|---|
| Gazeboで現行command pathを使う | [`docs/GAZEBO_USAGE_GUIDE.md`](docs/GAZEBO_USAGE_GUIDE.md) |
| staged JSONLをGazeboで再生する | [`docs/GAZEBO_USAGE_GUIDE.md`](docs/GAZEBO_USAGE_GUIDE.md) |
| Gazebo / 実機のruntime差を理解する | [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md) |
| `resample_factor` やGazebo補間引数を調べる | [`docs/JETSON_ARGUMENT_REFERENCE.md`](docs/JETSON_ARGUMENT_REFERENCE.md) |

### C. 設定値や通信仕様を調べる

| やりたいこと | 最初に読む文書 |
|---|---|
| Jetson programの引数を調べる | [`docs/JETSON_ARGUMENT_REFERENCE.md`](docs/JETSON_ARGUMENT_REFERENCE.md) |
| MCU parameterの意味を調べる | [`docs/MCU_PARAMETER_REFERENCE.md`](docs/MCU_PARAMETER_REFERENCE.md) |
| UI command / CAN IDを調べる | [`docs/COMMAND_REFERENCE.md`](docs/COMMAND_REFERENCE.md) |
| CAN StateMachineを理解する | [`tools/can_interface/README.md`](tools/can_interface/README.md) |

### D. Motionを開発する

| やりたいこと | 最初に読む文書 |
|---|---|
| motion生成・評価programを使う | [`docs/MOTION_DEVELOPMENT_GUIDE.md`](docs/MOTION_DEVELOPMENT_GUIDE.md) |
| JSONLを作る | [`docs/JSONL_CREATION_GUIDE.md`](docs/JSONL_CREATION_GUIDE.md) |
| runtime構成を理解する | [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md) |

### E. 現在の状態を確認する

| やりたいこと | 最初に読む文書 |
|---|---|
| current candidate / baselineを見る | [`docs/CURRENT_BASELINE.md`](docs/CURRENT_BASELINE.md) |
| current validation statusを見る | [`docs/VALIDATION_STATUS.md`](docs/VALIDATION_STATUS.md) |
| physical joint limitを見る | [`docs/HARDWARE_LIMITS.md`](docs/HARDWARE_LIMITS.md) |

### F. 文書全体を探す

| やりたいこと | 最初に読む文書 |
|---|---|
| 文書全体の地図を見る | [`docs/README.md`](docs/README.md) |

---

## 2. 実機で最初に行うCAN設定

現行のCAN初期設定は次の2コマンド。

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

詳細は [`docs/CAN_MCU_CONFIG_GUIDE.md`](docs/CAN_MCU_CONFIG_GUIDE.md) を正本とする。

---

## 3. Gazeboの位置付け

Gazeboでは、実機と同じ `/cmdForJetson` までのcommand pathを使う。

```text
staged JSONL
→ publish_cmdforjetson_jsonl.py
→ /cmdForJetson
→ mcu_position_interpolator_node.py
→ Gazebo
```

詳細は [`docs/GAZEBO_USAGE_GUIDE.md`](docs/GAZEBO_USAGE_GUIDE.md) を参照する。

注意: 現在このrepositoryにはLily本体・Gazebo world・joint controllerを起動するlaunch fileは含まれていないため、既存Gazebo環境を別途起動する必要がある。

---

## 4. JetsonとMCUのparameter境界

Jetson側:

```text
--resample-factor
--rate
--command-log
--axis
--leg-index
--can-channel
...
```

→ [`docs/JETSON_ARGUMENT_REFERENCE.md`](docs/JETSON_ARGUMENT_REFERENCE.md)

MCU側:

```text
gear_ratio
motor_direction
Kp / Ki / Kd
position jump / error limit
interpolation_time_ms
torque ramp
...
```

→ [`docs/MCU_PARAMETER_REFERENCE.md`](docs/MCU_PARAMETER_REFERENCE.md)

特に:

```text
Jetson --resample-factor
≠
MCU interpolation_time_ms
```

である。

---

## 5. MCU Configの位置付け

MCU Config GUI:

```text
tools/mcu_config/lily_mcu_config_editor.py
```

用途:

- gear ratio / motor direction確認
- joint limit設定値確認
- PID確認・調整
- position jump / error threshold確認
- interpolation time確認
- torque ramp設定確認
- HardwareConfig / SoftwareConfig SAVE

基本操作:

```text
READ
→ 必要ならWRITE
→ Echo / same-parameter READ back
→ 永続化が必要な場合だけSAVE
```

HardwareConfig SAVE後はMCUを再起動する。

---

## 6. Runtime architecture

```text
motion / staged JSONL
        ↓
tools/publish_cmdforjetson_jsonl.py
        ↓
/cmdForJetson
        │
        ├─ REAL
        │    ↓
        │  tools/can_interface/statemachine/
        │    ↓
        │   CAN → real MCU → motor
        │
        └─ GAZEBO
             ↓
           tools/gazebo/mcu_position_interpolator_node.py
             ↓
           Gazebo
```

実機時にGazebo MCU nodeを同時に `/cmdForJetson` へ接続しない。

---

## 7. 実機試験の基本順序

```text
CAN setup
→ 必要ならMCU Config READ確認
→ ROS / StateMachine / UI起動
→ single axis
→ one leg
→ 24-axis mapping/HOME check
→ suspended air-entry
→ controlled touchdown
→ risk-split roll
→ semantic quarter
→ final combined
```

正確なSTOP条件と進行条件は [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md) を優先する。

---

## 8. Repository map

```text
lily_motion/
├── README.md
├── lily                         # unified operator CLI launcher
├── config/
│   └── lily_cli_profile.json    # CLI execution defaults
├── lily_motion_v3/              # motion / geometry / shared runtime core
├── tools/
│   ├── lily_cli.py              # CLI implementation
│   ├── can_interface/           # CAN StateMachine / UI / emulator
│   ├── mcu_config/              # MCU Config GUI
│   ├── command_generation/      # command generation tools
│   ├── diagnostics/             # diagnostics / evaluation
│   ├── gazebo/                  # Gazebo MCU-equivalent path
│   └── publish_cmdforjetson_*   # hardware/Gazebo shared publishers
├── data/
│   ├── reference_candidates/
│   └── baselines/
├── docs/
│   ├── README.md                       # documentation map
│   ├── GAZEBO_USAGE_GUIDE.md           # Gazebo利用手順の正本
│   ├── JETSON_ARGUMENT_REFERENCE.md    # Jetson CLI argument正本
│   ├── MCU_PARAMETER_REFERENCE.md      # MCU Config parameter正本
│   ├── CAN_MCU_CONFIG_GUIDE.md         # CAN / MCU Config操作正本
│   ├── COPY_PASTE_COMMANDS.md
│   ├── HARDWARE_OPERATION_PROCEDURE.md
│   ├── COMMAND_REFERENCE.md
│   └── ...
├── tests/
├── testdata/
└── archive/
```

---

## 9. 文書の正本ルール

同じ事実を複数文書で正本化しない。

```text
Gazebo利用手順
→ docs/GAZEBO_USAGE_GUIDE.md

Jetson program引数
→ docs/JETSON_ARGUMENT_REFERENCE.md

MCU parameterの意味
→ docs/MCU_PARAMETER_REFERENCE.md

CAN setup / Config操作
→ docs/CAN_MCU_CONFIG_GUIDE.md

実行コマンド
→ docs/COPY_PASTE_COMMANDS.md

安全な実機順序
→ docs/HARDWARE_OPERATION_PROCEDURE.md

command / CAN IDの意味
→ docs/COMMAND_REFERENCE.md

current baseline
→ docs/CURRENT_BASELINE.md

current validation
→ docs/VALIDATION_STATUS.md
```

`config/lily_cli_profile.json` は上記current baselineを実行時に参照するためのmachine-readable CLI defaultであり、baselineの安全判断やvalidation statusを置き換えない。

historical `v3_0_*` noteや `archive/` はcurrent operationを上書きしない。
