# Lily 8脚ロボット ソフトウェア

更新日: 2026-08-23  
対象: `master`

Lilyは **8脚 × 3自由度 = 24軸** のロボットで、本リポジトリではmotion生成、Gazebo検証、実機CAN runtime、MCU Config調整を扱う。

このREADMEはプロジェクト全体の入口であり、詳細は目的別の正本文書へ分ける。

---

## 1. 最初にどこを読むか

| やりたいこと | 最初に読む文書 |
|---|---|
| 文書全体の地図を見る | [`docs/README.md`](docs/README.md) |
| Jetson programの引数を調べる | [`docs/JETSON_ARGUMENT_REFERENCE.md`](docs/JETSON_ARGUMENT_REFERENCE.md) |
| MCU parameterの意味を調べる | [`docs/MCU_PARAMETER_REFERENCE.md`](docs/MCU_PARAMETER_REFERENCE.md) |
| CANを接続する / MCUパラメータを変更する | [`docs/CAN_MCU_CONFIG_GUIDE.md`](docs/CAN_MCU_CONFIG_GUIDE.md) |
| コマンドをそのままコピーする | [`docs/COPY_PASTE_COMMANDS.md`](docs/COPY_PASTE_COMMANDS.md) |
| 実機試験を行う | [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md) |
| UI command / CAN IDを調べる | [`docs/COMMAND_REFERENCE.md`](docs/COMMAND_REFERENCE.md) |
| MCU Config GUIを使う | [`tools/mcu_config/README.md`](tools/mcu_config/README.md) |
| CAN StateMachineを理解する | [`tools/can_interface/README.md`](tools/can_interface/README.md) |
| current candidate / baselineを見る | [`docs/CURRENT_BASELINE.md`](docs/CURRENT_BASELINE.md) |
| current validation statusを見る | [`docs/VALIDATION_STATUS.md`](docs/VALIDATION_STATUS.md) |
| motion生成・評価programを使う | [`docs/MOTION_DEVELOPMENT_GUIDE.md`](docs/MOTION_DEVELOPMENT_GUIDE.md) |
| JSONLを作る | [`docs/JSONL_CREATION_GUIDE.md`](docs/JSONL_CREATION_GUIDE.md) |
| runtime構成を理解する | [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md) |
| physical joint limitを見る | [`docs/HARDWARE_LIMITS.md`](docs/HARDWARE_LIMITS.md) |

---

## 2. 実機で最初に行うCAN設定

現行のCAN初期設定は次の2コマンド。

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

詳細は [`docs/CAN_MCU_CONFIG_GUIDE.md`](docs/CAN_MCU_CONFIG_GUIDE.md) を正本とする。

---

## 3. JetsonとMCUのparameter境界

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

## 4. MCU Configの位置付け

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

## 5. Runtime architecture

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

## 6. 実機試験の基本順序

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

## 7. Repository map

```text
lily_motion/
├── README.md
├── lily_motion_v3/              # motion / geometry / shared runtime core
├── tools/
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

## 8. 文書の正本ルール

同じ事実を複数文書で正本化しない。

```text
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

historical `v3_0_*` noteや `archive/` はcurrent operationを上書きしない。
