# Lily Documentation Map

更新日: 2026-08-23

この文書は `docs/` の**索引と文書責務の正本**である。

同じ事実を複数文書で正本化せず、目的ごとに参照先を分ける。

---

## 1. 目的別の入口

| やりたいこと | 正本 / 最初に読む文書 |
|---|---|
| プロジェクト全体を知る | [`../README.md`](../README.md) |
| Jetson programの各引数を調べる | [`JETSON_ARGUMENT_REFERENCE.md`](JETSON_ARGUMENT_REFERENCE.md) |
| MCU Config parameterの意味を調べる | [`MCU_PARAMETER_REFERENCE.md`](MCU_PARAMETER_REFERENCE.md) |
| CANを接続する / MCU Configを変更する | [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md) |
| コマンドをそのままコピーして実行する | [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md) |
| 実機試験を行う | [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) |
| UI command / CAN IDの意味を調べる | [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md) |
| Config GUIを使う | [`../tools/mcu_config/README.md`](../tools/mcu_config/README.md) |
| CAN StateMachineを理解する | [`../tools/can_interface/README.md`](../tools/can_interface/README.md) |
| 現在のcandidate / baselineを確認する | [`CURRENT_BASELINE.md`](CURRENT_BASELINE.md) |
| 現在どこまで検証済みか確認する | [`VALIDATION_STATUS.md`](VALIDATION_STATUS.md) |
| motion生成・評価programを使う | [`MOTION_DEVELOPMENT_GUIDE.md`](MOTION_DEVELOPMENT_GUIDE.md) |
| JSONLを新しく作る | [`JSONL_CREATION_GUIDE.md`](JSONL_CREATION_GUIDE.md) |
| JSONL field / candidate package仕様を確認する | [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md) |
| Gazebo / 実機のruntimeを理解する | [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) |
| hardware joint limitを確認する | [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md) |

---

## 2. 実機operatorの推奨読書順

```text
../README.md
  ↓
JETSON_ARGUMENT_REFERENCE.md
  ↓
MCU_PARAMETER_REFERENCE.md
  ↓
CAN_MCU_CONFIG_GUIDE.md
  ↓
CURRENT_BASELINE.md
  ↓
VALIDATION_STATUS.md
  ↓
HARDWARE_OPERATION_PROCEDURE.md
  ↓
COPY_PASTE_COMMANDS.md
```

普段は全部を毎回読む必要はない。

- 引数を忘れた → `JETSON_ARGUMENT_REFERENCE.md`
- MCU parameterを忘れた → `MCU_PARAMETER_REFERENCE.md`
- 実際に操作する → `CAN_MCU_CONFIG_GUIDE.md` / `COPY_PASTE_COMMANDS.md`

---

## 3. Motion developerの推奨読書順

```text
../README.md
  ↓
MOTION_DEVELOPMENT_GUIDE.md
  ↓
JSONL_CREATION_GUIDE.md
  ↓
COMMAND_DATA_FORMAT.md
  ↓
RUNTIME_ARCHITECTURE.md
```

---

## 4. 文書の責務

```text
../README.md
  = project全体入口

JETSON_ARGUMENT_REFERENCE.md
  = Jetson / host programのCLI引数、transport timing、resample_factorの正本

MCU_PARAMETER_REFERENCE.md
  = HardwareConfig / SoftwareConfig parameterの意味・default・反映箇所の正本

CAN_MCU_CONFIG_GUIDE.md
  = CAN接続、CAN command体系、MCU Config READ/WRITE/SAVEの正本

COPY_PASTE_COMMANDS.md
  = そのまま実行するコマンド集

HARDWARE_OPERATION_PROCEDURE.md
  = 実機試験の順序、安全条件、STOP条件

COMMAND_REFERENCE.md
  = UI command / CAN ID / Config protocolの意味

../tools/mcu_config/README.md
  = Config GUIの具体的な使い方

../tools/can_interface/README.md
  = CAN StateMachine実装・runtime側の説明

CURRENT_BASELINE.md
  = current candidate / SHA / transport / frozen stageの正本

VALIDATION_STATUS.md
  = current verification status

RUNTIME_ARCHITECTURE.md
  = source JSONLからreal/Gazeboまでのprogram boundary

MOTION_DEVELOPMENT_GUIDE.md
  = motion生成・評価・export toolの標準作業フロー

JSONL_CREATION_GUIDE.md
  = JSONL生成の具体例

COMMAND_DATA_FORMAT.md
  = JSON / JSONL / candidate packageのデータ仕様

HARDWARE_LIMITS.md
  = physical joint limitの正本
```

---

## 5. Jetson / MCU parameterの切り分け

```text
Jetson側
  --resample-factor
  --rate
  --command-log
  --axis
  --leg-index
  --can-channel
  ...
→ JETSON_ARGUMENT_REFERENCE.md

MCU側
  gear_ratio
  motor_direction
  Kp / Ki / Kd
  position_jump_limit_rad
  position_error_limit_rad
  interpolation_time_ms
  torque ramp
  ...
→ MCU_PARAMETER_REFERENCE.md
```

特に:

```text
--resample-factor
≠ interpolation_time_ms
```

である。

前者はJetson側transport target生成、後者はMCU内部target interpolation。

---

## 6. CAN関連の正本ルール

CANについては次のように分ける。

```text
CANをUPするコマンド
→ CAN_MCU_CONFIG_GUIDE.md

runtimeを起動するコマンド
→ COPY_PASTE_COMMANDS.md

Jetson側引数
→ JETSON_ARGUMENT_REFERENCE.md

MCU parameterの意味
→ MCU_PARAMETER_REFERENCE.md

UI command / CAN IDの意味
→ COMMAND_REFERENCE.md

StateMachine内部仕様
→ tools/can_interface/README.md

MCU parameter変更手順
→ CAN_MCU_CONFIG_GUIDE.md
→ tools/mcu_config/README.md
```

CAN初期設定は次の2コマンドを正本とする。

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

---

## 7. MCU Config文書ルール

通常のパラメータ変更はGUIを使用する。

```text
tools/mcu_config/lily_mcu_config_editor.py
```

通常運用:

```text
READ
→ 必要ならWRITE
→ Echo / 同一parameter READ back
→ 永続化が必要な場合だけSAVE
```

HardwareConfig SAVE後は再起動する。

raw `cansend` は診断・protocol確認用とし、通常の調整操作の第一選択にはしない。

---

## 8. Historical documents

次はcurrent正本ではなく、履歴・互換用。

| 旧path | current正本 |
|---|---|
| `../README_V3_CORE.md` | `MOTION_DEVELOPMENT_GUIDE.md` |
| `BASELINE.md` | `CURRENT_BASELINE.md` |
| `HARDWARE_PRETEST_STATUS.md` | `VALIDATION_STATUS.md` |
| `Lily_8leg_Robot_Command_Reference.md` | `COMMAND_REFERENCE.md` |

`docs/v3_0_*` や `archive/` は開発履歴として残すが、現行実験の操作正本として使用しない。

---

## 9. Change control

current authoritative documentを変更する場合:

1. この `docs/README.md` の責務を確認する。
2. 同じ情報を複数箇所へコピーして正本化しない。
3. Jetson program引数を変えたら `JETSON_ARGUMENT_REFERENCE.md` を更新する。
4. MCU Config parameterの意味・default・適用先を変えたら `MCU_PARAMETER_REFERENCE.md` を更新する。
5. 実行コマンドを変えたら `COPY_PASTE_COMMANDS.md` を更新する。
6. CAN protocol / Config手順を変えたら `CAN_MCU_CONFIG_GUIDE.md` を更新する。
7. 実機の試験順序を変えたら `HARDWARE_OPERATION_PROCEDURE.md` を更新する。
8. immutable evidence documentは書き換えない。
