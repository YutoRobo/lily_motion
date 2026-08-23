# Lily Documentation Map

更新日: 2026-08-23

この文書は `docs/` の**索引と文書責務の正本**である。

同じ事実を複数文書で正本化せず、目的ごとに参照先を分ける。

---

## 1. 目的別の入口

まず次の大分類から選ぶ。

```text
A. 実機を動かす
   └─ CAN接続 / Config確認 / 実機試験 / 実行コマンド

B. Gazeboで確認する
   └─ Gazebo command path / staged replay / MCU相当補間

C. 設定値や通信仕様を調べる
   └─ Jetson引数 / MCU parameter / CAN ID / StateMachine

D. Motionを開発する
   └─ motion生成 / JSONL / runtime構成

E. 現在の状態を確認する
   └─ candidate / baseline / validation / hardware limit

F. 文書全体・プロジェクト全体を確認する
   └─ project入口 / docs索引
```

### A. 実機を動かす

| やりたいこと | 正本 / 最初に読む文書 |
|---|---|
| CANを接続する / MCU Configを確認・変更する | [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md) |
| 実機試験を行う | [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) |
| コマンドをそのままコピーして実行する | [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md) |
| Config GUIを使う | [`../tools/mcu_config/README.md`](../tools/mcu_config/README.md) |

### B. Gazeboで確認する

| やりたいこと | 正本 / 最初に読む文書 |
|---|---|
| Gazeboで現行command pathを使う | [`GAZEBO_USAGE_GUIDE.md`](GAZEBO_USAGE_GUIDE.md) |
| staged JSONLをGazeboで再生する | [`GAZEBO_USAGE_GUIDE.md`](GAZEBO_USAGE_GUIDE.md) |
| Gazebo / 実機のruntime差を理解する | [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) |
| Gazeboで使うJetson側引数を確認する | [`JETSON_ARGUMENT_REFERENCE.md`](JETSON_ARGUMENT_REFERENCE.md) |

### C. 設定値や通信仕様を調べる

| やりたいこと | 正本 / 最初に読む文書 |
|---|---|
| Jetson programの各引数を調べる | [`JETSON_ARGUMENT_REFERENCE.md`](JETSON_ARGUMENT_REFERENCE.md) |
| MCU Config parameterの意味を調べる | [`MCU_PARAMETER_REFERENCE.md`](MCU_PARAMETER_REFERENCE.md) |
| UI command / CAN IDの意味を調べる | [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md) |
| CAN StateMachineを理解する | [`../tools/can_interface/README.md`](../tools/can_interface/README.md) |

### D. Motionを開発する

| やりたいこと | 正本 / 最初に読む文書 |
|---|---|
| motion生成・評価programを使う | [`MOTION_DEVELOPMENT_GUIDE.md`](MOTION_DEVELOPMENT_GUIDE.md) |
| JSONLを新しく作る | [`JSONL_CREATION_GUIDE.md`](JSONL_CREATION_GUIDE.md) |
| JSONL field / candidate package仕様を確認する | [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md) |
| runtime構成を理解する | [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) |

### E. 現在の状態を確認する

| やりたいこと | 正本 / 最初に読む文書 |
|---|---|
| 現在のcandidate / baselineを確認する | [`CURRENT_BASELINE.md`](CURRENT_BASELINE.md) |
| 現在どこまで検証済みか確認する | [`VALIDATION_STATUS.md`](VALIDATION_STATUS.md) |
| hardware joint limitを確認する | [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md) |

### F. 文書全体・プロジェクト全体を確認する

| やりたいこと | 正本 / 最初に読む文書 |
|---|---|
| プロジェクト全体を知る | [`../README.md`](../README.md) |
| docsの構成・責務を確認する | この `docs/README.md` |

---

## 2. 実機operatorの推奨読書順

毎回すべてを読むのではなく、目的に応じて使い分ける。

```text
実機操作を始める
  ↓
CAN_MCU_CONFIG_GUIDE.md
  ↓
CURRENT_BASELINE.md / VALIDATION_STATUS.md
  ↓
HARDWARE_OPERATION_PROCEDURE.md
  ↓
COPY_PASTE_COMMANDS.md
```

途中で意味を確認したくなった場合:

```text
Jetson引数を忘れた
→ JETSON_ARGUMENT_REFERENCE.md

MCU parameterを忘れた
→ MCU_PARAMETER_REFERENCE.md

CAN ID / UI commandを忘れた
→ COMMAND_REFERENCE.md
```

---

## 3. Gazebo userの推奨読書順

```text
../README.md
  ↓
GAZEBO_USAGE_GUIDE.md
  ↓
CURRENT_BASELINE.md / VALIDATION_STATUS.md
  ↓
必要に応じて JETSON_ARGUMENT_REFERENCE.md
  ↓
必要に応じて RUNTIME_ARCHITECTURE.md
```

重要:

- Gazebo trialではCAN StateMachineを `/cmdForJetson` に接続しない。
- 現在このrepositoryにはGazebo world / Lily model / joint controllerを起動するlaunch fileは含まれていない。
- 既存Gazebo環境を起動した後のcommand pathを `GAZEBO_USAGE_GUIDE.md` で正本化する。

---

## 4. Motion developerの推奨読書順

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

## 5. 文書の責務

```text
../README.md
  = project全体入口

GAZEBO_USAGE_GUIDE.md
  = Gazebo利用手順、shared command path、Gazebo MCU-equivalent interpolationの正本

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

## 6. Jetson / MCU parameterの切り分け

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

## 7. CAN関連の正本ルール

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

## 8. MCU Config文書ルール

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

## 9. Historical documents

次はcurrent正本ではなく、履歴・互換用。

| 旧path | current正本 |
|---|---|
| `../README_V3_CORE.md` | `MOTION_DEVELOPMENT_GUIDE.md` |
| `BASELINE.md` | `CURRENT_BASELINE.md` |
| `HARDWARE_PRETEST_STATUS.md` | `VALIDATION_STATUS.md` |
| `Lily_8leg_Robot_Command_Reference.md` | `COMMAND_REFERENCE.md` |

`docs/v3_0_*` や `archive/` は開発履歴として残すが、現行実験の操作正本として使用しない。

---

## 10. Change control

current authoritative documentを変更する場合:

1. この `docs/README.md` の責務を確認する。
2. 同じ情報を複数箇所へコピーして正本化しない。
3. Gazeboの利用手順を変えたら `GAZEBO_USAGE_GUIDE.md` を更新する。
4. Jetson program引数を変えたら `JETSON_ARGUMENT_REFERENCE.md` を更新する。
5. MCU Config parameterの意味・default・適用先を変えたら `MCU_PARAMETER_REFERENCE.md` を更新する。
6. 実行コマンドを変えたら `COPY_PASTE_COMMANDS.md` を更新する。
7. CAN protocol / Config手順を変えたら `CAN_MCU_CONFIG_GUIDE.md` を更新する。
8. 実機の試験順序を変えたら `HARDWARE_OPERATION_PROCEDURE.md` を更新する。
9. immutable evidence documentは書き換えない。
