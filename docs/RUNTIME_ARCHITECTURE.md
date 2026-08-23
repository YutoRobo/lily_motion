# Lily Runtime Architecture

更新日: 2026-08-23

この文書は、**motion command経路とMCU Config経路の境界**を示す正本である。

---

## 1. Motion runtime path

Current common path:

```text
frozen / staged JSONL
        ↓
tools/publish_cmdforjetson_jsonl.py
        ↓
shared normalization / resampling
        ↓
/cmdForJetson
```

ここまでは実機/Gazebo共通。

Hardware:

```text
/cmdForJetson
→ tools/can_interface/statemachine/
→ CAN
→ real MCU
→ motor
```

Gazebo:

```text
/cmdForJetson
→ tools/gazebo/mcu_position_interpolator_node.py
→ Gazebo joint controllers
```

publisher自身はhardware/Gazebo backendを選ばない。

---

## 2. Config pathはmotion pathと別

MCU Configは `/cmdForJetson` を通らない。

```text
lily_mcu_config_editor.py
→ candump / cansend
→ can0
→ Config CAN request  0x080 | axis
→ MCU
→ Config CAN response 0x180 | axis
```

つまり:

```text
motion command path
≠
MCU parameter configuration path
```

この分離により、motion algorithm側へPIDやgear ratioの設定処理を混ぜない。

---

## 3. CAN interface setup

実機CANのOS-level setup:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

詳細:

- [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md)

---

## 4. StateMachine responsibility

StateMachine owns:

- runtime CAN ID
- runtime payload encoding
- Use gate
- ALIGN / HOME / RUN session
- position command送信
- joint limit check
- error latch
- CAN send failure handling

StateMachineはMCU Config GUIのparameter persistenceを管理しない。

---

## 5. Config GUI responsibility

Config GUI owns:

- Config READ
- Config WRITE
- MCU Echo確認
- same-parameter READ back
- HardwareConfig SAVE
- SoftwareConfig SAVE
- response timeout / missing axis表示

Config GUIはmotion trajectoryを生成・publishしない。

---

## 6. RuntimeとConfigを同時に扱うときのルール

MCU仕様:

```text
READ  : 全stateで可
WRITE : aliment_standbyのみ
SAVE  : aliment_standbyのみ
```

したがって通常の実機trialでは:

```text
trial前
→ Config READ確認
→ 必要ならstandbyでWRITE/SAVE
→ HardwareConfig SAVEならpower cycle
→ runtime起動
→ ALIGN / HOME / RUN
→ motion trial
```

RUN中はConfig WRITE / SAVEしない。

---

## 7. `/cmdForJetson` rule

```text
Message: sensor_msgs/JointState
position length: exactly 24
unit: rad
```

Hardware trial時:

```text
intended consumer = CAN StateMachine
```

Gazebo trial時:

```text
intended consumer = Gazebo MCU-equivalent node
```

両者を同時に接続しない。

---

## 8. Transport timing

Current staged profile:

```text
resample-factor = 2
rate            = 10 Hz
```

これはhost-side transport timing。

MCU側には別にinterpolation処理があるため、次を分ける。

```text
trajectory generation
host transport timing
MCU interpolation
motor control loop
```

---

## 9. Physical limits and Config values

Physical joint limitの正本:

- [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md)

Config GUIに保存された `joint_min_rad` / `joint_max_rad` はMCU Config parameterであるが、現段階ではphysical safety documentationの正本を置き換えない。

---

## 10. Related

- [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md)
- [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)
- [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md)
- [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md)
- [`../tools/can_interface/README.md`](../tools/can_interface/README.md)
- [`../tools/mcu_config/README.md`](../tools/mcu_config/README.md)
