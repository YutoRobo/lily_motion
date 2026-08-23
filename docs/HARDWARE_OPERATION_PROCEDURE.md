# Lily 実機操作手順

更新日: 2026-08-23  
対象: 現行 `master` / staged hardware validation

この文書は、**実機試験の順序、安全条件、STOP条件の正本**である。

コマンドそのものは [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md)、CAN接続とMCU Config操作は [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md) を使用する。

---

## 0. Path convention

この文書の `tools/...`、`docs/...`、`data/...` はすべて `lily_motion/` のrepository root基準である。

個人PC固有の絶対パスは使用しない。

repository内のsubdirectoryにいる場合は、必要に応じて次でrootへ戻る。

```bash
cd "$(git rev-parse --show-toplevel)"
```

---

## 1. Current runtime

```text
staged JSONL
→ tools/publish_cmdforjetson_jsonl.py
→ /cmdForJetson
→ tools/can_interface/statemachine/StateMachine
→ CAN
→ real MCU
```

Current transport:

```text
resample-factor = 2
rate            = 10 Hz
```

旧runner / archiveにある別rateをcurrent staged rollへ混用しない。

---

## 2. 実機試験で使わないもの

```text
archive/
external/can_interface/
tools/gazebo/mcu_position_interpolator_node.py
tools/gazebo/run_v3_0_gazebo_replay.py
```

実機試験時はGazebo MCU nodeを `/cmdForJetson` へ同時接続しない。

---

## 3. Physical safety

必須:

- physical emergency isolationを即操作できる
- 可動範囲に人を入れない
- robotを安全にsuspendできるfixture
- 対象外axisを `Use=False`
- CAN cable / power / mechanical fastening確認
- 異音、衝撃、予期しないmotion、heat、vibrationで即中止
- stageごとにSTOP可能なoperator配置

PC側STOPはphysical emergency isolationの代替ではない。

---

## 4. Trial前のGit / baseline確認

repository rootから:

```bash
git status -sb
git log -1 --oneline
```

current candidate / baselineの正確な値は [`CURRENT_BASELINE.md`](CURRENT_BASELINE.md) を確認する。

trial recordには使用commit SHAを残す。

---

## 5. CAN初期設定

実機CANは次の2コマンドを使用する。

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

確認:

```bash
ip -details link show can0
candump -L can0
```

CANの詳細は [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md) を正本とする。

---

## 6. MCU Configの事前確認

急ぎのmotion実験では、**Configを変更せずREAD確認だけ行う**のを基本とする。

Axis 11確認例。repository rootから:

```bash
python2 tools/mcu_config/lily_mcu_config_editor.py --interface can0 --axes 11
```

現在Axis 11で基準へ復元確認済み:

```text
Kp         = 500
gear_ratio = 30.8
```

必要な保存値であることを確認してからmotion試験へ進む。

パラメータを変更する場合:

```text
SoftwareConfig
aliment_standby
→ WRITE
→ Echo / same-parameter READ back
→ 必要ならSoftwareConfig SAVE

HardwareConfig
aliment_standby
→ WRITE
→ Echo / same-parameter READ back
→ HardwareConfig SAVE
→ MCU power cycle
→ READ再確認
```

SAVE中に電源を切らない。

physical joint limitは [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md) を正本とし、Config画面の保存値だけで安全判定しない。

---

## 7. ROS / StateMachine / UI起動

ROS:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

実機では `/use_sim_time` が `true` でないことを確認する。

```bash
rosparam get /use_sim_time 2>/dev/null || true
```

StateMachine / UIは [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md) のSection 3を使用する。

`/cmdForJetson` consumer確認:

```bash
rostopic info /cmdForJetson
```

実機時の意図したconsumerはCAN StateMachineである。

---

## 8. Runtime state progression

基本順序:

```text
Use設定
→ ALIGN
→ HOME jog / SET HOME
→ RUN
→ position / staged motion
→ STOP
```

RUNへ進む前に、active axisがconnected / aligned / homedであることを確認する。

Use setを変更するときはSTOPしてsessionを整理する。

---

## 9. First hardware progression

初回実機は次の順序を固定する。

```text
single-axis positive/negative small-angle
→ one-leg individual axes
→ one-leg coordinated
→ 24-axis mapping/HOME check
→ suspended air-entry
→ controlled touchdown
→ risk roll 0–50
→ risk roll 50–100
→ risk roll 100–300
→ risk roll 300–end
→ semantic quarter trial
→ final combined
```

semantic quarter fileが存在していても、初回risk-split確認を飛ばさない。

---

## 10. Single-axis validation

最初はrolling JSONLではなく専用publisherを使う。

```text
+0.002 rad
→ STOP / inspect
→ -0.002 rad
→ STOP / inspect
```

両方向PASS後に次の振幅へ進む。

exact commandは `COPY_PASTE_COMMANDS.md` を使用する。

---

## 11. One-leg validation

例: leg-index 3 = axes 9,10,11。

順序:

```text
axis 9 individual
→ axis 10 individual
→ axis 11 individual
→ 3-axis individual mode
→ 3-axis coordinated mode
```

対象3軸以外を `Use=False` とする。

---

## 12. Before 24-axis motion

必須確認:

- 24 actuator installation
- axis mapping
- sign
- each HOME direction
- each SET HOME
- no unexpected `Use=True`
- physical suspension
- STOP test
- no Gazebo MCU subscriber
- `/cmdForJetson` consumer確認
- Config baselineが意図値であること

不確定要素があればair-entryへ進まない。

---

## 13. Air-entry

条件:

```text
robot suspended
all required axes aligned/homed
HOME logical posture confirmed
RUN accepted
physical emergency path ready
```

Air-entryはHOMEからrolling-start postureへの遷移。

確認:

- initial jumpなし
- intended direction
- cable/fixture interferenceなし
- abnormal current/sound/vibration/heatなし
- final rolling-start postureで安定

異常時はSTOPし、touchdownへ進まない。

---

## 14. Controlled touchdown

Air-entry final postureを保持したまま接地させる。

確認:

- intended foot contact
- link/floor clearance
- unexpected slideなし
- supportが安定
- current/sound/vibration/heat

joint commandへ任意のtouchdown offsetを追加しない。

---

## 15. Initial risk-split roll

初回は必ず次の順序で進む。

```text
roll_0_50
→ inspect
→ roll_50_100
→ inspect
→ roll_100_300
→ inspect
→ roll_300_end
→ inspect
```

各stageで姿勢、接触、current、sound、vibration、temperature、STOP可能性を確認する。

前stage最終姿勢から次stageへ続ける。

---

## 16. Semantic quarter trial

正式file:

```text
roll_to_1of4_commands.jsonl
roll_to_2of4_commands.jsonl
roll_to_3of4_commands.jsonl
roll_to_4of4_commands.jsonl
```

quarterはrolling-startからの**累積prefix**。

したがって:

```text
1/4実行直後に2/4を続けて実行
```

とはしない。

基本:

```text
HOME
→ suspended air-entry
→ controlled touchdown
→ one semantic quarter file
→ STOP / inspect / record
```

別quarterは原則として別trialとしてHOMEからやり直す。

---

## 17. Final combined

risk-split full pathが実機PASSした後だけ使用する。

初回実機確認には使わない。

---

## 18. STOP criteria

次のいずれかで即STOP:

- unexpected direction
- unexpected axis motion
- sudden jump
- loss of support
- collision / floor penetration concern
- abnormal sound
- abnormal vibration
- excessive heat
- CAN error / repeated `0x0EE`
- StateMachine state mismatch
- operatorが挙動を説明できない

STOP後に原因を整理するまで次stageへ進まない。

---

## 19. Trial record

最低限記録する。

```text
Git commit SHA
candidate / staged file
axis / Use set
MCU Config確認値
CAN setup
ALIGN/HOME/RUN結果
publisher command
PASS / FAIL
異音・振動・温度・接触状態
STOP理由
```

Configを変更したtrialでは、変更前値、WRITE値、SAVE有無、power cycle後READ値も残す。
