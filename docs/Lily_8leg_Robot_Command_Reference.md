# Lily 8脚ロボット コマンド集

更新日: 2026-08-03  
対象リポジトリ:

```bash
~/Programs/PythonScripts/260522_lily_remake/lily_motion
```

## 0. このファイルの使い方

このコマンド集は、実施したい作業ごとに以下の順で整理する。

1. 目的
2. 使用するターミナル
3. 実行コマンド
4. 期待結果
5. 注意点

表記:

- **[確認済み]**: 会話中またはテスト結果で確認済み
- **[要実機確認]**: ソフト上は確認済みだが実機確認が必要
- **[要確認]**: 正式なファイル名・起動方法をリポジトリで再確認する必要あり

---

# 1. 共通操作

## 1.1 リポジトリへ移動

**[確認済み]**

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
```

現在位置確認:

```bash
pwd
```

Git状態確認:

```bash
git status --short
```

差分の形式確認:

```bash
git diff --check
```

---

# 2. 仮想CAN `vcan0` の準備

## 2.1 `vcan0`を作成・有効化

**[確認済み]**

```bash
sudo modprobe vcan

ip link show vcan0 >/dev/null 2>&1 || \
  sudo ip link add dev vcan0 type vcan

sudo ip link set up vcan0
```

状態確認:

```bash
ip -details link show vcan0
```

期待結果:

- `vcan0`が存在する
- 状態が`UP`

## 2.2 CANログを表示・保存

**[確認済み]**

別ターミナルで実行:

```bash
candump -L vcan0 | tee /tmp/lily_vcan.log
```

ログ確認:

```bash
less /tmp/lily_vcan.log
```

## 2.3 `vcan0`を削除

```bash
sudo ip link delete vcan0
```

---

# 3. 複数アクチュエータ・エミュレータ

## 3.1 3軸をすべて正常動作させる

**[確認済み]**

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion

python tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12
```

期待heartbeat:

```text
0FF [8] 0A 00 00 00 00 00 00 00
0FF [8] 0B 00 00 00 00 00 00 00
0FF [8] 0C 00 00 00 00 00 00 00
```

## 3.2 axis11だけ最初のALIGNを失敗させる

**[確認済み]**

```bash
python tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --align-fail-once 11
```

期待動作:

1. axis10、12は1回目でALIGN成功
2. axis11は`0x0EE`を送信
3. axis11だけ仮想リセット
4. axis11だけheartbeat再開
5. 再ALIGNで`0x00B`だけ送信
6. axis11が2回目で成功

## 3.3 24軸を起動する

**[確認済み: FakeBus統合]**  
**[要SocketCAN手動確認]**

```bash
python tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 0-23
```

## 3.4 軸番号を混合指定する

```bash
python tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 0-2,10,12-15,23
```

## 3.5 ALIGNを常に失敗させる

```bash
python tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --align-fail-always 12
```

## 3.6 指定したALIGN試行だけ失敗させる

```bash
python tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --align-fail-at 11:2
```

## 3.7 任意の`0x0EE`エラーを注入する

例: axis12、errorID=2

```bash
python tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --inject-error 12:2
```

## 3.8 RUN後に仮想MCUをリセットする

例: axis10がRUNに入った2秒後

```bash
python tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --reset-after-run 10:2.0
```

期待動作:

```text
run
→ start
→ aliment_standby
→ 0x0FF heartbeat再開
```

PC側では対象軸のAligned、Homed、RUNセッションが無効化される。

---

# 4. PC側CAN StateMachine

## 4.1 `vcan0`へ接続する

**[確認済み]**

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion

python tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel vcan0 \
  --can-bitrate 500000
```

注意:

- `--can-channel vcan0`を必ず指定する
- 省略時の既定値が`can0`の場合、実機CANへ接続する可能性がある
- `vcan`では実ビットレートはないが、既存引数として`500000`を指定してよい

## 4.2 実機`can0`へ接続する

**[要実機確認・安全確認]**

```bash
python tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

実行前確認:

```bash
ip -details link show can0
```

---

# 5. CAN UI

## 5.1 UIを起動する

**[要確認: 現行の正式起動コマンド]**

候補ファイル:

```text
tools/can_interface/initUI/ui.py
```

候補コマンド:

```bash
python tools/can_interface/initUI/ui.py
```

注意:

- 正式なUI起動方法は現行READMEまたは実際に使用しているコマンドで確定する
- UIはCAN IDや安全条件を独自生成せず、StateMachineへ要求する

## 5.2 基本操作順

```text
Connected確認
→ Use設定
→ ALIGN
→ HOME
→ Diagnostic RUN または RUN ALL AXES
→ 外部指令／動作確認
→ STOP
```

### Connectedの意味

- `aliment_standby`中のアクチュエータから`0x0FF`を受信済み
- ALIGN後にheartbeatが停止しても切断扱いしない

### Useの意味

- PC側で初期化・診断対象に含めるかを示す
- 初回Connected時は自動ON
- 使用者が手動でOFFにした後はheartbeatでONへ戻さない

---

# 6. 外部publisherによる単軸診断

## 6.1 正方向へ小振幅往復

**[確認済み: FakeBus]**  
**[要実機axis10確認]**

例: axis11

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion

python tools/can_interface/publish_single_axis_external_test.py \
  --axis 11 \
  --direction plus
```

例: 実機axis10

```bash
python tools/can_interface/publish_single_axis_external_test.py \
  --axis 10 \
  --direction plus
```

## 6.2 負方向へ小振幅往復

```bash
python tools/can_interface/publish_single_axis_external_test.py \
  --axis 10 \
  --direction minus
```

標準指令:

- 0.005 rad刻み
- 最大0.020 rad
- 100 ms周期
- 最終的にDiagnostic RUN開始時のq0へ復帰

期待CAN ID（axis10）:

```text
Diagnostic RUN: 0x60A
Position:       0x40A
```

動作確認中に新しい`0x00A`が送信されてはいけない。

---

# 7. 外部単軸指令のROS入力形式

**[確認済み]**

Topic:

```text
/can/axis_command
```

Message:

```text
std_msgs/String
```

受付形式:

```text
diagnostic_run:<axis>
position:<axis>:<absolute_rad>
position_offset:<axis>:<diagnostic_run時q0からのoffset_rad>
stop
```

例:

```bash
rostopic pub -1 /can/axis_command std_msgs/String \
  "data: 'diagnostic_run:10'"
```

```bash
rostopic pub -1 /can/axis_command std_msgs/String \
  "data: 'position_offset:10:0.005'"
```

```bash
rostopic pub -1 /can/axis_command std_msgs/String \
  "data: 'stop'"
```

注意:

- PositionはUse、Connected、Aligned、Homed、RUN送信済み、q0既知などの安全条件をStateMachineで確認する
- `rostopic pub`による単発指令は診断用
- 回転動作の正式経路には使用しない

---

# 8. CANエミュレータ標準試験手順

## 8.1 使用ターミナル

| Terminal | 用途 |
|---|---|
| 1 | `vcan0`作成 |
| 2 | `candump` |
| 3 | エミュレータ |
| 4 | StateMachine |
| 5 | UIまたは外部publisher |

## 8.2 最小手順

Terminal 1:

```bash
sudo modprobe vcan
ip link show vcan0 >/dev/null 2>&1 || sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Terminal 2:

```bash
candump -L vcan0 | tee /tmp/lily_vcan.log
```

Terminal 3:

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion

python tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --align-fail-once 11
```

Terminal 4:

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion

python tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel vcan0 \
  --can-bitrate 500000
```

Terminal 5:

```text
UI起動
→ Connected確認
→ Use確認
→ ALIGN
→ axis11だけ再ALIGN
→ HOME
→ axis11 Diagnostic RUN
```

外部publisher:

```bash
python tools/can_interface/publish_single_axis_external_test.py \
  --axis 11 \
  --direction plus
```

---

# 9. Gazebo: candidate_02の再生とeffort評価

## 9.1 rate=5で再生

**[確認済み]**

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion

python run_v3_0_42e_effort_replay_plot.py \
  --command-log testdata/v3_0_42c_candidates/candidate_02_x8_sw40_commands.jsonl \
  --strict-command-log-input \
  --rate 5 \
  --hold-start-sec 2.0 \
  --hold-end-sec 2.0 \
  --diagnose-command-log \
  --effort-limit 40 \
  --output testdata/v3_0_42e_effort_candidate_02_rate5.json \
  --plot-dir testdata/v3_0_42e_effort_candidate_02_rate5_plots
```

確認済み結果:

- rate=5で正常動作
- Gazebo上で再生完了

## 9.2 rate=15で再生

**[確認済み]**

```bash
python run_v3_0_42e_effort_replay_plot.py \
  --command-log testdata/v3_0_42c_candidates/candidate_02_x8_sw40_commands.jsonl \
  --strict-command-log-input \
  --rate 15 \
  --hold-start-sec 2.0 \
  --hold-end-sec 2.0 \
  --diagnose-command-log \
  --effort-limit 40 \
  --output testdata/v3_0_42e_effort_candidate_02_rate15.json \
  --plot-dir testdata/v3_0_42e_effort_candidate_02_rate15_plots
```

確認済み結果:

- rate=15でも正常動作

---

# 10. Gazebo: candidate_022_wideの全回転確認

**[確認済み: コマンド存在]**  
**[要パラメータ内容確認]**

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion

bash testdata/visual_near_contact_local_fix_candidate/candidate_022_wide/gazebo_review_global/run_gazebo_review_global.sh \
  full_roll \
  normal
```

注意:

- `normal`が速度・プロファイル・姿勢条件のどれを表すかはスクリプト内容で確認する
- `legacy_body_z=0.35`を使っているかは、この引数だけでは断定できない

設定確認:

```bash
grep -RniE \
  'legacy_body_z|body_z|spawn.*z|0\.35|normal|full_roll' \
  testdata/visual_near_contact_local_fix_candidate/candidate_022_wide/gazebo_review_global
```

スクリプト確認:

```bash
sed -n '1,240p' \
  testdata/visual_near_contact_local_fix_candidate/candidate_022_wide/gazebo_review_global/run_gazebo_review_global.sh
```

実際の展開コマンド確認:

```bash
bash -x \
  testdata/visual_near_contact_local_fix_candidate/candidate_022_wide/gazebo_review_global/run_gazebo_review_global.sh \
  full_roll \
  normal
```

---

# 11. Gazebo／回転候補ファイルを探す

## 11.1 再生スクリプトを探す

```bash
find . -type f \
  \( -name '*gazebo*' -o -name '*replay*' -o -name 'run_v3_0_*.py' \) \
  | sort
```

## 11.2 candidate関連ファイルを探す

```bash
find testdata -type f \
  | grep -E 'candidate_02|candidate_022|softlimit|commands\.jsonl' \
  | sort
```

## 11.3 `run_v3_0_42e_effort_replay_plot.py`を探す

```bash
find ~/Programs/PythonScripts/260522_lily_remake \
  -name 'run_v3_0_42e_effort_replay_plot.py' \
  -print
```

---

# 12. 第二関節角度・softlimit関連

## 12.1 candidate_02評価済み情報

確認済み値:

```text
最大第2関節角度: 約95.9743°
95°超過量:       約0.9743°
```

## 12.2 softlimit後処理

**[要確認: 現行の正式入力・出力ファイル名]**

既知スクリプト候補:

```text
run_v3_0_42c_candidate02_second_joint_softlimit.py
```

関連ファイル検索:

```bash
find . -type f \
  | grep -E 'candidate02.*softlimit|second_joint_softlimit' \
  | sort
```

ヘルプ確認:

```bash
python run_v3_0_42c_candidate02_second_joint_softlimit.py --help
```

確認済み結果:

```text
max_before:      約95.9743°
max_after:       94.8°
violation_count: 198 → 0
```

---

# 13. CAN関連テスト

## 13.1 Diagnostic RUNテスト

```bash
pytest -q tests/test_can_diagnostic_run.py
```

過去結果:

```text
10/10 PASS
```

## 13.2 外部単軸指令テスト

```bash
pytest -q tests/test_can_external_axis_command.py
```

過去結果:

```text
9/9 PASS
```

## 13.3 エミュレータテスト

```bash
pytest -q \
  tests/test_can_multi_actuator_emulator.py \
  tests/test_can_emulator_integration.py
```

過去結果:

```text
18/18 PASS
```

## 13.4 CAN関連全テスト

**[要確認: 現行のテスト選択式]**

候補:

```bash
pytest -q tests -k 'can'
```

過去結果:

```text
81/81 PASS
```

## 13.5 リポジトリ全体

```bash
pytest -q
```

過去結果:

```text
132件中121件PASS
11件はnumpy、sympy、matplotlib不足による既存依存エラー
CAN関連失敗: 0
```

---

# 14. Python構文・生成物・差分確認

## 14.1 Git差分形式

```bash
git diff --check
```

## 14.2 `__pycache__`と`.pyc`確認

```bash
find . \
  \( -type d -name '__pycache__' -o -type f -name '*.pyc' \) \
  -print
```

削除する場合:

```bash
find . -type d -name '__pycache__' -prune -exec rm -rf {} +
find . -type f -name '*.pyc' -delete
```

## 14.3 主要ファイルの構文確認

Python 3:

```bash
python3 -m py_compile \
  tools/can_interface/emulator/virtual_actuator.py \
  tools/can_interface/emulator/multi_actuator_emulator.py \
  tools/can_interface/emulator/scenario.py \
  tools/can_interface/statemachine/state_machine.py
```

Python 2.7環境を使用する場合:

```bash
python2 -m py_compile \
  tools/can_interface/emulator/virtual_actuator.py \
  tools/can_interface/emulator/multi_actuator_emulator.py \
  tools/can_interface/emulator/scenario.py \
  tools/can_interface/statemachine/state_machine.py
```

---

# 15. 実機axis10の単軸外部試験

**[要実機確認]**

## 15.1 前提

- ロボットを浮かせる
- 外部位置publisherを停止
- axis10をUse=True
- 新規セッション
- Connected
- ALIGN
- HOME
- 通常の`RUN ALL AXES`を押さない
- UIのmotion check専用ボタンを押さない

## 15.2 CANログ

```bash
candump -L can0 | tee /tmp/axis10_external_test.log
```

## 15.3 外部publisher

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion

python tools/can_interface/publish_single_axis_external_test.py \
  --axis 10 \
  --direction plus
```

## 15.4 合格条件

```text
0x60AだけDiagnostic RUN
0x40AだけPosition
publisher開始後に新しい0x00Aなし
軸10だけが往復
最終的にq0へ復帰
0x0EEなし
pc_send_errorなし
can_interface_errorなし
```

---

# 16. ENOBUFS調査

既知エラー:

```text
[Errno 105] No buffer space available
```

発生状況:

- 全24軸RUN送信中
- 一部のRUN ID送信後に失敗
- 軸10はRUNへ進んだ可能性がある
- PC側は`pc_send_error`、`can_interface_error`をラッチ

ログ検索:

```bash
grep -RniE \
  'ENOBUFS|No buffer space|pc_send_error|can_interface_error' \
  . \
  --exclude-dir=.git
```

注意:

- `vcan0`では実CAN固有のENOBUFSを再現できない可能性がある
- 単軸診断とENOBUFS対策は分離して扱う
- 安全ゲートを解除して回避しない

---

# 17. 初期姿勢・回転指令の正式経路

## 17.1 既知の本番24軸経路

```text
JSONL
→ tools/publish_cmdforjetson_jsonl.py
→ /cmdForJetson
→ StateMachine
→ 0x400～0x417
```

## 17.2 publisherのヘルプ確認

**[要確認]**

```bash
python tools/publish_cmdforjetson_jsonl.py --help
```

ファイル検索:

```bash
find . -type f \
  -name 'publish_cmdforjetson_jsonl.py' \
  -print
```

## 17.3 初期姿勢指令

**[要確認: 正式コマンドと基準JSONL]**

この欄へ今後追加する内容:

- HOME `[0,0,0]`から開始姿勢へ遷移するJSONL
- 実行rate
- hold-start、hold-end
- 接地前後の条件
- 出力ログ
- 使用基準バージョン

## 17.4 回転指令

**[要確認: 正式コマンドと基準JSONL]**

現行基準情報:

```text
v3.0.36 RF-1 current-angle anchor
surface_sequence=1,5,6,2,1
move_dist=0.4
support_dist=0.7
legacy_body_z=0.35
max_step=30
resample_factor=8
smooth_window=40
```

この欄へ今後追加する内容:

- 正式な回転コマンド生成
- Gazebo再生
- 実機送信
- 初期姿勢最終値と回転先頭値の一致確認
- 使用候補ファイル
- STOP／異常時の操作

---

# 18. 標準作業フロー

## 18.1 CAN通信・状態機械だけ確認したい

```text
vcan0作成
→ candump
→ 複数軸エミュレータ
→ StateMachineをvcan0で起動
→ UI起動
→ Connected
→ Use
→ ALIGN
→ HOME
→ Diagnostic RUN
→ 外部単軸publisher
```

## 18.2 ALIGN失敗と再試行を確認したい

```bash
python tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --align-fail-once 11
```

```text
1回目ALIGN
→ axis10、12成功
→ axis11失敗・heartbeat復帰
→ 2回目ALIGN
→ axis11だけ成功
```

## 18.3 24軸CAN ID対応を確認したい

```bash
python tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 0-23
```

```text
全軸Connected
→ ALIGN
→ HOME
→ RUN
→ /cmdForJetson
→ 0x400～0x417確認
```

## 18.4 Gazeboでcandidate_02を確認したい

```bash
python run_v3_0_42e_effort_replay_plot.py \
  --command-log testdata/v3_0_42c_candidates/candidate_02_x8_sw40_commands.jsonl \
  --strict-command-log-input \
  --rate 5 \
  --hold-start-sec 2.0 \
  --hold-end-sec 2.0 \
  --diagnose-command-log \
  --effort-limit 40 \
  --output testdata/v3_0_42e_effort_candidate_02_rate5.json \
  --plot-dir testdata/v3_0_42e_effort_candidate_02_rate5_plots
```

## 18.5 candidate_022_wideの全回転を確認したい

```bash
bash testdata/visual_near_contact_local_fix_candidate/candidate_022_wide/gazebo_review_global/run_gazebo_review_global.sh \
  full_roll \
  normal
```

## 18.6 実機axis10を外部指令で動かしたい

```text
Connected
→ Use=True
→ ALIGN
→ HOME
→ UIのmotion checkボタンは押さない
```

```bash
python tools/can_interface/publish_single_axis_external_test.py \
  --axis 10 \
  --direction plus
```

---

# 19. 今後このファイルへ追加する項目

- 正式なUI起動コマンド
- 初期姿勢JSONLの正式ファイル名
- 初期姿勢publisherの正式コマンド
- 回転指令JSONLの正式ファイル名
- 実機回転publisherの正式コマンド
- candidate02 softlimit 94.8の正式入力・出力
- candidate_022_wideの`normal`の意味
- Gazebo body height 0.35 mの設定元
- ENOBUFS対策後の標準起動条件
- STOP・再起動の正式手順
- 使用コミット・タグ・基準データ

---

# 20. 安全上の注意

- `vcan0`試験では必ず`--can-channel vcan0`を明示する
- 実機試験前に`can0`／`vcan0`を取り違えていないことを確認する
- 実機単軸試験ではロボットを浮かせる
- 未Aligned、未Homed、Use=False軸へ位置指令を送らない
- `pc_send_error`や`can_interface_error`を手動で無効化しない
- 予期しないheartbeat受信後は再ALIGN、再HOMEする
- UI専用ボタンではなく、本番外部入力経路も確認する
- 回転動作へ進む前に初期姿勢最終値と回転先頭値の一致を確認する
