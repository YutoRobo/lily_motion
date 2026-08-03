# Lily 8脚ロボット ソフトウェア概要

更新日: 2026-08-04  
対象ブランチ: `master`

本リポジトリは、Lily 8脚ロボットの回転歩容生成、運動学、制約評価、Gazebo確認、ROS指令配信、CAN状態機械、操作UI、仮想アクチュエータ試験をまとめたソフトウェア一式である。

このREADMEはリポジトリ全体の入口であり、詳細な実行コマンド、安全手順、候補別の評価根拠は各専門文書を参照する。

---

## 1. ソフトウェアの対象範囲

本リポジトリの現行ソフトウェアは、主に次を扱う。

- 8脚・24関節の回転歩容生成
- 順運動学・逆運動学
- RF-1～RF-6を含む回転シーケンス
- 指令列の補間、再サンプリング、平滑化
- 関節角度、床クリアランス、脚間距離、ハウジング干渉などの評価
- Gazeboでの指令再生と目視確認
- JSONL指令列から`/cmdForJetson`へのROS配信
- UI操作、ALIGN、HOME、RUN、STOPを管理するStateMachine
- 24軸指令から各アクチュエータへのCANフレーム展開
- `vcan`上の複数仮想アクチュエータによる統合試験
- Python単体試験・回帰試験

本リポジトリだけでは、次は扱わない、または完全には再現しない。

- アクチュエータMCUの実ファームウェア本体
- モータ電流制御やサーボ制御器の内部処理
- 実機のトルク、負荷、温度、バックラッシ、機械変形
- 厳密なハードリアルタイム保証
- 実機24軸の同期応答を再現する物理シミュレーション

---

## 2. システム全体像

```text
歩容パラメータ／既存参照指令
        ↓
lily_motion_v3
  運動学・歩容生成・制約評価
        ↓
tools/command_generation
  JSONL指令生成・変換・再サンプリング
        ↓
24要素の関節指令列 [rad]
        ├──────────────→ tools/diagnostics
        │                  角度・床・脚間・連続性評価
        │
        ├──────────────→ tools/gazebo
        │                  Gazebo再生・姿勢確認
        │
        └→ tools/publish_cmdforjetson_jsonl.py
                    ↓
             /cmdForJetson
       sensor_msgs/JointState
          position: 24要素
                    ↓
tools/can_interface/statemachine/StateMachine
        ├─ /ui/leg_command
        ├─ Use=True安全ゲート
        ├─ ALIGN / HOME / RUN / STOP
        └─ 関節制限・セッション・エラー判定
                    ↓
         SocketCAN: can0 または vcan0
                    ↓
           0x400 + axis の位置指令
                    ↓
          24個のアクチュエータMCU
```

### 本番位置指令経路

本番の位置指令入力は`/cmdForJetson`に一本化されている。

```text
単軸試験／1脚試験／複数脚試験／回転歩容
                    ↓
/cmdForJetson（常に24要素）
                    ↓
StateMachine
                    ↓
Use=True軸だけへCAN RUN／POSITION送信
```

削除済みの`/can/axis_command`を本番位置指令経路として使用してはならない。

---

## 3. 現在の統合状況

### 3.1 回転歩容候補

現在の最新凍結済みpre-hardware候補は次である。

```text
data/reference_candidates/
  v3_0_44_candidate_022_wide_urdf0p075/
```

主な状態:

- coxa長: `0.075 m`
- thigh長: `0.300 m`
- tibia長: `0.300 m`
- 指令数: `2233`
- 第2関節最大角度: `94.8 deg`
- 第2関節95度超過: `0`
- Gazebo full roll: `PASS`
- URDF FK評価を0.075 m形状判断の基準とする
- 実機full roll: 未確認

実機で最初に使う候補別ファイルと段階順序は、次を参照する。

- [`data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md`](data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md)

### 3.2 CAN・ROS経路

2026-08-04時点で確認済みの範囲:

- `/cmdForJetson`を唯一の本番位置指令経路として統合
- `JointState.position`は常に24要素
- RUNとPOSITIONは`Use=True`軸だけへ送信
- axis10単軸の`vcan`往復試験: `PASS`
- axis10、11、12への複数軸ファンアウト: `PASS`
- CAN関連テスト: `81/81 PASS`
- Python 2.7構文確認: `PASS`
- Python 3構文確認: `PASS`
- 実機axis10の`+0.002 rad`往復: 目視上問題なし

未確認または今後の確認対象:

- 実機axis10の負方向小振幅
- 実機axis10の`±0.005 rad`
- 同一脚3軸の実機動作
- 複数実アクチュエータの同時応答
- 実機上のair-entry、touchdown、分割roll、full roll
- Jetson Orin上での長時間周期・CPU負荷・ジッタ測定

---

## 4. ロボットと関節のソフトウェア表現

### 4.1 軸数

```text
8脚 × 3自由度 = 24軸
```

各脚は次の3関節で構成される。

| 関節内番号 | 名称 | ソフト制限 |
|---:|---|---:|
| 0 | base / yaw | ±360 deg |
| 1 | thigh / pitch | ±95 deg |
| 2 | tibia / pitch | ±150 deg |

軸番号は基本的に次で対応する。

```text
axis = 3 × leg_index + joint_index
```

例:

```text
axis9  = 第4脚 base
axis10 = 第4脚 thigh
axis11 = 第4脚 tibia
```

### 4.2 単位

- ROS位置指令: `rad`
- JSONL関節指令: `rad`
- CAN位置payload: little-endian `float32(rad)`
- HOME後の論理初期値: 原則`0.0 rad`

### 4.3 現行リンク長

[`lily_motion_v3/robot_geometry.py`](lily_motion_v3/robot_geometry.py)を共有形状定数の基準とする。

```text
coxa  = 0.075 m
thigh = 0.300 m
tibia = 0.300 m
```

---

## 5. 現行コードと非現行コード

### 現行の実行対象

- `lily_motion_v3/`
- `tools/command_generation/`
- `tools/diagnostics/`
- `tools/gazebo/`
- `tools/can/`
- `tools/can_interface/`
- `tools/publish_cmdforjetson_jsonl.py`
- `tools/publish_cmdforjetson_single_axis_test.py`
- `data/reference_candidates/`
- `tests/`

### 参照・履歴用途

- `archive/`
  - 過去の実験、旧候補、旧スイープ、再現用スクリプト
  - 現行の実機操作には使用しない
- `external/`
  - 外部由来または移設前のスナップショット
  - `external/can_interface`は現行実行対象ではない
- `testdata/`
  - 生成結果、評価結果、試験証跡、比較用データ
  - すべてが正式基準とは限らない

正式な参照候補は`data/reference_candidates/`に置く。参照候補の指令列は原則として直接編集しない。

---

## 6. ディレクトリ構成

```text
lily_motion/
├── README.md                         この文書
├── README_V3_CORE.md                 v3-coreの評価・生成入口
├── lily_motion_v3/                   回転歩容コア、運動学、評価器
├── tools/
│   ├── command_generation/           指令生成・変換・再サンプリング
│   ├── diagnostics/                  制約評価・解析・可視化
│   ├── gazebo/                       Gazebo再生・姿勢確認
│   ├── can/                          CAN dry-run・変換確認
│   ├── can_interface/
│   │   ├── statemachine/             UI／ROS／CAN状態機械
│   │   ├── initUI/                   操作UI
│   │   └── emulator/                 vcan仮想アクチュエータ
│   ├── publish_cmdforjetson_jsonl.py
│   └── publish_cmdforjetson_single_axis_test.py
├── data/reference_candidates/        凍結済み正式候補
├── testdata/                          評価結果・試験用データ
├── tests/                             Python回帰試験
├── docs/                              運用手順・コマンド集・技術記録
├── archive/                           過去検討・非現行スクリプト
└── external/                          外部／旧スナップショット
```

---

## 7. 各ソフトウェア層

### 7.1 `lily_motion_v3`: 回転歩容コア

旧`LilyRobot`、隠れた旧IK、xacroロードへ依存しない、プロジェクト内完結型のv3コアである。

主な責務:

- ロボット形状定義
- FK／IK
- 接触計画
- RF位相管理
- 回転姿勢生成
- legacy-style／reference軌道との比較
- 指令列の連続化
- 制約評価用データ生成

詳細:

- [`README_V3_CORE.md`](README_V3_CORE.md)

### 7.2 `tools/command_generation`: 指令生成

主な用途:

- v3-native候補のJSONL出力
- legacy/reference指令の取込み
- 指令列の再サンプリング
- command sourceのraw／filtered切替
- 実行可能な24軸指令列への変換

代表入口:

```bash
python tools/command_generation/run_v3_0_export_commands.py --help
python tools/command_generation/run_v3_0_import_legacy_reference.py --help
python tools/command_generation/run_v3_0_resample_commands.py --help
```

### 7.3 `tools/diagnostics`: 評価・解析

代表的な評価対象:

- IK失敗
- 関節角度制限
- 隣接フレーム角度差
- 第2関節床クリアランス
- 脚先床貫通
- 脚間カプセル距離
- ハウジング干渉
- 面切替・繰返しroll境界
- 候補パラメータスイープ

代表入口:

```bash
python tools/diagnostics/run_v3_0_whole_roll_eval.py --summary-only
python tools/diagnostics/run_v3_0_diagnose_failures.py
python tools/diagnostics/run_v3_0_command_diagnostics.py --help
python tools/diagnostics/run_v3_0_visualize_roll.py --help
```

### 7.4 `tools/gazebo`: Gazebo確認

JSONL指令列をGazeboへ再生し、動作、姿勢、関節状態、必要に応じてeffortやリンク状態を確認する。

代表入口:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py --help
python tools/gazebo/run_v3_0_gazebo_touchdown_pose_check.py --help
```

Gazebo確認は実機安全性を保証しない。特に接触、トルク、バックラッシ、配線、実減速機の挙動は別途実機確認が必要である。

### 7.5 ROS指令publisher

#### JSONL再生

```text
tools/publish_cmdforjetson_jsonl.py
```

- JSONLから24要素位置指令を読む
- `/cmdForJetson`へ`JointState`を配信する
- CANを直接開かない
- `--start-index`と`--max-frames`で分割試験が可能

#### 単軸試験

```text
tools/publish_cmdforjetson_single_axis_test.py
```

- 24要素`JointState`を配信する
- 対象軸だけ有限値
- 対象外23軸は`NaN`
- 正方向または負方向へ往復し、中心へ戻る
- ALIGN、HOME、RUN、STOPは発行しない
- CANを直接開かない

詳細コマンド:

- [`docs/Lily_8leg_Robot_Command_Reference.md`](docs/Lily_8leg_Robot_Command_Reference.md)

### 7.6 `tools/can_interface`: UI・StateMachine・CAN

現行のCAN実行対象はここだけである。

```text
tools/can_interface/statemachine/main.py
tools/can_interface/initUI/ui.py
```

StateMachineの責務:

- アクチュエータ接続状態管理
- Use=True軸管理
- ALIGN進行・再試行管理
- HOME jog／SET HOME
- RUN安全ゲート
- STOP
- `/cmdForJetson`の24要素検証
- 対象関節の角度制限確認
- CAN IDとpayload生成
- MCUエラーのラッチ
- CAN送信失敗時の停止処理

詳細:

- [`tools/can_interface/README.md`](tools/can_interface/README.md)

### 7.7 `tools/can_interface/emulator`: 仮想アクチュエータ

`vcan`専用の複数アクチュエータMCUエミュレータである。

再現対象:

- standby heartbeat
- ALIGN成功／失敗
- HOME完了
- RUN遷移
- position受信
- 初期化エラー注入
- RUN後リセット

再現しない対象:

- 実トルク
- 機械動作
- 実位置フィードバック
- 電流、温度、振動

詳細:

- [`tools/can_interface/emulator/README.md`](tools/can_interface/emulator/README.md)

---

## 8. ROSインターフェース

### 8.1 本番位置指令

| 項目 | 値 |
|---|---|
| Topic | `/cmdForJetson` |
| Message | `sensor_msgs/JointState` |
| 使用フィールド | `position` |
| 要素数 | 常に24 |
| 単位 | rad |

StateMachineは`Use=True`軸だけを検証し、CANへ展開する。

単軸試験では対象外軸を`NaN`にする。意図しない別軸が`Use=True`になっていた場合、その軸の`NaN`によってフレーム全体をCAN送信前に拒否する。

### 8.2 UI要求

| Topic | Message | 用途 |
|---|---|---|
| `/ui/leg_command` | `std_msgs/String` | Use、ALIGN、HOME、RUN、STOP |
| `/ui/leg_status` | `std_msgs/String` | 軸状態通知 |
| `/ui/leg_use_status` | `std_msgs/String` | Use状態通知 |
| `/ui/motion_check_status` | `std_msgs/String` | motion check状態 |
| `/ui/diagnostic_status` | `std_msgs/String` | 診断状態 |

CAN ID、payload、安全ゲート、論理原点はUIではなくStateMachineが所有する。

---

## 9. CAN仕様

### 9.1 CAN ID

| 処理 | CAN ID |
|---|---:|
| ALIGN request TX | `0x000 + axis` |
| ALIGN result RX | `0x100 + axis` |
| HOME jog TX | `0x200 + axis` |
| SET HOME TX | `0x300 + axis` |
| POSITION TX | `0x400 + axis` |
| RUN start TX | `0x600 + axis` |
| standby heartbeat RX | `0x0FF` |
| error RX | `0x0EE` |

axis10の例:

```text
ALIGN      0x00A
ALIGN結果  0x10A
SET HOME   0x30A
POSITION   0x40A
RUN        0x60A
```

RUN開始後の位置指令中に`0x40A`だけが繰り返し出ることは正常である。`0x60A`はRUN開始時に送るもので、各位置指令周期には再送しない。

### 9.2 POSITION payload

```text
[0, 0, 0, 0] + little-endian float32(position_rad)
```

### 9.3 Use=True

- ALIGN、HOME、RUN、POSITIONの対象
- RUN開始時にconnected／aligned／homedの安全条件を確認
- `/cmdForJetson`受信時に角度と数値を検証

### 9.4 Use=False

- RUN成立条件から除外
- POSITION CANを送信しない
- 非接続でも他のUse=True軸のRUNを妨げない

RUNはUse=True軸が0本の場合に拒否される。STOPは全体停止要求として扱い、`is_run=False`にする。

---

## 10. 最小vcan確認手順

実機CANを使わず、PC経路を確認する例である。

### Terminal 1: ROS

```bash
roscore
```

### Terminal 2: vcan

```bash
sudo modprobe vcan
ip link show vcan0 >/dev/null 2>&1 || sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
candump -L vcan0
```

### Terminal 3: 仮想アクチュエータ

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12
```

### Terminal 4: StateMachine

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel vcan0 \
  --can-bitrate 500000
```

### Terminal 5: UI

```bash
python2 tools/can_interface/initUI/ui.py
```

操作順:

```text
Connected確認
→ 対象軸だけUse=True
→ ALIGN
→ SET HOME
→ RUN
→ /cmdForJetson publisher
→ STOP
```

完全な手順と期待CANログは次を参照する。

- [`docs/Lily_8leg_Robot_Command_Reference.md`](docs/Lily_8leg_Robot_Command_Reference.md)

---

## 11. 参照候補とデータ管理

### 11.1 最新pre-hardware候補

```text
data/reference_candidates/
  v3_0_44_candidate_022_wide_urdf0p075/
```

主なファイル:

```text
manifest.json
summary.json
pre_hardware_decision.md
commands.jsonl
staged/air_entry_and_hold_only_commands.jsonl
staged/combined_with_hold_commands.jsonl
staged/roll_0_50_commands.jsonl
staged/roll_50_100_commands.jsonl
staged/roll_100_300_commands.jsonl
staged/roll_300_end_commands.jsonl
```

`commands.jsonl`や終盤rollを最初の実機指令として使用してはならない。

### 11.2 旧基準・比較候補

候補評価の履歴、旧形状、旧softlimit、旧パラメータスイープは、比較と再現のために残している。

旧候補が存在しても、現行実機試験で自動的に採用してよいことを意味しない。候補ディレクトリ内の`manifest`、`summary`、decision文書を確認する。

### 11.3 `testdata`

`testdata/`は次を含む。

- 生成途中の候補
- 診断JSON
- プロット
- Gazeboログ
- CAN dry-runログ
- 制約比較
- 実機段階試験用の一時ファイル

正式候補へ昇格したデータは`data/reference_candidates/`へ凍結する。

---

## 12. 試験体系

### 12.1 Pythonテスト

```bash
pytest -q tests
```

CAN系だけを確認する例:

```bash
pytest -q tests -k 'can'
```

代表的なCANテスト:

```text
tests/test_can_cmdforjetson_unified_path.py
tests/test_can_diagnostic_run.py
tests/test_can_emulator_integration.py
tests/test_can_legacy_alignment_retry.py
tests/test_can_multi_actuator_emulator.py
```

代表的な歩容・運動学テスト:

```text
tests/test_v3_0_kinematics.py
tests/test_v3_0_command_resampler.py
tests/test_v3_0_repeated_roll_connection.py
tests/test_v3_0_second_joint_clearance_eval.py
tests/test_v3_0_39_inter_leg_capsule_eval.py
```

### 12.2 検証レベル

| レベル | 内容 | 現在の状態 |
|---|---|---|
| 単体試験 | 運動学、補間、制約、CAN変換 | 多数実装済み |
| FakeBus | StateMachine安全ゲート、CAN ID、payload | PASS |
| vcan単軸 | axis10 RUN／POSITION往復 | PASS |
| vcan複数軸 | axis10～12ファンアウト | PASS |
| mock end-to-end | HOME姿勢から候補指令、CAN変換 | PASS |
| Gazebo full roll | v3.0.44 candidate_022_wide | PASS |
| 実機単軸 | axis10 `+0.002 rad` | 暫定PASS |
| 実機1脚 | 3軸 | 未確認 |
| 実機複数脚 | 同時動作 | 未確認 |
| 実機full roll | 8脚回転 | 未確認 |

---

## 13. 実機試験の基本順序

新しい回転歩容系と、以前に実機確認された旧6脚歩行系を混同しない。

### 13.1 既知正常系

機体Jetson上に残る旧CANプログラムと旧6脚歩行アルゴリズムは、過去に実機確認された既知正常系として扱う。

確認順:

```text
旧CAN＋旧6脚歩行
→ 1軸
→ 1脚
→ 6脚
```

### 13.2 現行master系

本リポジトリ`master`のCAN統合と回転歩容は、既知正常系とは別ソフトとして段階確認する。

推奨順:

```text
vcan単軸
→ 実機単軸 ±0.002 rad
→ 実機単軸 ±0.005 rad
→ 同一脚3軸
→ air-entry + hold
→ touchdown確認
→ roll 0～50
→ roll 50～100
→ roll 100～300
→ roll 300～end
→ full sequence
```

各段階で異音、衝撃、別軸動作、原点未復帰、CAN異常、接触異常があればSTOPし、次段階へ進まない。

---

## 14. 実行環境

主な開発・評価環境:

- Ubuntu 18.04
- ROS Melodic
- Gazebo
- Python 2.7
- SocketCAN
- `can-utils`
- `python-can`
- pytest

CAN関連コードはPython 2.7とPython 3で構文確認している。ただし、ROS Melodic環境の`rospy`や依存パッケージを含む実運用は、現在のPython 2.7環境を基準としている。

数値評価・可視化では、スクリプトに応じて次を使用する。

- NumPy
- SymPy
- Matplotlib

Jetson Orinは実行対象候補である。StateMachineの処理量は小さいが、Jetson上の正式な周期、CPU使用率、温度、スケジューリングジッタは実測して判断する。

---

## 15. 性能とリアルタイム性

StateMachineの位置変換は、24要素確認、Use=True抽出、float32変換、CAN送信が中心であり、計算負荷は比較的小さい。

確認時の参考値:

- `vcan`で3軸POSITIONファンアウト: 約`57 us`
- 単純比例した24軸ソフト展開: 1 ms未満の可能性が高い

ただし`vcan`には実CANのビット転送時間がないため、正式な性能保証値ではない。

500 kbpsの実CANでは、24本の8-byte標準フレームを順次送る通信時間が先に支配的になる。24軸の先頭と末尾には数ms規模の到着差が生じ得る。

現在のPython／通常Linux構成はハードリアルタイムではない。周期やジッタが問題になった場合は、次の順で対応する。

1. Jetson上でプロファイリング
2. 軌道生成とCAN送信を分離測定
3. 重い数値関数をCython／Numba化
4. StateMachineまたは送信部をC++化
5. 必要に応じてPREEMPT_RT、CPU固定、優先度設定を検討

C++化してもCAN物理帯域そのものは増えない。

---

## 16. 安全上の原則

- dry-runや`vcan`試験で`can0`を開かない
- 実機試験前に`can0`／`vcan0`を目視確認する
- 実機単軸試験では対象脚を浮かせる
- 対象外軸は`Use=False`にする
- 非常停止を即時操作できる状態にする
- 可動範囲へ人を入れない
- ALIGN、HOME方向、SET HOME姿勢を確認してからRUNする
- full rollから開始しない
- `archive/`と`external/can_interface/`を現行実機経路として実行しない
- `data/reference_candidates/`の指令を直接編集しない
- 予期しないheartbeat、`0x0EE`、CAN送信失敗、姿勢跳躍があれば中止する
- 試験終了後は必ずSTOPする

---

## 17. 文書一覧

### 全体

- [`README.md`](README.md): ソフト全体像と現在状態
- [`README_V3_CORE.md`](README_V3_CORE.md): v3-coreの生成・評価

### 実行・コマンド

- [`docs/Lily_8leg_Robot_Command_Reference.md`](docs/Lily_8leg_Robot_Command_Reference.md): 実行コマンド集
- [`tools/can_interface/README.md`](tools/can_interface/README.md): CAN UI／StateMachine
- [`tools/can_interface/emulator/README.md`](tools/can_interface/emulator/README.md): vcan仮想アクチュエータ

### 実機手順

- [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md): 実機操作手順
- [`docs/HARDWARE_PRETEST_STATUS.md`](docs/HARDWARE_PRETEST_STATUS.md): mock／pretest状況
- [`data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md`](data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md): 最新候補の実機段階判断

### 形状

- [`docs/kinematics_link_length_update_0p075.md`](docs/kinematics_link_length_update_0p075.md): coxa長0.075 mへの更新記録
- [`lily_motion_v3/robot_geometry.py`](lily_motion_v3/robot_geometry.py): 現行共有形状定数

### 履歴

- [`archive/README.md`](archive/README.md): archive方針

文書の日付が異なる場合、候補固有の最新decisionと、更新日の新しい文書の状態記述を優先する。古い文書の安全原則は、明示的に撤回されない限り継続して適用する。

---

## 18. 開発時の基本ルール

- `master`は統合済みの現行基準とする
- 実験変更はブランチで分離する
- 正式候補はmanifest、summary、checksum、評価根拠を残す
- 実行スクリプトと評価スクリプトを混同しない
- 本番位置指令経路を増やさない
- CAN IDや安全判定をUIや個別publisherへ複製しない
- StateMachineをCAN安全判定の単一責任箇所とする
- `git diff --check`と関連テストを通してから統合する
- 実機試験結果は、使用コミット、軸、振幅、周期、CANログ、目視結果とともに記録する

---

## 19. 現在の次段階

2026-08-04時点の次の実機段階は次である。

```text
axis10 負方向 ±0.002 rad確認
→ axis10 正負 ±0.005 rad確認
→ 同一脚3軸確認
```

その後、最新pre-hardware候補のstagedログを用いてair-entry以降へ進む。
