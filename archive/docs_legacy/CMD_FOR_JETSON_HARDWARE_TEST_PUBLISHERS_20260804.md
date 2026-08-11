# `/cmdForJetson` Hardware Test Publishers

更新日: 2026-08-04

この文書は、現行の本番位置指令経路を使い、段階的に実機確認するためのpublisherを説明する。

```text
publisher
→ /cmdForJetson
→ StateMachine
→ Use=True軸だけ
→ CAN RUN / POSITION
```

各publisherはSocketCANを直接開かず、ALIGN、HOME、RUN、STOPも送信しない。これらは通常のUIまたは`/ui/leg_command`から実施する。

---

## 1. 追加されたpublisher

### 1.1 1脚3軸小振幅試験

```text
tools/publish_cmdforjetson_one_leg_test.py
```

用途:

- 1脚の3軸をすべて有限値にした24要素`JointState`を送る
- 3軸を1軸ずつ順番に往復させる
- 3軸を同時に小振幅往復させる
- 3軸`Use=True`時にNaNで拒否されない正しい1脚試験経路を確認する

### 1.2 24軸ログの論理1軸を実機1軸へ割り当てる

```text
tools/publish_cmdforjetson_mapped_axis_replay.py
```

用途:

- 24軸JSONLから任意の論理軸を抽出する
- 初回値からの相対変位へ変換する
- 倍率を掛ける
- 小振幅上限内へ制限する
- 実機の1軸へ安全マスク付きで送る
- 1台の実機アクチュエータを使い、論理axis0～23の波形を1本ずつ確認する

これは絶対姿勢再現ではなく、指令波形、周期、ROS、StateMachine、CAN変換の確認である。

---

## 2. 共通安全条件

- 機体または対象脚を浮かせる
- 非常停止またはUI STOPを直ちに操作できる状態にする
- 可動範囲へ人を入れない
- 対象外軸を`Use=False`にする
- ALIGN、HOME方向、SET HOME姿勢を確認する
- 通常RUN成立後にpublisherを起動する
- `candump`でCAN IDを監視する
- 異音、衝撃、別軸動作、原点未復帰、`0x0EE`、送信エラー時は中止する
- 試験終了後は必ずSTOPする

publisherは安全停止を代行しない。

---

## 3. 第4脚の軸番号

この文書の例では`leg-index=3`を使用する。

```text
axis9  = base
axis10 = thigh
axis11 = tibia
```

軸番号:

```text
axis = 3 × leg_index + joint_index
```

`leg-index`は0始まりである。

---

## 4. 1脚3軸試験

### 4.1 前提

1脚3軸publisherを使う前に、axis9、10、11をそれぞれ単軸publisherで確認する。

単軸確認中は対象1軸だけを`Use=True`にする。

1脚3軸publisherの実行時は、次だけを`Use=True`にする。

```text
Use=True:  axis9, axis10, axis11
Use=False: その他21軸
```

送信される24要素のうち、axis9、10、11は常に有限値であり、その他はNaNである。

### 4.2 3軸を1軸ずつ正方向へ動かす

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode individual \
  --direction plus \
  --centers-rad 0,0,0 \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500 \
  --start-hold-sec 1.000 \
  --peak-hold-sec 1.000 \
  --between-motion-hold-sec 1.000 \
  --end-hold-sec 1.000
```

順序:

```text
[0,0,0]
→ axis9だけ 0 → +0.002 → 0
→ axis10だけ 0 → +0.002 → 0
→ axis11だけ 0 → +0.002 → 0
→ [0,0,0]
```

他の2軸も毎フレーム有限な中心値`0.0 rad`を送る。

### 4.3 3軸を1軸ずつ負方向へ動かす

正方向が正常だった場合だけ実行する。

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode individual \
  --direction minus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

### 4.4 3軸協調小振幅

個別正負方向が正常だった場合だけ実行する。

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode coordinated \
  --direction plus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

指令:

```text
[0,0,0]
→ [+0.001,+0.001,+0.001]
→ [+0.002,+0.002,+0.002]
→ [+0.001,+0.001,+0.001]
→ [0,0,0]
```

この動作は各関節の論理正方向を同時に与えるものであり、ロボット空間の上、下、前、後を直接意味しない。

### 4.5 最終小振幅セット

個別・協調とも正常な場合に限り、正負を連続確認できる。

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode all \
  --direction both \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

`--amplitude-rad`はスクリプト内で最大`0.020 rad`に制限される。初回は`0.002 rad`を使用する。

### 4.6 期待CAN ID

```text
RUN:      0x609, 0x60A, 0x60B
POSITION: 0x409, 0x40A, 0x40B
```

その他のRUN/POSITION IDが出た場合は中止する。

---

## 5. 24軸プログラムを1軸実機で確認する方法

目的が異なる2通りの方法がある。

### 5.1 正式24軸ログのphysical axis10成分をそのまま使う

```text
24軸JSONL
→ publish_cmdforjetson_jsonl.py
→ /cmdForJetson（24軸すべて有限値）
→ StateMachine
→ axis10だけUse=True
→ 0x40Aだけ実送信
```

この方法では、JSONLの`position[10]`を絶対角度としてそのまま実機axis10へ送る。

回転歩容ログの絶対角度は大きいため、単軸小振幅、1脚確認、姿勢条件、機械安全を満たす前に実施してはならない。最初は必ず短いstagedログまたは`--max-frames`を使う。

例:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_0_50_commands.jsonl \
  --rate 3 \
  --max-frames 10
```

この試験はphysical axis10とlogical axis10が同じ場合の正式絶対指令確認である。

### 5.2 任意のlogical axisをphysical axis10へ安全変換する

任意の論理軸を1台の実機へ割り当てる場合は、mapped-axis publisherを使用する。

変換:

```text
source_delta[k]
  = source[k] - source[first]

physical[k]
  = center
  + clamp(sign × scale × source_delta[k], -limit, +limit)
```

- `scale`: 正の倍率
- `--invert`: 符号反転
- `limit`: 最大`0.020 rad`
- 終了時: `return-step-rad`で中心へ段階復帰

### 5.3 最初にdry-runする

例としてlogical axis0をphysical axis10へ割り当てる。

```bash
python2 tools/publish_cmdforjetson_mapped_axis_replay.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/commands.jsonl \
  --logical-axis 0 \
  --physical-axis 10 \
  --confirm-physical-axis 10 \
  --rate 5 \
  --scale 0.01 \
  --limit-rad 0.005 \
  --return-step-rad 0.001 \
  --max-frames 50 \
  --dry-run
```

出力で確認する項目:

```text
samples
baseline_source_rad
raw_delta_range_rad
mapped_range_rad
limit_rad
clipped_count
```

`clipped_count > 0`の場合は、まず`--scale`を下げる。

### 5.4 実機再生

次だけを`Use=True`にする。

```text
Use=True:  physical axis10
Use=False: その他23軸
```

実行前に通常RUNを成立させる。

```bash
python2 tools/publish_cmdforjetson_mapped_axis_replay.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/commands.jsonl \
  --logical-axis 0 \
  --physical-axis 10 \
  --confirm-physical-axis 10 \
  --rate 5 \
  --scale 0.01 \
  --limit-rad 0.005 \
  --return-step-rad 0.001 \
  --max-frames 50
```

`--physical-axis`と`--confirm-physical-axis`が一致しなければ起動しない。

live実行時にクリップが残る場合は拒否される。dry-run結果を確認した上で、意図的に制限波形を使う場合だけ`--allow-clipping`を付ける。

### 5.5 logical axis0～23の確認

安全のため、実機では自動ループを使わず1軸ずつ実行する。

```text
logical axis0  → physical axis10
STOP・ログ確認
logical axis1  → physical axis10
STOP・ログ確認
...
logical axis23 → physical axis10
STOP・ログ確認
```

各軸について記録する。

- logical axis
- physical axis
- source JSONLとchecksum
- start-indexとmax-frames
- rate
- scale、invert、limit
- dry-run範囲とclipped count
- CAN ID
- 目視結果
- 異音、振動、温度、エラー

---

## 6. mapped-axis publisherの安全仕様

- JSONLの各レコードは24要素必須
- 対象logical axisは有限値必須
- physical axisだけ有限値、その他23軸はNaN
- `--confirm-physical-axis`必須
- `--rate`必須
- `scale > 0`
- 符号反転は明示的な`--invert`
- `0 < limit-rad <= 0.020`
- physical center ± limitが関節制限内であることを事前確認
- クリップありのlive実行は既定で拒否
- 終端値から中心へ段階復帰
- CAN直接送信なし

---

## 7. 確認できる範囲

1台の実機アクチュエータで確認できるもの:

- JSONL読込み
- 24要素`JointState`生成
- logical axis選択
- 相対変位、倍率、反転、振幅制限
- ROS publish周期
- StateMachine安全ゲート
- physical axisのCAN IDとpayload
- 長い指令列の連続受信
- 1軸の物理応答

1台では確認できないもの:

- 24台同時のCANバス占有率
- 24軸の先頭・末尾到着時間差
- 複数アクチュエータ同期
- 電源電圧降下
- 8脚の機械干渉
- 支持荷重、接地安定性、実機回転成立

最終判断は、vcan 24軸試験、1台実機による論理24軸逐次試験、将来の複数実機試験を組み合わせる。

---

## 8. テスト

```bash
python2 tests/test_cmdforjetson_hardware_publishers.py
```

Python 3環境:

```bash
python3 -m unittest \
  tests.test_cmdforjetson_hardware_publishers
```

確認対象:

- 1脚3軸が常に有限値であること
- 対象外21軸がNaNであること
- 個別・協調シーケンスが中心へ復帰すること
- logical axis相対変換
- scale、invert、limit、clip count
- physical axisだけ有限値であること
- 中心復帰ステップが指定値以下であること
- 不正軸、23要素JSONL、過大振幅を拒否すること
