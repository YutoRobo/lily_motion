# Quarter-Roll Gazebo → Hardware Workflow

更新日: 2026-08-11

## 1. 目的

回転動作を「1/4回転目まで」「2/4回転目まで」「3/4回転目まで」「4/4回転目まで」の累積単位で安全に確認する。

重要原則:

- 2233 frame を単純に4等分しない。
- `roll_index` の連続ブロックを意味上の各回転として扱う。
- Gazeboと実機では**同じJSONLファイル**を使用する。
- パラメータ変更後の候補はまず`testdata/`へ生成し、正式reference candidateを上書きしない。
- GazeboでPASSする前に同じstageを実機へ送らない。

## 2. 現行正式候補

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

現行候補は凍結済みのpre-hardware referenceである。

注意: 現行manifestにはsource candidate/source commitは残っているが、`candidate_022_wide`を生成した全パラメータ一式が完全な再生成profileとしては保存されていない。そのため、この文書では現行v3.0.44の「完全再生成済み」を主張しない。

## 3. 1/4〜4/4 stage生成

現行正式候補からsemantic stageを作る:

```bash
python2 tools/build_roll_quarter_stages.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/commands.jsonl \
  --output-dir data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged
```

まず書き込まず境界だけ確認する場合:

```bash
python2 tools/build_roll_quarter_stages.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/commands.jsonl \
  --output-dir /tmp/lily_quarter_stages \
  --dry-run
```

生成物:

```text
roll_to_1of4_commands.jsonl
roll_to_2of4_commands.jsonl
roll_to_3of4_commands.jsonl
roll_to_4of4_commands.jsonl
quarter_stage_manifest.json
```

各ファイルは回転開始から指定回転終了までの**累積prefix**である。`roll_to_2of4`は「2回転目だけ」ではなく「開始〜2/4終了」を含む。

`quarter_stage_manifest.json`にはsource SHA-256、roll_indexごとの開始/終了frame、各stageのframe数とSHA-256を記録する。

## 4. Gazebo確認

例: 2/4回転まで

```bash
python2 tools/run_roll_quarter_stage.py \
  --mode gazebo \
  --stage 2/4
```

デフォルトは現行v3.0.44候補を参照する。

確認項目:

- 姿勢遷移にジャンプがない
- 第2関節95deg制約を維持している
- 既知の近接部で異常干渉がない
- 2/4終了姿勢が意図した面遷移終了に一致する
- STOP相当の最終姿勢保持に問題がない

## 5. 実機確認

Gazeboで同じstageがPASSした後だけ実施する。

例: 2/4回転まで

```bash
python2 tools/run_roll_quarter_stage.py \
  --mode hardware \
  --stage 2/4 \
  --rate 3 \
  --confirm-hardware
```

実機modeでは:

1. `/ui/leg_command`へ`run`
2. Gazeboで使用したものと同じstage JSONLを`/cmdForJetson`へ送信
3. 終了/中断時に`stop`

を行う。

StateMachine、UI、CAN、Use選択、ALIGN/HOMEは事前に正しく準備済みであること。

## 6. パラメータ微調整 → 再生成

編集例:

```text
config/roll_rebuild_profiles/example_middle_swing_tuning.json
```

このprofileは**v3.0.44完全再現profileではなく、編集方法を示すtuning example**である。

例えば以下のような値をJSONで変更できる:

```text
--move-dist
--middle-swing-y-escapes
--middle-swing-y-escape-modes
--middle-swing-y-escape-phases
--resample-factor
--smooth-window
```

書き込みなしで実行内容を確認:

```bash
python2 tools/rebuild_roll_candidate.py \
  --profile config/roll_rebuild_profiles/example_middle_swing_tuning.json \
  --dry-run
```

実生成:

```bash
python2 tools/rebuild_roll_candidate.py \
  --profile config/roll_rebuild_profiles/example_middle_swing_tuning.json
```

処理順:

```text
profile JSON
   ↓
明示的generator argv
   ↓
testdata/generated_roll_candidates/... に候補生成
   ↓
生成command logのroll_indexを解析
   ↓
1/4〜4/4累積stage生成
   ↓
generation_manifest.json + quarter_stage_manifest.json
```

`tools/rebuild_roll_candidate.py`は`data/reference_candidates`への直接出力を拒否する。

## 7. 再生成候補をGazeboで確認

例:

```bash
python2 tools/run_roll_quarter_stage.py \
  --mode gazebo \
  --stage 1/4 \
  --candidate-dir testdata/generated_roll_candidates/example_middle_swing_tuning

python2 tools/run_roll_quarter_stage.py \
  --mode gazebo \
  --stage 2/4 \
  --candidate-dir testdata/generated_roll_candidates/example_middle_swing_tuning
```

1/4 → 2/4 → 3/4 → 4/4の順で進める。

## 8. 同じ再生成候補を実機へ送る

Gazebo PASS後:

```bash
python2 tools/run_roll_quarter_stage.py \
  --mode hardware \
  --stage 2/4 \
  --candidate-dir testdata/generated_roll_candidates/example_middle_swing_tuning \
  --rate 3 \
  --confirm-hardware
```

`--candidate-dir`と`--stage`が同じなら、Gazeboと実機が参照するJSONLも同じである。

## 9. 正式候補への昇格

パラメータ微調整した候補は自動ではreference candidateへ昇格しない。

昇格前に少なくとも:

1. generation manifest確認
2. semantic quarter manifest確認
3. strict command-log dry-run
4. constraint diagnostics
5. Gazebo 1/4→4/4確認
6. 必要な比較評価
7. 実機段階試験の判断

を実施する。

## 10. テスト

```bash
python2 tests/test_roll_quarter_stages.py
```

このテストは、各回転のframe数が不均等でも正しくsemantic boundaryを作ること、roll_indexが再出現する異常ログを拒否すること、4回転でないログを拒否すること、24軸形式を検証することを確認する。
