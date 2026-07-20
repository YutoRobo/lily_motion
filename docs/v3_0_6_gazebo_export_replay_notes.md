# v3.0.6 Gazebo export / preview replay

## 目的

v3.0.6では、プロジェクト内完結のv3候補フレームを、既存Gazeboの24関節controller topic順へ変換するexporterと、Gazeboへ再生するpreview scriptを追加した。

現段階のv3候補はまだ `completed=false` であり、Gazeboで成功歩容を見る段階ではない。目的は、破綻直前までを可視化して、どのphase・どの姿勢から幾何的に厳しくなるかを目視確認することである。

## 追加ファイル

```text
lily_motion_v3/gazebo_export.py
tools/command_generation/run_v3_0_export_commands.py
tools/gazebo/run_v3_0_gazebo_replay.py
docs/v3_0_6_gazebo_export_replay_notes.md
```

## 重要な設計

v3のleg idはproject-containedな独自順である。
一方、Gazebo controller topic順は既存 `lily_motion.config.robot_config.JOINT_STATE_ORDER` に従う。

そのため、v3.0.6ではleg idではなくleg nameで対応付ける。

```text
v3 leg name -> existing Gazebo joint topic order
```

これにより、v3内部のleg idをlegacy互換へ無理に合わせずに済む。

## ROSなしでコマンド列だけ確認

```bash
python tools/command_generation/run_v3_0_export_commands.py \
  --output testdata/v3_0_6_preview_commands.jsonl
```

既定では、IK失敗・床貫通・base pose search失敗の最初のframe手前で止める。

最初の異常frameも含めたい場合：

```bash
python tools/command_generation/run_v3_0_export_commands.py \
  --include-invalid-frame \
  --output testdata/v3_0_6_preview_with_invalid.jsonl
```

全部出したい場合：

```bash
python tools/command_generation/run_v3_0_export_commands.py \
  --allow-invalid-frames \
  --output testdata/v3_0_6_all_commands.jsonl
```

## Gazeboで破綻直前までpreview

Gazeboを別端末で起動したうえで実行する。

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --rate 30 \
  --command-log testdata/v3_0_6_gazebo_preview_commands.jsonl \
  --candidate-output testdata/v3_0_6_candidate.json \
  --gazebo-link-state-log testdata/v3_0_6_gazebo_link_states.jsonl
```

既定では破綻直前で止める。最初の異常frameまで見たい場合：

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --include-invalid-frame \
  --rate 30
```

全frameを流すこともできるが、現段階では推奨しない。

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --allow-invalid-frames \
  --rate 30
```

## dry-run

ROSなしでreplay scriptの変換だけ確認する場合：

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --dry-run \
  --command-log testdata/v3_0_6_dryrun_commands.jsonl
```

## 注意

これはGazebo成功動作用ではなく、v3候補の可視化ブリッジである。
現状では `completed=false` が残っているため、まずは破綻直前の姿勢を確認する。
