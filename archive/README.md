# Archive

`archive/` は過去検討、旧実験、非現行runner、旧文書の保管用である。

## Rule

**archive内の内容を、現在の実機操作の入口として使用しない。**

現在のruntimeやcandidateはarchive文書に重複記載しない。現行情報は常に次を参照する。

- [`../README.md`](../README.md)
- [`../docs/RUNTIME_ARCHITECTURE.md`](../docs/RUNTIME_ARCHITECTURE.md)
- [`../docs/HARDWARE_OPERATION_PROCEDURE.md`](../docs/HARDWARE_OPERATION_PROCEDURE.md)
- [`../docs/BASELINE.md`](../docs/BASELINE.md)

## Contents

```text
archive/v3_experiment_scripts/
```

過去のsweep、旧candidate生成、旧Gazebo replay、旧評価runnerなど。

```text
archive/docs_legacy/
```

過去時点の設計note、experiment log、旧operation文書。

これらはtraceability / reproductionのため残す。

## Current-vs-history

古い文書中の次の表現は、その文書作成時点の意味で読む。

```text
current baseline
current provisional baseline
current hardware candidate
approved rate
```

2026-08-12以降のcurrent statusを意味しない。

## Stale staged wrapper

common `/cmdForJetson` architecture成立前の `tools/run_hardware_staged_manual.sh` はarchiveへ移設する。

このwrapperは:

- old candidate path
- old hardware/Gazebo rate
- separate Gazebo direct replay path

を含むため、現行hardware operationには使用しない。
