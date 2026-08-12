# Lily Current Baseline

更新日: 2026-08-12

この文書は「現在どのbaselineを使うか」だけを示す短い入口である。過去候補の詳細履歴はここへ追記し続けない。

## Current pre-hardware candidate

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

```text
command count:              2233
coxa:                       0.075 m
thigh:                      0.300 m
tibia:                      0.300 m
second-joint max:           94.8 deg
violations over 95 deg:     0
Gazebo full-roll review:    PASS
hardware full roll:         NOT TESTED
```

Candidate command SHA256:

```text
e60c9de63287c5c198e78e11c1da89475b2293e6de45950cf09f5f2c170304a5
```

## Current pre-hardware software baseline

```text
commit:
3ff47e223c2ba67b3f6bf62de327f71de5226d86

branch:
baseline/pre-hardware-gazebo-pass-20260812
```

Frozen record:

- [`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md)

このimmutable baseline recordは後続のderived data追加では変更しない。

## Frozen transport profile for staged hardware validation

```text
resample-factor = 2
transport rate  = 10 Hz
```

Gazebo MCU-equivalent comparison profile:

```text
interpolation duration = 0.100 s
update period          = 0.002 s
```

## Verified common-path Gazebo result

既存risk-split stage:

```text
air-entry       PASS
roll 0–50       PASS
roll 50–100     PASS
roll 100–300    PASS
roll 300–end    PASS
final hold      PASS
```

Canonical architecture:

- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md)

## Frozen semantic quarter derived stages

2026-08-12に、同じfrozen `commands.jsonl` から `roll_index` の連続block境界を用いて次の累積stageを固定した。

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/
  roll_to_1of4_commands.jsonl
  roll_to_2of4_commands.jsonl
  roll_to_3of4_commands.jsonl
  roll_to_4of4_commands.jsonl
  quarter_stage_manifest.json
```

Data-freeze commit:

```text
2e42343dccf3b56066cdcc97e011dca328388a20
```

Semantic boundaries:

| stage | roll_index end | source range | cumulative frames | SHA256 |
|---|---:|---:|---:|---|
| 1/4 | 0 | 0–559 | 560 | `cf2f2592b6dd688a996b4bcc872509fa9ee3b85d8db53825ce2a01671a70dc58` |
| 2/4 | 1 | 0–1119 | 1120 | `3e54fdef3c3285b2d45f43b086081ce1dc659e7a87098981a5702561878e0bf0` |
| 3/4 | 2 | 0–1679 | 1680 | `2599ea79a90ae4746a10f6771589e50e0a5acf7d3a1e2e0f8e146b602cad3998` |
| 4/4 | 3 | 0–2232 | 2233 | `e60c9de63287c5c198e78e11c1da89475b2293e6de45950cf09f5f2c170304a5` |

Rule:

```text
Each stage is a cumulative prefix ending at the final frame
of the corresponding contiguous roll_index block.
```

したがって、2233 frameを単純4等分したものではない。

`roll_to_4of4_commands.jsonl` は元の `commands.jsonl` とbyte-for-byte同一である。

Validation status:

```text
builder/regression validation: PASS
semantic 1/4 Gazebo:          PASS
semantic 2/4 Gazebo:          PASS
semantic 3/4 Gazebo:          PASS
semantic 4/4 Gazebo:          PASS
hardware semantic quarter:    NOT TESTED
```

semantic quarterは正式に固定されたderived staged dataである。ただし、**初回実機の安全進行では既存risk-split stageを置き換えない。**

初回実機:

```text
roll 0–50 → roll 50–100 → roll 100–300 → roll 300–end
```

これがPASSした後、1/4・2/4・3/4・4/4を選択して再現する用途ではsemantic quarterを使用できる。

quarter fileはすべてrolling-start postureから始まる累積fileであるため、`1/4` の直後に `2/4` を続けて実行しない。

## Historical baseline policy

次は重要な過去候補だがcurrent baselineではない。

```text
v3.0.36 RF-1 current-angle anchor + smooth_window=40
v3.0.42c candidate_02
v3.0.42c candidate_02 softlimit 94.8
baseline_v2_42c_case27_x8_sw40
```

これらは比較・再現用に保持する。過去文書中の “current provisional baseline” という表現は、その文書作成時点の状態を表し、2026-08-12以降のcurrent baselineを意味しない。

## Change-control rule

実機validation中に次をsilent変更しない。

- frozen candidate JSONL
- staged JSONL
- semantic quarter JSONL / manifest
- transport resample factor
- transport rate
- `/cmdForJetson` semantics
- CAN StateMachine mapping
- Gazebo MCU interpolation assumption

変更が必要なら、新しいbaseline/versionを作成する。
