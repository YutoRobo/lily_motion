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
- transport resample factor
- transport rate
- `/cmdForJetson` semantics
- CAN StateMachine mapping
- Gazebo MCU interpolation assumption

変更が必要なら、新しいbaseline/versionを作成する。
