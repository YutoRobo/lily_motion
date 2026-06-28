# v3.0.9 Contact-Locked Generation and Contact-Plan Sweep

## Purpose

v3.0.8 made contact drift visible after raw/filtered evaluation.  v3.0.9 moves one step upstream: SUPPORT feet now receive generation-side contact locks.  A SUPPORT leg creates a world-frame lock point when it enters SUPPORT and keeps that foot target fixed until the leg leaves SUPPORT.

This does not magically make the roll feasible.  It makes the failure more honest: if a locked support foot cannot be maintained, the generator must report IK failure or drift after filtering instead of silently changing the contact target.

## Added behavior

- `enable_contact_lock_generation=True` by default.
- `--no-contact-lock-generation` can disable the behavior for comparison.
- Each frame records `diagnostics.contact_lock_generation`.
- `task_success.contact_lock_generation_enabled` is reported.
- `task_success.contact_plan_variant` is reported.

## Contact-plan variants

The concept phase generator now supports explicit support-set variants:

- `default`
- `next_only_roll`
- `six_support_roll`
- `front_pair_roll`
- `rear_pair_roll`

The point is not that these are final gait patterns.  The point is that the support-set assumption is no longer hidden.  It can now be swept and compared.

## Run examples

```bash
python run_v3_0_whole_roll_eval.py --summary-only
```

```bash
python run_v3_0_whole_roll_eval.py \
  --summary-only \
  --contact-plan-variant next_only_roll
```

```bash
python run_v3_0_parameter_sweep.py \
  --contact-plan-variants default,next_only_roll,six_support_roll,front_pair_roll,rear_pair_roll \
  --steps-per-phase 6,8 \
  --lift-heights 0.06,0.08,0.10 \
  --clearance-heights 0.05,0.08 \
  --candidate-support-shift-xs 0.02,0.04,0.06 \
  --candidate-support-drop-zs=-0.04,-0.02,0.0 \
  --output testdata/v3_0_9_parameter_sweep_summary.json
```

## Current interpretation

The default plan still fails.  `next_only_roll` reduces contact drift and penetration somewhat in the local check, but it still does not satisfy the full whole-roll feasibility conditions.  This means the next problem is not replay timing; it is the contact plan and pre-roll support placement.
