# v3.0.39 Inter-leg capsule collision evaluation

## Purpose

v3.0.39 upgrades the inter-leg check from a rough tibia-only proxy to a capsule-style link distance check.

Each leg link is approximated as a capsule:

- upper link: first/upper-link root to second joint
- lower link: second joint to foot

For every pair of different legs, the evaluator checks all upper/lower segment combinations.

## Criteria

Let `r` be the leg link radius.

- collision: `segment_distance < 2*r`
- near / insufficient margin: `segment_distance < max(inter_leg_limit, 2*r + safety_margin)`

Default values:

```text
inter_leg_link_radius = 0.015 m
inter_leg_safety_margin = 0.010 m
inter_leg_limit = 0.040 m
```

Therefore the default geometric contact threshold is `0.030 m`, and the default required clearance is `0.040 m`.

## Report fields

The constraint report now includes:

```text
inter_leg_collision_count
inter_leg_near_count
inter_leg_collision:
  method
  link_radius_m
  collision_threshold_m
  safety_margin_m
  required_clearance_m
  collision_count
  near_count
  min_distance_m
  worst
  top_collisions
  top_near
```

The `worst` entry includes roll/phase/frame, both leg names, both link names, the distance, and the required clearance.

## Important limitation

This is still a geometric diagnostic, not a full Gazebo collision solver.  It is intentionally conservative and useful for screening gait candidates before Gazebo confirmation.
