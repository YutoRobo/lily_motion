# v3.0.40 Joint-housing inter-leg collision evaluation

## Purpose

v3.0.39 checked inter-leg collision by the shortest distance between upper/lower link centerline segments, modeled as capsules. That is not enough for cases where the bulky second-third joint housing / lower-link-root area appears to contact another leg's second link. The link centerlines can still be far enough apart while the joint housing visually collides.

v3.0.40 adds a separate diagnostic:

- second-third joint housing of one leg, approximated as a sphere
- vs. upper/lower link capsule of every other leg

This keeps the previous link-vs-link capsule check and adds a housing-vs-link check.

## Geometry

For each leg:

- upper link: hip/root to second joint
- lower link: second joint to foot
- joint housing: sphere centered at the second joint / second-third joint location

For every pair of different legs:

```text
joint_housing_sphere(leg A) vs upper_link_capsule(leg B)
joint_housing_sphere(leg A) vs lower_link_capsule(leg B)
```

The distance is computed as point-to-segment distance between the housing center and the other leg's link centerline.

## Thresholds

Default values:

```text
inter_leg_link_radius = 0.015 m
inter_leg_joint_housing_radius = 0.030 m
inter_leg_joint_housing_safety_margin = 0.005 m
```

Therefore:

```text
collision_threshold = link_radius + joint_housing_radius = 0.045 m
required_clearance  = collision_threshold + safety_margin = 0.050 m
```

Interpretation:

```text
distance < 0.045 m: collision
0.045 m <= distance < 0.050 m: near / insufficient margin
```

## Report fields

The evaluator adds:

```text
inter_leg_joint_housing_collision_count
inter_leg_joint_housing_near_count
inter_leg_joint_housing_collision:
  method
  joint_housing_radius_m
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

`worst` includes:

```text
frame_index
roll_index
phase_name
phase_step_index
joint_leg_id / joint_leg_name
joint_name
link_leg_id / link_leg_name
link_name
distance_m
required_clearance_m
clearance_margin_m
```

## Notes

This is still a geometric approximation. It is intentionally conservative for visual/Gazebo screening. If the CAD/URDF joint housing radius differs from 30 mm, use the CLI options to tune the thresholds.
