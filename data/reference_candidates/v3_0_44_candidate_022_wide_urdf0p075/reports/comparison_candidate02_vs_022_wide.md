# candidate02 vs candidate_022_wide URDF FK Comparison

## Method

Both command logs were evaluated with `tools/diagnostics/urdf_fk_evaluator.py`, which parses `testdata/body_root_transform_geometry_check/robot_description_from_ros.xml` and follows URDF joint origins and axes. It does not use `default_octpus_mounts()`.

The inter-leg value is a primitive segment clearance proxy: segment centerline distance minus approximate segment radii. It is not a Gazebo visual collision result.

## Results

- candidate02_softlimit_94p8 min clearance: `0.016471 m` at line_index `229`, frame_index `28`, roll_index `0`, phase `RF-2_Goal2_UpperLegLanding`, pair `TRF:coxa_end_to_knee` vs `BRF:knee_to_foot`
- candidate_022_wide min clearance: `0.019649 m` at line_index `1164`, frame_index `145`, roll_index `2`, phase `RF-2_Goal2_UpperLegLanding`, pair `TLF:coxa_end_to_knee` vs `BLF:coxa_end_to_knee`
- clearance delta candidate_022_wide - candidate02: `0.003177 m`

Foot clearance:

- candidate02 min foot z: `-0.003903 m` at line_index `223`, phase `RF-2_Goal2_UpperLegLanding`
- candidate_022_wide min foot z: `-0.003903 m` at line_index `223`, phase `RF-2_Goal2_UpperLegLanding`

## Interpretation

Under the URDF FK primitive proxy, `candidate_022_wide` improves the worst inter-leg clearance by about `3.177 mm` relative to candidate02. Both still have the same minimum foot z in this evaluation, so this result should not be treated as full safety approval.

Older primitive reports based on Python fallback FK are stale because that FK used fallback roots and a different q0 axis convention.

## Safety state

- data_reference_candidates_modified=false
- tools_can_interface_modified=false
- publish_cmdforjetson_jsonl_modified=false
- can0_opened=false
- hardware_can_sent=false
