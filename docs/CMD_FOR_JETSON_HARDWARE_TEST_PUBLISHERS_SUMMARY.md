# Implementation Summary

This branch adds two ROS publishers that keep the existing production path unchanged:

```text
publisher → /cmdForJetson → StateMachine → Use=True axes → CAN
```

Files:

- `tools/publish_cmdforjetson_one_leg_test.py`
- `tools/publish_cmdforjetson_mapped_axis_replay.py`
- `tests/test_cmdforjetson_hardware_publishers.py`
- `docs/CMD_FOR_JETSON_HARDWARE_TEST_PUBLISHERS.md`
- `docs/CMD_FOR_JETSON_HARDWARE_TEST_PUBLISHERS_VALIDATION.md`

No StateMachine, CAN protocol, frozen candidate, or reference command data was modified.
