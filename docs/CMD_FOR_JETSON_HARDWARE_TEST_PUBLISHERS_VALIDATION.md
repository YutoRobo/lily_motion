# Hardware Publisher Validation

Validation date: 2026-08-04

Branch:

```text
agent/add-hardware-test-publishers
```

## Local validation

The publisher modules and pure-function tests were copied into an isolated local tree and validated without ROS or CAN access.

```text
Python 3 py_compile: PASS
unittest: 9/9 PASS
whitespace check: PASS
```

Test coverage includes:

- one-leg axes 9, 10, 11 mapping for `leg-index=3`
- exactly three finite one-leg values and 21 NaN safety guards
- individual-axis sequence returns to center
- coordinated three-axis sequence returns to center
- invalid leg, center triplet, mode, and amplitude rejection
- 24-element JSONL logical-axis extraction
- first-sample-relative mapping
- scale, invert, limit, and clipping count
- exactly one finite physical-axis value and 23 NaN guards
- stepped return to center
- invalid 23-element JSONL and excessive limit rejection

Python 2.7 was not available in the validation container. The implementation intentionally uses Python 2.7-compatible syntax and imports ROS only inside `main()` so pure functions remain independently testable.

## Not performed

- ROS publisher integration
- StateMachine integration with these two new publishers
- vcan replay
- real CAN transmission
- real one-leg motion
- real mapped-axis replay

These remain staged verification items after review and merge.
