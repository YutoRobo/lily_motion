# UI coalesced A/B test

This temporary launcher is for reproducing and evaluating the Jetson UI backlog issue without changing the production CAN/StateMachine path.

Run the existing UI for baseline:

```bash
python2 tools/can_interface/initUI/ui.py
```

Run the coalesced A/B launcher:

```bash
python2 tools/can_interface/initUI/ui_coalesced_ab.py
```

The launcher reuses `ui.py` and changes only `_drain_ui_events()` at runtime. Repeated `status`, `use_status`, `diagnostic_status`, `diagnostic_targets`, and `motion_check_status` events are reduced to the newest event in each 200 ms UI cycle. Unknown event types preserve FIFO dispatch.

Measured baseline on the desktop before this change:

- `/ui/leg_status`: about 731 msg/s
- `/ui/leg_use_status`: about 742 msg/s
- `/ui/diagnostic_targets`: about 30 msg/s
- total: about 1500 msg/s

The current production UI drains at most 256 events every 200 ms, equivalent to 1280 events/s, so the measured input can grow the queue indefinitely. The A/B launcher drains up to 4096 queued events per UI cycle and coalesces repeated display state before Tk updates.

For desktop comparison, keep the same StateMachine running and compare UI responsiveness for several minutes. The ROS topic rates are expected to remain unchanged because this A/B launcher changes only the UI consumer.
