# -*- coding: utf-8 -*-
"""A/B test launcher for the Tk/ROS UI event backlog issue.

This launcher keeps the production ROS topics and CAN path unchanged.  It
reuses ``ui.py`` but replaces only the UI event drain policy:

- rospy callbacks still enqueue plain Python events.
- Tk/Tcl is touched only by the Tk main thread.
- repeated status/use/diagnostic events are coalesced to their latest value.
- the drain budget is intentionally much larger than the measured input rate.

Run this file instead of ``ui.py`` only for the desktop/Jetson A/B test.
"""

import Queue
import Tkinter as tk
import rospy

import ui as base_ui


UI_EVENT_DRAIN_MAX = 4096


def _event_key(event):
    """Return a coalescing key, or None when order must be preserved."""
    if not event:
        return None
    event_type = event[0]
    if event_type in ("status", "use_status", "diagnostic_status"):
        if len(event) < 2:
            return None
        return (event_type, event[1])
    if event_type in ("diagnostic_targets", "motion_check_status"):
        return (event_type, None)
    return None


def drain_ui_events_coalesced(self, max_events=UI_EVENT_DRAIN_MAX):
    """Drain queued ROS events while applying only the latest repeated state.

    The original UI drains at most 256 events every 200 ms, i.e. 1280 events/s.
    The measured input was about 1500 events/s, so the FIFO backlog can grow
    indefinitely.  This implementation drains well above that rate and
    collapses repeated display-state events before any Tk update is performed.
    """
    latest = {}
    passthrough = []
    drained = 0

    while drained < max_events:
        try:
            event = self.ui_event_queue.get_nowait()
        except Queue.Empty:
            break

        key = _event_key(event)
        if key is None:
            passthrough.append((drained, event))
        else:
            # Keep sequence number of the latest occurrence so dispatch order
            # still follows the newest observed ROS event ordering.
            latest[key] = (drained, event)
        drained += 1

    pending = passthrough + list(latest.values())
    pending.sort(key=lambda item: item[0])
    for unused_seq, event in pending:
        self._dispatch_ui_event(event)

    return drained


def main():
    # Patch only the event-drain policy.  All widgets, commands, ROS topics and
    # StateMachine/CAN behavior come from the existing production ui.py.
    base_ui.LegControlUI._drain_ui_events = drain_ui_events_coalesced

    rospy.init_node("leg_ui")
    root = tk.Tk()
    app = base_ui.LegControlUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
