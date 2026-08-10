# -*- coding: utf-8 -*-
import Tkinter as tk
import Queue
import rospy
from std_msgs.msg import String

NUM_LEGS = 24
HOME_REPEAT_MS = 80  # 長押し時の繰り返し周期[ms]（調整OK）
UI_EVENT_DRAIN_MAX = 4096

STATE_COLOR = {
    "Disconnected": "#cccccc",
    "Connected":        "#ffff99",
    "Aligning":         "#ffcc66",
    "ALIGN incomplete": "#ffcc99",
    "Aligned":          "#99ccff",
    "Homed":            "#99ff99",
    "Running":          "#ff9999",
    "Error":            "#ff6666",
}

# ===============================
# 内部状態
# ===============================
class LegUIState(object):
    def __init__(self, leg_id):
        self.leg_id = leg_id
        self.state = "Disconnected"
        self.use = False

# ===============================
# UI本体
# ===============================
class LegControlUI(object):
    def __init__(self, root):
        self.root = root
        self.root.title("Leg Control UI")

        # rospy subscriber callbacks run outside the Tk main thread.
        # They must never call Tk/Tcl APIs, including root.after().
        # Create the queue before subscribing so an immediate ROS callback is
        # safe even before root.mainloop() starts.
        self.ui_event_queue = Queue.Queue()

        self.pub_cmd = rospy.Publisher("/ui/leg_command", String, queue_size=10)
        rospy.Subscriber("/ui/leg_status", String, self.status_callback)
        rospy.Subscriber("/ui/leg_use_status", String, self.use_status_callback)
        rospy.Subscriber(
            "/ui/motion_check_status", String, self.motion_check_status_callback)
        rospy.Subscriber(
            "/ui/diagnostic_targets", String, self.diagnostic_targets_callback)
        rospy.Subscriber(
            "/ui/diagnostic_status", String, self.diagnostic_status_callback)

        self.legs = [LegUIState(i) for i in range(NUM_LEGS)]
        self.widgets = {}
        self.motion_axis_var = tk.StringVar(value="")
        self.motion_direction_var = tk.StringVar(value="+")
        self.motion_check_active = False
        self.motion_candidates = []
        self.motion_target_labels = {}
        self.diagnostic_run_sent_axes = set()

        # 長押し用：after job 管理
        self.home_repeat_job = {}  # key=(leg_id,dir) -> job id

        self.build_ui()
        self.update_ui_loop()

    # ===============================
    # UI構築
    # ===============================
    def build_ui(self):
        top = tk.Frame(self.root)
        top.pack(side=tk.TOP, fill=tk.X)

        self.align_all_button = tk.Button(
            top, text="ALIGN (Use Legs)", command=self.align_use_legs,
            font=("Helvetica", 12, "bold"))
        self.align_all_button.pack(side=tk.LEFT, padx=5)

        self.run_button = tk.Button(
            top, text="RUN ALL AXES", command=self.send_run,
            font=("Helvetica", 12, "bold"))
        self.run_button.pack(side=tk.LEFT, padx=5)

        tk.Button(top, text="STOP", command=self.send_stop,
                  font=("Helvetica", 12, "bold")).pack(side=tk.LEFT, padx=5)

        motion = tk.LabelFrame(self.root, text="RUN動作確認")
        motion.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        tk.Label(motion, text="対象軸").pack(side=tk.LEFT)
        self.motion_axis_menu = tk.OptionMenu(motion, self.motion_axis_var, "")
        self.motion_axis_menu.pack(side=tk.LEFT)
        tk.Label(motion, text="方向").pack(side=tk.LEFT)
        tk.Radiobutton(
            motion, text="+", variable=self.motion_direction_var,
            value="+").pack(side=tk.LEFT)
        tk.Radiobutton(
            motion, text="-", variable=self.motion_direction_var,
            value="-").pack(side=tk.LEFT)
        self.diagnostic_run_button = tk.Button(
            motion, text="選択軸 Diagnostic RUN",
            command=self.start_diagnostic_run)
        self.diagnostic_run_button.pack(side=tk.LEFT, padx=5)
        self.motion_start_button = tk.Button(
            motion, text="RUN動作確認開始", command=self.start_motion_check)
        self.motion_start_button.pack(side=tk.LEFT, padx=5)
        self.motion_cancel_button = tk.Button(
            motion, text="動作確認中止", command=self.cancel_motion_check)
        self.motion_cancel_button.pack(side=tk.LEFT, padx=5)
        self.motion_status_label = tk.Label(
            motion,
            text="待機 / 振幅0.020 rad (1.15 deg), 速度0.05 rad/s, q0へ復帰")
        self.motion_status_label.pack(side=tk.LEFT, padx=5)
        tk.Label(
            self.root,
            text="警告: RUN動作確認中は外部回転publisherを起動しないこと",
            fg="#aa0000").pack(side=tk.TOP, fill=tk.X)

        table = tk.Frame(self.root)
        table.pack(side=tk.TOP)

        header = ["Leg", "Use", "State", "Align", "Home"]
        for c, h in enumerate(header):
            tk.Label(table, text=h, font=("Helvetica", 10, "bold"),
                     width=12).grid(row=0, column=c)

        for i in range(NUM_LEGS):
            self.build_leg_row(table, i)

    def build_leg_row(self, parent, leg_id):
        leg = self.legs[leg_id]
        row = leg_id + 1

        tk.Label(parent, text=str(leg_id), width=5).grid(row=row, column=0)

        use_var = tk.IntVar(value=0)
        chk = tk.Checkbutton(parent, variable=use_var,
                             command=lambda i=leg_id: self.user_toggle_use(i))
        chk.grid(row=row, column=1)

        lbl_state = tk.Label(parent, text=leg.state,
                             width=12, relief=tk.SUNKEN)
        lbl_state.grid(row=row, column=2)

        btn_align = tk.Button(parent, text="Align",
                              command=lambda i=leg_id: self.send_align(i))
        btn_align.grid(row=row, column=3)

        frame_home = tk.Frame(parent)

        # 長押し対応：commandではなくbindで制御
        btn_l = tk.Button(frame_home, text="◀")
        btn_r = tk.Button(frame_home, text="▶")
        btn_set = tk.Button(frame_home, text="Set",
                            command=lambda i=leg_id: self.set_home(i))

        btn_l.bind("<ButtonPress-1>",   lambda e, i=leg_id: self.start_home_repeat(i, -1))
        btn_l.bind("<ButtonRelease-1>", lambda e, i=leg_id: self.stop_home_repeat(i, -1))
        btn_l.bind("<Leave>",           lambda e, i=leg_id: self.stop_home_repeat(i, -1))

        btn_r.bind("<ButtonPress-1>",   lambda e, i=leg_id: self.start_home_repeat(i, +1))
        btn_r.bind("<ButtonRelease-1>", lambda e, i=leg_id: self.stop_home_repeat(i, +1))
        btn_r.bind("<Leave>",           lambda e, i=leg_id: self.stop_home_repeat(i, +1))

        btn_l.pack(side=tk.LEFT)
        btn_r.pack(side=tk.LEFT)
        btn_set.pack(side=tk.LEFT)
        frame_home.grid(row=row, column=4)

        self.widgets[leg_id] = {
            "use_var": use_var,
            "use": chk,
            "state": lbl_state,
            "align": btn_align,
            "home_l": btn_l,
            "home_r": btn_r,
            "home_set": btn_set
        }

    # ===============================
    # ROS受信
    # ===============================
    def _enqueue_ui_event(self, event):
        """Thread-safe handoff from rospy callbacks to the Tk main thread."""
        self.ui_event_queue.put(event)

    def diagnostic_targets_callback(self, msg):
        targets = []
        labels = {}
        for field in msg.data.split(";") if msg.data else []:
            try:
                axis_text, label = field.split("|", 1)
                axis = int(axis_text)
            except Exception:
                continue
            targets.append(axis)
            labels[axis] = label
        self._enqueue_ui_event(("diagnostic_targets", targets, labels))

    def _apply_diagnostic_targets(self, targets, labels):
        self.motion_candidates = list(targets)
        self.motion_target_labels = dict(labels)

    def diagnostic_status_callback(self, msg):
        try:
            axis_text, status = msg.data.split("|", 1)
            axis = int(axis_text)
        except Exception:
            return
        self._enqueue_ui_event(("diagnostic_status", axis, status))

    def _apply_diagnostic_status(self, axis, status):
        if status == "Diagnostic RUN sent":
            self.diagnostic_run_sent_axes.add(axis)
        else:
            self.diagnostic_run_sent_axes.discard(axis)
        self.motion_status_label.config(text="axis%d: %s" % (axis, status))

    def motion_check_status_callback(self, msg):
        self._enqueue_ui_event(("motion_check_status", msg.data))

    def _apply_motion_check_status(self, status):
        self.motion_check_active = (
            status.startswith("active:") or status.startswith("returning:"))
        self.motion_status_label.config(text=status)

    def status_callback(self, msg):
        try:
            leg_id, state = msg.data.split(",")
            leg_id = int(leg_id)
        except Exception:
            return
        self._enqueue_ui_event(("status", leg_id, state))

    def use_status_callback(self, msg):
        try:
            leg_id, active = msg.data.split(",")
            leg_id = int(leg_id)
            active = bool(int(active))
        except Exception:
            return
        self._enqueue_ui_event(("use_status", leg_id, active))

    def _apply_use_status_update(self, leg_id, active):
        if leg_id not in self.widgets:
            return
        self.legs[leg_id].use = bool(active)
        self.widgets[leg_id]["use_var"].set(1 if active else 0)

    def _apply_status_update(self, leg_id, state):
        # ★これがないとKeyErrorは消えない
        if leg_id not in self.widgets:
            return
        leg = self.legs[leg_id]
        leg.state = state

        # Use is an explicit initialization/RUN configuration. Connection
        # status must never auto-enable or auto-disable it.

    def _ui_event_key(self, event):
        """Return a coalescing key for display-state events."""
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

    def _dispatch_ui_event(self, event):
        """Apply one queued ROS event. Called only by the Tk main thread."""
        if not event:
            return
        event_type = event[0]
        if event_type == "diagnostic_targets":
            self._apply_diagnostic_targets(event[1], event[2])
        elif event_type == "diagnostic_status":
            self._apply_diagnostic_status(event[1], event[2])
        elif event_type == "motion_check_status":
            self._apply_motion_check_status(event[1])
        elif event_type == "status":
            self._apply_status_update(event[1], event[2])
        elif event_type == "use_status":
            self._apply_use_status_update(event[1], event[2])
        else:
            rospy.logwarn("[UI] unknown queued event: %s", event_type)

    def _drain_ui_events(self, max_events=UI_EVENT_DRAIN_MAX):
        """Drain queued ROS events and apply only the latest repeated state.

        Repeated display-state events are coalesced per event type/axis before
        touching Tk widgets. Unknown event types preserve FIFO dispatch order.
        """
        latest = {}
        passthrough = []
        drained = 0

        while drained < max_events:
            try:
                event = self.ui_event_queue.get_nowait()
            except Queue.Empty:
                break

            key = self._ui_event_key(event)
            if key is None:
                passthrough.append((drained, event))
            else:
                # Keep the sequence index of the newest occurrence so the
                # final dispatch order follows the newest observed ROS order.
                latest[key] = (drained, event)
            drained += 1

        pending = passthrough + list(latest.values())
        pending.sort(key=lambda item: item[0])
        for unused_seq, event in pending:
            self._dispatch_ui_event(event)

        return drained

    # ===============================
    # Use操作（ユーザーが管理）
    # ===============================
    def user_toggle_use(self, leg_id):
        var = self.widgets[leg_id]["use_var"]
        self.legs[leg_id].use = bool(var.get())
        self.publish_use_state(leg_id)

    def publish_use_state(self, leg_id):
        active = 1 if self.legs[leg_id].use else 0
        self.pub_cmd.publish("use:{}:{}".format(leg_id, active))

    # ===============================
    # 長押し：原点微動（◀▶）
    # ===============================
    def start_home_repeat(self, leg_id, direction):
        key = (leg_id, direction)
        if key in self.home_repeat_job:
            return
        self._home_repeat_tick(leg_id, direction)

    def stop_home_repeat(self, leg_id, direction):
        key = (leg_id, direction)
        job = self.home_repeat_job.pop(key, None)
        if job is not None:
            try:
                self.root.after_cancel(job)
            except:
                pass

    def _home_repeat_tick(self, leg_id, direction):
        self.home_move(leg_id, direction)
        key = (leg_id, direction)
        job = self.root.after(HOME_REPEAT_MS, lambda: self._home_repeat_tick(leg_id, direction))
        self.home_repeat_job[key] = job

    # ===============================
    # UI → ROS
    # ===============================
    def start_diagnostic_run(self):
        axis = self._selected_motion_axis()
        if axis is None:
            self.motion_status_label.config(text="rejected: target_not_selected")
            return
        self.pub_cmd.publish("diagnostic_run:{}".format(axis))

    def _selected_motion_axis(self):
        label = self.motion_axis_var.get()
        for axis, candidate_label in self.motion_target_labels.items():
            if candidate_label == label:
                return axis
        return None

    def start_motion_check(self):
        axis = self._selected_motion_axis()
        if axis is None:
            self.motion_status_label.config(text="rejected: target_not_selected")
            return
        direction = self.motion_direction_var.get()
        self.pub_cmd.publish(
            "motion_check_start:{}:{}".format(axis, direction))

    def cancel_motion_check(self):
        self.pub_cmd.publish("motion_check_cancel")

    def align_use_legs(self):
        # StateMachine selects only Use=True, incomplete, standby-ready axes.
        self.pub_cmd.publish("align")

    def send_align(self, leg_id):
        if self.legs[leg_id].use:
            self.pub_cmd.publish("align:{}".format(leg_id))

    def home_move(self, leg_id, direction):
        # UI側ではUseだけ見る（Aligned制約はボタンdisableで担保）
        if self.legs[leg_id].use:
            self.pub_cmd.publish("home_move:{}:{}".format(leg_id, direction))

    def set_home(self, leg_id):
        if self.legs[leg_id].use:
            self.pub_cmd.publish("set_home:{}".format(leg_id))

    def send_run(self):
        self.pub_cmd.publish("run")

    def send_stop(self):
        self.pub_cmd.publish("stop")

    # ===============================
    # UI更新
    # ===============================
    def update_ui_loop(self):
        # All Tk/Tcl updates triggered by ROS messages happen here, on the
        # Tk main thread. rospy callbacks only enqueue plain Python data.
        self._drain_ui_events()
        for i in range(NUM_LEGS):
            self.update_leg_ui(i)
        self.update_motion_check_ui()
        self.root.after(200, self.update_ui_loop)

    def update_motion_check_ui(self):
        candidates = list(self.motion_candidates)
        labels = [self.motion_target_labels[axis] for axis in candidates
                  if axis in self.motion_target_labels]
        menu = self.motion_axis_menu["menu"]
        current = self.motion_axis_var.get()
        current_labels = getattr(self, "_rendered_motion_labels", None)
        if labels != current_labels:
            self._rendered_motion_labels = list(labels)
            menu.delete(0, "end")
            for label in labels:
                menu.add_command(
                    label=label,
                    command=lambda value=label: self.motion_axis_var.set(value))
            if not labels:
                self.motion_axis_var.set("")
            elif current not in labels:
                self.motion_axis_var.set(labels[0])
        axis = self._selected_motion_axis()
        diagnostic_sent = axis in self.diagnostic_run_sent_axes
        self.diagnostic_run_button.config(
            state=tk.NORMAL if axis is not None and not self.motion_check_active
            else tk.DISABLED)
        self.motion_start_button.config(
            state=tk.NORMAL if diagnostic_sent and not self.motion_check_active
            else tk.DISABLED)
        self.motion_cancel_button.config(
            state=tk.NORMAL if self.motion_check_active else tk.DISABLED)
        self.align_all_button.config(
            state=tk.DISABLED if self.motion_check_active else tk.NORMAL)
        self.run_button.config(
            state=tk.DISABLED if self.motion_check_active else tk.NORMAL)

    def update_leg_ui(self, leg_id):
        leg = self.legs[leg_id]
        w = self.widgets[leg_id]

        # Detailed diagnostics may suffix Error / ALIGN incomplete.
        color_key = leg.state
        if leg.state.startswith("Error"):
            color_key = "Error"
        elif leg.state.startswith("ALIGN incomplete"):
            color_key = "ALIGN incomplete"
        w["state"].config(text=leg.state, bg=STATE_COLOR.get(color_key, "white"))

        # Use configuration is frozen once ALIGN starts.
        session_state = (leg.state in (
            "Aligning", "Aligned", "Homed", "Running")
            or leg.state.startswith("ALIGN incomplete")
            or leg.state.startswith("Error"))
        w["use"].config(
            state=tk.DISABLED if session_state or self.motion_check_active
            else tk.NORMAL)

        # Retry is accepted only for standby-ready incomplete axes.
        align_enable = leg.use and leg.state == "Connected"
        w["align"].config(
            state=tk.NORMAL if align_enable and not self.motion_check_active
            else tk.DISABLED)

        # 原点操作はAlignedのみ
        home_enable = (
            leg.state == "Aligned" and not self.motion_check_active)
        for k in ["home_l", "home_r", "home_set"]:
            w[k].config(state=tk.NORMAL if home_enable else tk.DISABLED)

# ===============================
# main
# ===============================
if __name__ == "__main__":
    rospy.init_node("leg_ui")
    root = tk.Tk()
    app = LegControlUI(root)
    root.mainloop()
