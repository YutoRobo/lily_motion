# -*- coding: utf-8 -*-
import Tkinter as tk
import rospy
from std_msgs.msg import String

NUM_LEGS = 24
HOME_REPEAT_MS = 80  # 長押し時の繰り返し周期[ms]（調整OK）

STATE_COLOR = {
    "Disconnected": "#cccccc",
    "Connected":    "#ffff99",
    "Aligned":      "#99ccff",
    "Homed":        "#99ff99",
    "Run":          "#ff9999",
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

        self.pub_cmd = rospy.Publisher("/ui/leg_command", String, queue_size=10)
        rospy.Subscriber("/ui/leg_status", String, self.status_callback)

        self.legs = [LegUIState(i) for i in range(NUM_LEGS)]
        self.widgets = {}

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

        tk.Button(top, text="ALIGN (Use Legs)", command=self.align_use_legs,
                  font=("Helvetica", 12, "bold")).pack(side=tk.LEFT, padx=5)

        tk.Button(top, text="RUN", command=self.send_run,
                  font=("Helvetica", 12, "bold")).pack(side=tk.LEFT, padx=5)

        tk.Button(top, text="STOP", command=self.send_stop,
                  font=("Helvetica", 12, "bold")).pack(side=tk.LEFT, padx=5)

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
            "state": lbl_state,
            "align": btn_align,
            "home_l": btn_l,
            "home_r": btn_r,
            "home_set": btn_set
        }

    # ===============================
    # ROS受信
    # ===============================
    def status_callback(self, msg):
        try:
            leg_id, state = msg.data.split(",")
            leg_id = int(leg_id)
        except:
            return

        # GUIスレッドで実行させる
        self.root.after(0, lambda: self._apply_status_update(leg_id, state))

    def _apply_status_update(self, leg_id, state):
        # ★これがないとKeyErrorは消えない
        if leg_id not in self.widgets:
            return
        leg = self.legs[leg_id]
        prev = leg.state
        leg.state = state

        # 切断時は強制OFFし、StateMachine側のactive selectionも落とす
        if state == "Disconnected":
            if leg.use:
                leg.use = False
                self.widgets[leg_id]["use_var"].set(0)
                self.publish_use_state(leg_id)
            else:
                self.widgets[leg_id]["use_var"].set(0)
            return

        # Disconnected -> 何か の瞬間は自動ONし、StateMachineへUse=Trueを通知
        if prev == "Disconnected":
            leg.use = True
            self.widgets[leg_id]["use_var"].set(1)
            self.publish_use_state(leg_id)

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
    def align_use_legs(self):
        for leg in self.legs:
            if leg.state == "Connected" and leg.use:
                self.pub_cmd.publish("align:{}".format(leg.leg_id))

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
        for i in range(NUM_LEGS):
            self.update_leg_ui(i)
        self.root.after(200, self.update_ui_loop)

    def update_leg_ui(self, leg_id):
        leg = self.legs[leg_id]
        w = self.widgets[leg_id]

        # 状態色
        w["state"].config(text=leg.state, bg=STATE_COLOR.get(leg.state, "white"))

        # AlignはConnectedのみ
        w["align"].config(state=tk.NORMAL if leg.state == "Connected" else tk.DISABLED)

        # 原点操作はAlignedのみ
        home_enable = (leg.state == "Aligned")
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
