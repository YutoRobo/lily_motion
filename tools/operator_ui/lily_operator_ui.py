#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

import math
import os
import sys
import threading
import time

try:
    import Queue as queue
except ImportError:
    import queue

try:
    import Tkinter as tk
    import tkFileDialog as filedialog
    import tkMessageBox as messagebox
except ImportError:
    import tkinter as tk
    from tkinter import filedialog, messagebox

import rospy
from sensor_msgs.msg import JointState

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.dirname(THIS_DIR)
ROOT = os.path.dirname(TOOLS_DIR)
INIT_UI_DIR = os.path.join(TOOLS_DIR, 'can_interface', 'initUI')
for path in (ROOT, THIS_DIR, INIT_UI_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

import ui as legacy_ui
from motion_stream import MotionStreamError, continuity, load_motion_stream

DEFAULT_RESAMPLE_FACTOR = 5
DEFAULT_RATE_HZ = 10.0
POSITION_LENGTH = 24


class OperatorLegControlUI(legacy_ui.LegControlUI):
    """Existing control UI with operator-motion interlocks layered on top."""

    def __init__(self, root):
        self.operator_motion_active = False
        legacy_ui.LegControlUI.__init__(self, root)

    def update_motion_check_ui(self):
        legacy_ui.LegControlUI.update_motion_check_ui(self)
        if self.operator_motion_active:
            self.diagnostic_run_button.config(state=tk.DISABLED)
            self.motion_start_button.config(state=tk.DISABLED)
            self.motion_cancel_button.config(state=tk.DISABLED)
            self.align_all_button.config(state=tk.DISABLED)
            self.run_button.config(state=tk.DISABLED)

    def update_leg_ui(self, leg_id):
        legacy_ui.LegControlUI.update_leg_ui(self, leg_id)
        if not self.operator_motion_active:
            return
        widgets = self.widgets[leg_id]
        for key in ('use', 'align', 'home_l', 'home_r', 'home_set'):
            widgets[key].config(state=tk.DISABLED)

    def start_diagnostic_run(self):
        if self.operator_motion_active:
            self.motion_status_label.config(
                text='rejected: operator JSONL motion is sending')
            return
        return legacy_ui.LegControlUI.start_diagnostic_run(self)

    def start_motion_check(self):
        if self.operator_motion_active:
            self.motion_status_label.config(
                text='rejected: operator JSONL motion is sending')
            return
        return legacy_ui.LegControlUI.start_motion_check(self)

    def send_run(self):
        if self.operator_motion_active:
            return
        return legacy_ui.LegControlUI.send_run(self)


class MotionPanel(object):
    def __init__(self, root, leg_ui):
        self.root = root
        self.leg_ui = leg_ui
        self.pub = rospy.Publisher('/cmdForJetson', JointState, queue_size=10)
        self.events = queue.Queue()

        self.loaded = None
        self.loaded_continuity = None
        self.last_sent_position = None
        self.sending = False
        self.send_abort_event = threading.Event()
        self.send_thread = None
        self.was_run_ready = False
        self.abort_reason = ''
        self.last_publisher_check_wall = 0.0

        self.path_var = tk.StringVar(value='')
        self.resample_var = tk.StringVar(value=str(DEFAULT_RESAMPLE_FACTOR))
        self.rate_var = tk.StringVar(value='%.3f' % DEFAULT_RATE_HZ)
        self.file_status_var = tk.StringVar(value='No JSONL loaded')
        self.check_status_var = tk.StringVar(value='CHECK: -')
        self.progress_var = tk.StringVar(value='IDLE')

        self._build_ui()
        self.root.after(100, self._periodic_update)

    def _build_ui(self):
        frame = tk.LabelFrame(self.root, text='JSONL Motion (RUN session)')
        frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        self.frame = frame

        row1 = tk.Frame(frame)
        row1.pack(side=tk.TOP, fill=tk.X, padx=5, pady=3)
        tk.Label(row1, text='File').pack(side=tk.LEFT)
        self.path_entry = tk.Entry(row1, textvariable=self.path_var, width=72)
        self.path_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.browse_button = tk.Button(row1, text='Browse...', command=self.browse)
        self.browse_button.pack(side=tk.LEFT, padx=3)
        self.load_button = tk.Button(row1, text='LOAD / CHECK', command=self.load)
        self.load_button.pack(side=tk.LEFT, padx=3)

        row2 = tk.Frame(frame)
        row2.pack(side=tk.TOP, fill=tk.X, padx=5, pady=3)
        tk.Label(row2, text='Resample factor').pack(side=tk.LEFT)
        self.resample_entry = tk.Entry(row2, width=7, textvariable=self.resample_var)
        self.resample_entry.pack(side=tk.LEFT, padx=(4, 12))
        tk.Label(row2, text='Rate [Hz]').pack(side=tk.LEFT)
        self.rate_entry = tk.Entry(row2, width=9, textvariable=self.rate_var)
        self.rate_entry.pack(side=tk.LEFT, padx=(4, 12))
        tk.Label(
            row2,
            text='LOAD never sends a position command. SEND uses the checked in-memory stream.',
            fg='#444444').pack(side=tk.LEFT, padx=5)

        row3 = tk.Frame(frame)
        row3.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)
        tk.Label(row3, textvariable=self.file_status_var,
                 anchor='w', justify=tk.LEFT).pack(side=tk.LEFT, fill=tk.X, expand=True)

        row4 = tk.Frame(frame)
        row4.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)
        self.check_label = tk.Label(
            row4, textvariable=self.check_status_var,
            anchor='w', justify=tk.LEFT)
        self.check_label.pack(side=tk.LEFT, fill=tk.X, expand=True)

        row5 = tk.Frame(frame)
        row5.pack(side=tk.TOP, fill=tk.X, padx=5, pady=4)
        self.send_button = tk.Button(
            row5, text='SEND', width=14, command=self.send,
            font=('Helvetica', 11, 'bold'))
        self.send_button.pack(side=tk.LEFT, padx=(0, 8))
        tk.Label(row5, textvariable=self.progress_var,
                 anchor='w').pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Label(
            row5,
            text='Global STOP remains available for abnormal conditions.',
            fg='#aa0000').pack(side=tk.RIGHT, padx=5)

    def browse(self):
        if self.sending:
            return
        initial_dir = os.path.join(ROOT, 'data', 'reference_candidates')
        if not os.path.isdir(initial_dir):
            initial_dir = ROOT
        path = filedialog.askopenfilename(
            parent=self.root,
            title='Select Lily motion JSONL',
            initialdir=initial_dir,
            filetypes=[('JSONL', '*.jsonl'), ('All files', '*')])
        if path:
            self.path_var.set(path)
            self._invalidate_loaded('File selection changed; press LOAD / CHECK')

    def _parse_resample_factor(self):
        try:
            value = int(self.resample_var.get().strip())
        except Exception:
            raise MotionStreamError('resample factor must be an integer')
        if value < 1:
            raise MotionStreamError('resample factor must be >= 1')
        return value

    def _parse_rate(self):
        try:
            value = float(self.rate_var.get().strip())
        except Exception:
            raise MotionStreamError('rate must be numeric')
        if math.isnan(value) or math.isinf(value) or value <= 0.0:
            raise MotionStreamError('rate must be > 0')
        return value

    def _active_axes(self):
        return [leg.leg_id for leg in self.leg_ui.legs if leg.use]

    def _run_ready(self):
        active = self._active_axes()
        if not active:
            return False
        return all(self.leg_ui.legs[axis].state == 'Running' for axis in active)

    def _continuity_reference(self):
        if self.last_sent_position is not None:
            return list(self.last_sent_position), 'previous SEND final command'
        return [0.0] * POSITION_LENGTH, 'HOME logical zero'

    def _refresh_continuity(self):
        if self.loaded is None:
            self.loaded_continuity = None
            self.check_status_var.set('CHECK: -')
            return
        reference, reference_name = self._continuity_reference()
        try:
            result = continuity(reference, self.loaded['first_position'])
        except MotionStreamError as exc:
            self.loaded_continuity = None
            self.check_status_var.set('CHECK: ERROR - %s' % exc)
            return
        result['reference_name'] = reference_name
        self.loaded_continuity = result
        if result['pass']:
            self.check_status_var.set(
                'CHECK: PASS | continuity from %s max=%.6f rad (%.3f deg) axis%d | '
                'per-frame jump < 4 deg' % (
                    reference_name, result['max_delta_rad'],
                    result['max_delta_deg'], result['axis']))
        else:
            self.check_status_var.set(
                'CHECK: FAIL | continuity from %s max=%.6f rad (%.3f deg) axis%d '
                '>= 4 deg; SEND disabled' % (
                    reference_name, result['max_delta_rad'],
                    result['max_delta_deg'], result['axis']))

    def _invalidate_loaded(self, reason):
        self.loaded = None
        self.loaded_continuity = None
        self.file_status_var.set(reason)
        self.check_status_var.set('CHECK: -')
        self.progress_var.set('IDLE')

    def load(self):
        if self.sending:
            return
        path = self.path_var.get().strip()
        if not path:
            messagebox.showerror('LOAD failed', 'Select a JSONL file first.')
            return
        try:
            resample_factor = self._parse_resample_factor()
            loaded = load_motion_stream(path, resample_factor)
            self._parse_rate()
        except MotionStreamError as exc:
            self._invalidate_loaded('LOAD failed')
            messagebox.showerror('LOAD / CHECK failed', str(exc))
            return
        except Exception as exc:
            self._invalidate_loaded('LOAD failed')
            messagebox.showerror('LOAD / CHECK failed', str(exc))
            return

        self.loaded = loaded
        self.file_status_var.set(
            'Loaded: %s | source=%d frames | transport=%d frames | RF=%d | '
            'max_step=%.6f rad (%.3f deg) | sha256=%s' % (
                os.path.basename(loaded['path']),
                loaded['source_frame_count'],
                loaded['transport_frame_count'],
                loaded['resample_factor'],
                loaded['max_step_rad'], math.degrees(loaded['max_step_rad']),
                loaded['transport_sha256']))
        self.progress_var.set('READY after RUN/continuity checks')
        self._refresh_continuity()

    def _cmdforjetson_topology(self):
        """Return (other_publishers, subscribers, error) for /cmdForJetson."""
        try:
            import rosgraph
            master = rosgraph.Master(rospy.get_name())
            publishers, subscribers, unused_services = master.getSystemState()
            pub_nodes = []
            sub_nodes = []
            for topic, topic_nodes in publishers:
                if topic == '/cmdForJetson':
                    pub_nodes.extend(topic_nodes)
            for topic, topic_nodes in subscribers:
                if topic == '/cmdForJetson':
                    sub_nodes.extend(topic_nodes)
            own_name = rospy.get_name()
            other_publishers = sorted(set(
                node for node in pub_nodes if node != own_name))
            return other_publishers, sorted(set(sub_nodes)), None
        except Exception as exc:
            return None, None, str(exc)

    def _can_send(self):
        if self.sending or self.loaded is None:
            return False
        if self.leg_ui.motion_check_active:
            return False
        if not self._run_ready():
            return False
        if self.loaded_continuity is None or not self.loaded_continuity['pass']:
            return False
        current_path = self.path_var.get().strip()
        if not current_path or os.path.abspath(current_path) != self.loaded['path']:
            return False
        if int(self.pub.get_num_connections()) != 1:
            return False
        try:
            current_rf = self._parse_resample_factor()
            self._parse_rate()
        except MotionStreamError:
            return False
        return current_rf == self.loaded['resample_factor']

    def send(self):
        if not self._can_send():
            messagebox.showerror(
                'SEND rejected',
                'SEND requires: RUN on every Use axis, a checked JSONL, '
                'continuity PASS, unchanged file/resample settings, exactly one '
                '/cmdForJetson subscriber, and no other UI motion.')
            return
        try:
            rate_hz = self._parse_rate()
        except MotionStreamError as exc:
            messagebox.showerror('SEND rejected', str(exc))
            return

        others, subscribers, topology_error = self._cmdforjetson_topology()
        if topology_error is not None:
            messagebox.showerror(
                'SEND rejected',
                'Could not verify /cmdForJetson topology: %s' % topology_error)
            return
        if others:
            messagebox.showerror(
                'SEND rejected',
                'Another /cmdForJetson publisher is active: %s' % ', '.join(others))
            return
        if len(subscribers) != 1 or int(self.pub.get_num_connections()) != 1:
            messagebox.showerror(
                'SEND rejected',
                '/cmdForJetson must have exactly one subscriber. Found: %s' %
                (', '.join(subscribers) if subscribers else 'none'))
            return

        loaded = self.loaded
        self.sending = True
        self.send_abort_event.clear()
        self.abort_reason = ''
        self.last_publisher_check_wall = 0.0
        self.leg_ui.operator_motion_active = True
        self.progress_var.set(
            'SENDING 0/%d @ %.3f Hz' % (
                loaded['transport_frame_count'], rate_hz))
        self._apply_interlocks()

        self.send_thread = threading.Thread(
            target=self._send_worker,
            args=(loaded, rate_hz))
        self.send_thread.daemon = True
        self.send_thread.start()

    def _send_worker(self, loaded, rate_hz):
        total = loaded['transport_frame_count']
        count = 0
        last_position = None
        status = 'complete'
        error_text = ''
        try:
            rate = rospy.Rate(rate_hz)
            for position in loaded['positions']:
                if rospy.is_shutdown() or self.send_abort_event.is_set():
                    status = 'interrupted'
                    break
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.position = list(position)
                self.pub.publish(msg)
                count += 1
                last_position = list(position)
                if count == 1 or count == total or count % 20 == 0:
                    self.events.put(('progress', count, total, rate_hz))
                rate.sleep()
        except Exception as exc:
            status = 'error'
            error_text = str(exc)
        self.events.put(
            ('send_done', status, count, total, last_position, error_text))

    def _apply_interlocks(self):
        state = tk.DISABLED if self.sending else tk.NORMAL
        self.browse_button.config(state=state)
        self.load_button.config(state=state)
        self.path_entry.config(state=state)
        self.resample_entry.config(state=state)
        self.rate_entry.config(state=state)
        self.leg_ui.operator_motion_active = self.sending
        self.leg_ui.update_motion_check_ui()
        for axis in range(len(self.leg_ui.legs)):
            self.leg_ui.update_leg_ui(axis)

    def _handle_event(self, event):
        if event[0] == 'progress':
            unused, count, total, rate_hz = event
            self.progress_var.set(
                'SENDING %d/%d (%.1f%%) @ %.3f Hz' % (
                    count, total, 100.0 * count / max(total, 1), rate_hz))
            return
        if event[0] != 'send_done':
            return
        unused, status, count, total, last_position, error_text = event
        self.sending = False
        self.leg_ui.operator_motion_active = False
        if last_position is not None:
            self.last_sent_position = list(last_position)
        self._apply_interlocks()
        self._refresh_continuity()
        if status == 'complete' and count == total:
            self.progress_var.set(
                'COMPLETE %d/%d | RUN remains active; final command is held' % (
                    count, total))
        elif status == 'interrupted':
            detail = self.abort_reason or 'position output stopped'
            self.progress_var.set(
                'INTERRUPTED %d/%d | %s' % (count, total, detail))
        else:
            self.progress_var.set(
                'ERROR %d/%d | %s' % (count, total, error_text))
            messagebox.showerror('Motion SEND error', error_text or 'unknown error')

    def _drain_events(self):
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            self._handle_event(event)

    def _periodic_update(self):
        self._drain_events()
        run_ready = self._run_ready()

        if self.sending and not run_ready:
            if not self.send_abort_event.is_set():
                self.abort_reason = 'RUN state lost; position output stopped'
            self.send_abort_event.set()

        if self.sending and run_ready:
            now = time.time()
            if now - self.last_publisher_check_wall >= 0.2:
                self.last_publisher_check_wall = now
                others, subscribers, topology_error = self._cmdforjetson_topology()
                if topology_error is not None:
                    self.abort_reason = (
                        'publisher/subscriber topology check failed: %s' % topology_error)
                    self.send_abort_event.set()
                elif others:
                    self.abort_reason = (
                        'another /cmdForJetson publisher appeared: %s' %
                        ', '.join(others))
                    self.send_abort_event.set()
                elif len(subscribers) != 1 or int(self.pub.get_num_connections()) != 1:
                    self.abort_reason = (
                        '/cmdForJetson subscriber count changed: %s' %
                        (', '.join(subscribers) if subscribers else 'none'))
                    self.send_abort_event.set()

        if self.was_run_ready and not run_ready and not self.sending:
            self.last_sent_position = None
            self._refresh_continuity()
        self.was_run_ready = run_ready

        self.send_button.config(
            state=tk.NORMAL if self._can_send() else tk.DISABLED)
        if self.loaded is not None:
            current_path = self.path_var.get().strip()
            path_changed = (
                not current_path or os.path.abspath(current_path) != self.loaded['path'])
            try:
                rf_changed = self._parse_resample_factor() != self.loaded['resample_factor']
            except MotionStreamError:
                rf_changed = True
            if path_changed and not self.sending:
                self.progress_var.set(
                    'File path changed after LOAD; LOAD / CHECK again before SEND')
            elif rf_changed and not self.sending:
                self.progress_var.set(
                    'Resample factor changed after LOAD; LOAD / CHECK again before SEND')
        self.root.after(100, self._periodic_update)

    def on_close(self):
        if self.sending:
            messagebox.showwarning(
                'Motion is sending',
                'The Operator UI cannot be closed while JSONL SEND is active. '
                'Use the system STOP only for an abnormal condition.')
            return False
        return True


def main():
    rospy.init_node('lily_operator_ui')
    root = tk.Tk()
    leg_ui = OperatorLegControlUI(root)
    motion_panel = MotionPanel(root, leg_ui)
    root.title('Lily Operator UI')

    def close_handler():
        if motion_panel.on_close():
            root.destroy()

    root.protocol('WM_DELETE_WINDOW', close_handler)
    root.mainloop()


if __name__ == '__main__':
    main()
