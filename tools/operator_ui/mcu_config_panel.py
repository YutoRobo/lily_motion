#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import sys
import threading

try:
    import Queue as queue
except ImportError:
    import queue

try:
    import Tkinter as tk
    import ttk
    import tkMessageBox as messagebox
except ImportError:
    import tkinter as tk
    from tkinter import ttk, messagebox

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.dirname(THIS_DIR)
MCU_CONFIG_DIR = os.path.join(TOOLS_DIR, 'mcu_config')
if MCU_CONFIG_DIR not in sys.path:
    sys.path.insert(0, MCU_CONFIG_DIR)

import lily_mcu_config_editor as config


class McuConfigPanel(object):
    """Embedded adapter around the maintained MCU Config protocol helpers.

    READ is allowed whenever the CAN interface is available.  WRITE/SAVE are
    additionally gated by the integrated operator state so that the UI will not
    start a modifying Config transaction while RUN or JSONL SEND is active.
    The MCU remains the final authority and may still reject an invalid state.
    """

    def __init__(self, root, can_interface='can0', axes=None,
                 allow_modify_callback=None, modify_active_callback=None):
        self.root = root
        self.interface = config.normalize_interface(can_interface)
        self.axes = list(axes if axes is not None else range(24))
        if not self.axes:
            raise ValueError('MCU Config requires at least one axis')

        self.allow_modify_callback = allow_modify_callback
        self.modify_active_callback = modify_active_callback
        self.can = config.CanUtilsCAN(self.interface)
        self.events = queue.Queue()
        self.stop_event = threading.Event()
        self.worker = None
        self.worker_kind = None
        self.axis_data = dict((a, {}) for a in self.axes)
        self.axis_status = dict((a, u'未取得') for a in self.axes)
        self.selected_param_key = None
        self.hw_reboot_required_axes = set()
        self.closed = False

        self._build_ui()
        try:
            self.can.open()
            self.connection_var.set('%s: 接続済み' % self.interface)
        except Exception as exc:
            self.connection_var.set('%s: 接続失敗' % self.interface)
            self.status_var.set(u'CAN Config通信を開始できません: %s' % exc)

        self.root.after(50, self._poll_events)
        self.root.after(150, self._refresh_modify_gate)

    @property
    def modify_active(self):
        return self.worker_kind in ('write', 'save') and self._worker_alive()

    def _worker_alive(self):
        return self.worker is not None and self.worker.is_alive()

    def _modification_allowed(self):
        if self.allow_modify_callback is None:
            return True
        try:
            return bool(self.allow_modify_callback())
        except Exception:
            return False

    def _notify_modify_active(self, active):
        if self.modify_active_callback is None:
            return
        try:
            self.modify_active_callback(bool(active))
        except Exception:
            pass

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)

        self.connection_var = tk.StringVar(value='%s: 未接続' % self.interface)
        ttk.Label(top, textvariable=self.connection_var).pack(
            side=tk.LEFT, padx=(0, 14))

        ttk.Label(top, text='Axis').pack(side=tk.LEFT)
        self.axis_var = tk.StringVar(value=str(self.axes[0]))
        self.axis_combo = ttk.Combobox(
            top, width=6, textvariable=self.axis_var,
            values=[str(a) for a in self.axes], state='readonly')
        self.axis_combo.pack(side=tk.LEFT, padx=(4, 10))
        self.axis_combo.bind('<<ComboboxSelected>>', self._on_axis_selected)

        self.btn_selected = ttk.Button(
            top, text=u'選択軸をREAD', command=self.refresh_selected)
        self.btn_selected.pack(side=tk.LEFT, padx=4)
        self.btn_all = ttk.Button(
            top, text=u'全軸をREAD', command=self.refresh_all)
        self.btn_all.pack(side=tk.LEFT, padx=4)
        self.btn_stop = ttk.Button(
            top, text=u'READ停止', command=self.stop_refresh, state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, padx=4)

        self.progress = ttk.Progressbar(
            top, orient=tk.HORIZONTAL, mode='determinate', length=180)
        self.progress.pack(side=tk.RIGHT, padx=(10, 0))

        self.status_var = tk.StringVar(
            value=u'READは常時可。WRITE/SAVEはSTANDBY時のみOperator UIから許可します。')
        ttk.Label(
            self.root, textvariable=self.status_var,
            padding=(10, 0, 10, 6)).pack(fill=tk.X)

        body = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        left = ttk.Frame(body)
        body.add(left, weight=3)
        ttk.Label(left, text=u'軸一覧').pack(anchor=tk.W, pady=(0, 4))
        cols = ('axis', 'status', 'gear', 'dir', 'min', 'max', 'kp', 'ki', 'kd')
        self.summary = ttk.Treeview(left, columns=cols, show='headings', height=20)
        headers = {
            'axis': 'Axis', 'status': 'Status', 'gear': 'Gear', 'dir': 'Dir',
            'min': 'Min [deg]', 'max': 'Max [deg]',
            'kp': 'Kp', 'ki': 'Ki', 'kd': 'Kd'}
        widths = {
            'axis': 55, 'status': 110, 'gear': 70, 'dir': 55,
            'min': 85, 'max': 85, 'kp': 65, 'ki': 65, 'kd': 65}
        for col in cols:
            self.summary.heading(col, text=headers[col])
            self.summary.column(
                col, width=widths[col], minwidth=45, anchor=tk.CENTER)
        sy = ttk.Scrollbar(left, orient=tk.VERTICAL, command=self.summary.yview)
        sx = ttk.Scrollbar(left, orient=tk.HORIZONTAL, command=self.summary.xview)
        self.summary.configure(yscrollcommand=sy.set, xscrollcommand=sx.set)
        self.summary.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        sx.pack(side=tk.BOTTOM, fill=tk.X)
        sy.pack(side=tk.RIGHT, fill=tk.Y)
        self.summary.bind('<<TreeviewSelect>>', self._on_summary_select)
        for axis in self.axes:
            self.summary.insert(
                '', tk.END, iid=str(axis),
                values=(axis, u'未取得', '-', '-', '-', '-', '-', '-', '-'))

        right = ttk.Frame(body)
        body.add(right, weight=2)
        self.detail_title_var = tk.StringVar(value='Axis %s Detail' % self.axis_var.get())
        ttk.Label(right, textvariable=self.detail_title_var).pack(
            anchor=tk.W, pady=(0, 4))

        dcols = ('section', 'parameter', 'value', 'unit')
        self.detail = ttk.Treeview(right, columns=dcols, show='headings', height=20)
        self.detail.heading('section', text='Type')
        self.detail.heading('parameter', text='Parameter')
        self.detail.heading('value', text='Value')
        self.detail.heading('unit', text='Unit')
        self.detail.column('section', width=55, anchor=tk.CENTER)
        self.detail.column('parameter', width=175, anchor=tk.W)
        self.detail.column('value', width=220, anchor=tk.W)
        self.detail.column('unit', width=60, anchor=tk.CENTER)
        dy = ttk.Scrollbar(right, orient=tk.VERTICAL, command=self.detail.yview)
        self.detail.configure(yscrollcommand=dy.set)
        self.detail.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        dy.pack(side=tk.RIGHT, fill=tk.Y)
        self.detail.bind('<<TreeviewSelect>>', self._on_parameter_selected)

        editor = ttk.LabelFrame(right, text=u'選択パラメータ編集', padding=8)
        editor.pack(fill=tk.X, pady=(8, 0))
        self.edit_param_var = tk.StringVar(value=u'パラメータを選択してください')
        ttk.Label(editor, textvariable=self.edit_param_var).grid(
            row=0, column=0, columnspan=3, sticky=tk.W)
        ttk.Label(editor, text=u'書込値').grid(
            row=1, column=0, sticky=tk.W, pady=(6, 0))
        self.edit_value_var = tk.StringVar()
        self.edit_entry = ttk.Entry(editor, textvariable=self.edit_value_var, width=22)
        self.edit_entry.grid(row=1, column=1, sticky=tk.EW, padx=6, pady=(6, 0))
        self.edit_unit_var = tk.StringVar()
        ttk.Label(editor, textvariable=self.edit_unit_var).grid(
            row=1, column=2, sticky=tk.W, pady=(6, 0))
        self.btn_write = ttk.Button(
            editor, text=u'WRITE（RAM反映）',
            command=self.write_selected, state=tk.DISABLED)
        self.btn_write.grid(row=2, column=0, columnspan=3, sticky=tk.EW, pady=(8, 0))
        self.echo_var = tk.StringVar(value=u'Echo: -')
        ttk.Label(editor, textvariable=self.echo_var).grid(
            row=3, column=0, columnspan=3, sticky=tk.W, pady=(5, 0))
        editor.columnconfigure(1, weight=1)

        save_frame = ttk.LabelFrame(right, text=u'不揮発保存', padding=8)
        save_frame.pack(fill=tk.X, pady=(8, 0))
        self.btn_save_hw = ttk.Button(
            save_frame, text=u'HardwareConfig SAVE',
            command=lambda: self.save_selected_config(config.CONFIG_TYPE_HW))
        self.btn_save_hw.pack(fill=tk.X, pady=2)
        self.btn_save_sw = ttk.Button(
            save_frame, text=u'SoftwareConfig SAVE',
            command=lambda: self.save_selected_config(config.CONFIG_TYPE_SW))
        self.btn_save_sw.pack(fill=tk.X, pady=2)
        self.reboot_var = tk.StringVar(
            value=u'HardwareConfig SAVE後は電源再投入が必要です。')
        ttk.Label(
            save_frame, textvariable=self.reboot_var,
            wraplength=360).pack(anchor=tk.W, pady=(5, 0))

        self._show_axis_detail(self.axes[0])

    def _set_busy(self, busy, kind=None):
        if busy:
            self.worker_kind = kind or 'read'
        self.btn_selected.configure(state=tk.DISABLED if busy else tk.NORMAL)
        self.btn_all.configure(state=tk.DISABLED if busy else tk.NORMAL)
        self.axis_combo.configure(state='disabled' if busy else 'readonly')
        self.btn_stop.configure(
            state=tk.NORMAL if (busy and self.worker_kind == 'read') else tk.DISABLED)
        if busy:
            self.btn_write.configure(state=tk.DISABLED)
            self.btn_save_hw.configure(state=tk.DISABLED)
            self.btn_save_sw.configure(state=tk.DISABLED)
        else:
            self.worker_kind = None
            self._refresh_modify_gate()

    def _start_worker(self, target, args, kind):
        if self._worker_alive():
            return False
        self.stop_event.clear()
        self._set_busy(True, kind)
        if kind in ('write', 'save'):
            self._notify_modify_active(True)
        self.worker = threading.Thread(target=target, args=args)
        self.worker.daemon = True
        self.worker.start()
        return True

    def refresh_selected(self):
        axis = int(self.axis_var.get())
        self.progress['maximum'] = len(config.PARAMETERS)
        self.progress['value'] = 0
        self._start_worker(self._worker_refresh_axes, ([axis], False), 'read')

    def refresh_all(self):
        self.progress['maximum'] = max(1, len(self.axes))
        self.progress['value'] = 0
        self._start_worker(self._worker_refresh_axes, (list(self.axes), True), 'read')

    def stop_refresh(self):
        if self.worker_kind == 'read':
            self.stop_event.set()
            self.status_var.set(u'READ停止要求を送信しました。')

    def _worker_refresh_axes(self, axes, discovery_mode):
        try:
            for index, axis in enumerate(axes):
                if self.stop_event.is_set():
                    break
                self.events.put(('axis_status', axis, u'読取中'))
                self.events.put(('status', u'Axis %d を読取中...' % axis))
                values = {}
                if discovery_mode:
                    p = config.PARAM_BY_KEY['gear_ratio']
                    try:
                        values[p[0]] = self.can.read_parameter(
                            axis, p[2], p[3], p[4], timeout_sec=0.15)
                    except Exception:
                        self.events.put(('axis_data', axis, values, u'応答なし'))
                        self.events.put(('progress', index + 1))
                        continue
                ok = True
                error_text = ''
                for p in config.PARAMETERS:
                    if self.stop_event.is_set():
                        ok = False
                        error_text = u'停止'
                        break
                    key, section, cfg_type, param_id, value_type, label, unit = p
                    if key in values:
                        continue
                    try:
                        values[key] = self.can.read_parameter(
                            axis, cfg_type, param_id, value_type, timeout_sec=0.25)
                    except Exception as exc:
                        ok = False
                        error_text = str(exc)
                        break
                    if not discovery_mode:
                        self.events.put(('progress', len(values)))
                self.events.put(('axis_data', axis, values, 'OK' if ok else error_text))
                if discovery_mode:
                    self.events.put(('progress', index + 1))
            self.events.put(('done', u'READ完了。'))
        except Exception as exc:
            self.events.put(('operation_error', u'READ失敗', str(exc)))

    def write_selected(self):
        if not self.selected_param_key:
            return
        if not self._modification_allowed():
            messagebox.showwarning(
                u'WRITE禁止',
                u'RUNまたはMotion SEND中はMCU Config WRITEできません。\nSTOP/STANDBYで実行してください。')
            return
        axis = int(self.axis_var.get())
        p = config.PARAM_BY_KEY[self.selected_param_key]
        key, section, config_type, param_id, value_type, label, unit = p
        try:
            value = config.wire_value_from_edit(
                key, self.edit_value_var.get(), value_type)
            config.pack_value(value, value_type)
        except Exception as exc:
            messagebox.showerror(u'入力値エラー', str(exc))
            return
        if config_type == config.CONFIG_TYPE_HW:
            if not messagebox.askyesno(
                u'HardwareConfig WRITE確認',
                u'Axis %d の %s を %s %s にWRITEします。\n\nHardwareConfigはSAVE後に電源再投入が必要です。\nWRITEしますか？' %
                (axis, label, self.edit_value_var.get(), unit)):
                return
        self.echo_var.set(u'Echo: 送信中...')
        self._start_worker(self._worker_write, (axis, p, value), 'write')

    def _worker_write(self, axis, p, value):
        key, section, config_type, param_id, value_type, label, unit = p
        try:
            self.events.put(('status', u'Axis %d %s WRITE中...' % (axis, label)))
            echo_value = self.can.write_parameter(
                axis, config_type, param_id, value_type, value)
            readback = self.can.read_parameter(
                axis, config_type, param_id, value_type, timeout_sec=0.30)
            self.events.put(('write_ok', axis, key, echo_value, readback))
            self.events.put(('done', u'WRITE成功。Echo確認 + READ back完了。'))
        except Exception as exc:
            self.events.put(('operation_error', u'WRITE失敗', str(exc)))

    def save_selected_config(self, config_type):
        if not self._modification_allowed():
            messagebox.showwarning(
                u'SAVE禁止',
                u'RUNまたはMotion SEND中はMCU Config SAVEできません。\nSTOP/STANDBYで実行してください。')
            return
        axis = int(self.axis_var.get())
        name = u'HardwareConfig' if config_type == config.CONFIG_TYPE_HW else u'SoftwareConfig'
        extra = u'\n\n保存後は電源再投入が必要です。' if config_type == config.CONFIG_TYPE_HW else u''
        if not messagebox.askyesno(
            u'SAVE確認',
            u'Axis %d の%sをFlashへ保存します。%s\n\nSAVEしますか？' %
            (axis, name, extra)):
            return
        self._start_worker(self._worker_save, (axis, config_type), 'save')

    def _worker_save(self, axis, config_type):
        name = u'HardwareConfig' if config_type == config.CONFIG_TYPE_HW else u'SoftwareConfig'
        try:
            self.events.put(('status', u'Axis %d %s SAVE中...' % (axis, name)))
            self.can.save_config(axis, config_type)
            self.events.put(('save_ok', axis, config_type))
            self.events.put(('done', u'%s SAVE成功。' % name))
        except Exception as exc:
            self.events.put(('operation_error', u'SAVE失敗', str(exc)))

    def _poll_events(self):
        if self.closed:
            return
        try:
            while True:
                event = self.events.get_nowait()
                kind = event[0]
                if kind == 'axis_status':
                    _, axis, status = event
                    self.axis_status[axis] = status
                    self._update_summary_row(axis)
                elif kind == 'axis_data':
                    _, axis, values, status = event
                    self.axis_data[axis].update(values)
                    self.axis_status[axis] = status
                    self._update_summary_row(axis)
                    if str(axis) == self.axis_var.get():
                        self._show_axis_detail(axis)
                elif kind == 'progress':
                    self.progress['value'] = event[1]
                elif kind == 'status':
                    self.status_var.set(event[1])
                elif kind == 'write_ok':
                    _, axis, key, echo_value, readback = event
                    self.axis_data[axis][key] = readback
                    self.axis_status[axis] = 'OK'
                    self.echo_var.set(u'Echo: %s' % config.display_value(key, echo_value))
                    self._update_summary_row(axis)
                    if str(axis) == self.axis_var.get():
                        self._show_axis_detail(axis)
                        self.selected_param_key = key
                        self._select_detail_key(key)
                        self.edit_value_var.set(config.edit_value_from_wire(key, readback))
                elif kind == 'save_ok':
                    _, axis, config_type = event
                    if config_type == config.CONFIG_TYPE_HW:
                        self.hw_reboot_required_axes.add(axis)
                        self.reboot_var.set(
                            u'Axis %d: HardwareConfig SAVE成功。電源再投入が必要です。' % axis)
                        messagebox.showinfo(
                            u'SAVE成功',
                            u'HardwareConfigを保存しました。\n電源再投入が必要です。')
                    else:
                        messagebox.showinfo(u'SAVE成功', u'SoftwareConfigを保存しました。')
                    self._update_summary_row(axis)
                elif kind == 'operation_error':
                    _, title, text = event
                    self.status_var.set(u'%s: %s' % (title, text))
                    messagebox.showerror(title, text)
                    was_modify = self.worker_kind in ('write', 'save')
                    self._set_busy(False)
                    if was_modify:
                        self._notify_modify_active(False)
                elif kind == 'done':
                    text = event[1] if len(event) > 1 else u'完了。'
                    was_modify = self.worker_kind in ('write', 'save')
                    self._set_busy(False)
                    if was_modify:
                        self._notify_modify_active(False)
                    self.status_var.set(text)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_events)

    def _refresh_modify_gate(self):
        if self.closed:
            return
        if not self._worker_alive():
            allowed = self._modification_allowed()
            self.btn_save_hw.configure(state=tk.NORMAL if allowed else tk.DISABLED)
            self.btn_save_sw.configure(state=tk.NORMAL if allowed else tk.DISABLED)
            self.btn_write.configure(
                state=tk.NORMAL if (allowed and self.selected_param_key) else tk.DISABLED)
            if not allowed:
                self.status_var.set(
                    u'READは可。RUNまたはMotion SEND中のためWRITE/SAVEはロック中。')
        self.root.after(150, self._refresh_modify_gate)

    def _update_summary_row(self, axis):
        data = self.axis_data.get(axis, {})
        def deg_or_dash(key):
            if key not in data:
                return '-'
            return '%.2f' % (data[key] * 180.0 / 3.141592653589793)
        status = self.axis_status.get(axis, u'未取得')
        if axis in self.hw_reboot_required_axes and status == 'OK':
            status += u' / 要再起動'
        values = (
            axis, status,
            ('%.4g' % data['gear_ratio']) if 'gear_ratio' in data else '-',
            str(data['motor_direction']) if 'motor_direction' in data else '-',
            deg_or_dash('joint_min_rad'), deg_or_dash('joint_max_rad'),
            str(data['kp']) if 'kp' in data else '-',
            str(data['ki']) if 'ki' in data else '-',
            str(data['kd']) if 'kd' in data else '-')
        self.summary.item(str(axis), values=values)

    def _show_axis_detail(self, axis):
        self.detail_title_var.set('Axis %d Detail' % axis)
        for item in self.detail.get_children():
            self.detail.delete(item)
        data = self.axis_data.get(axis, {})
        for p in config.PARAMETERS:
            key, section, cfg_type, param_id, value_type, label, unit = p
            value = config.display_value(key, data[key]) if key in data else '-'
            shown_unit = unit
            if key in ('joint_min_rad', 'joint_max_rad', 'pos_jump_rad',
                       'pos_error_rad', 'can_termination'):
                shown_unit = ''
            self.detail.insert(
                '', tk.END, iid=key,
                values=(section, label, value, shown_unit))
        if self.selected_param_key and self.detail.exists(self.selected_param_key):
            self._select_detail_key(self.selected_param_key)

    def _select_detail_key(self, key):
        try:
            self.detail.selection_set(key)
            self.detail.see(key)
        except Exception:
            pass

    def _on_parameter_selected(self, _event=None):
        selected = self.detail.selection()
        if not selected:
            return
        key = selected[0]
        if key not in config.PARAM_BY_KEY:
            return
        self.selected_param_key = key
        p = config.PARAM_BY_KEY[key]
        self.edit_param_var.set(u'%s / %s' % (p[1], p[5]))
        self.edit_unit_var.set(p[6])
        axis = int(self.axis_var.get())
        if key in self.axis_data.get(axis, {}):
            self.edit_value_var.set(
                config.edit_value_from_wire(key, self.axis_data[axis][key]))
        else:
            self.edit_value_var.set('')
        self._refresh_modify_gate()

    def _on_axis_selected(self, _event=None):
        axis = int(self.axis_var.get())
        self._show_axis_detail(axis)
        self.summary.selection_set(str(axis))
        self.summary.see(str(axis))
        if axis in self.hw_reboot_required_axes:
            self.reboot_var.set(
                u'Axis %d: HardwareConfig SAVE済み。電源再投入が必要です。' % axis)
        else:
            self.reboot_var.set(u'HardwareConfig SAVE後は電源再投入が必要です。')
        if self.selected_param_key:
            self._on_parameter_selected()

    def _on_summary_select(self, _event=None):
        selected = self.summary.selection()
        if not selected:
            return
        axis = int(selected[0])
        self.axis_var.set(str(axis))
        self._show_axis_detail(axis)
        if axis in self.hw_reboot_required_axes:
            self.reboot_var.set(
                u'Axis %d: HardwareConfig SAVE済み。電源再投入が必要です。' % axis)
        else:
            self.reboot_var.set(u'HardwareConfig SAVE後は電源再投入が必要です。')

    def close(self):
        self.closed = True
        self.stop_event.set()
        if self.modify_active:
            self._notify_modify_active(False)
        try:
            self.can.close()
        except Exception:
            pass
