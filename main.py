"""
main.py - the ground-station window (the "view + controller").

Data flow
    keyboard / buttons --signals--> SimWorker (own thread) --Telemetry signal--> on_telemetry()
                                    owns RovSimulator                              |
                                                                  labels, plot, map, thrust bars

The window holds NO vehicle logic. It only (a) turns clicks/keys into requests
and (b) draws whatever telemetry arrives. Run with:  python main.py
"""
from __future__ import annotations

import sys
from collections import deque
from datetime import datetime
from html import escape
from pathlib import Path

import pyqtgraph as pg
from PySide6.QtCore import QEvent, QMetaObject, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPainter
from PySide6.QtWidgets import (
    QAbstractSpinBox, QApplication, QButtonGroup, QDoubleSpinBox, QFrame, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPushButton, QScrollArea,
    QSlider, QSplitter, QTextEdit, QVBoxLayout, QWidget)

from map_widget import MapWidget
from rov_sim import HEAVE, SURGE, SWAY, YAW, Mode, Telemetry
from sim_worker import SimWorker

# Start point of the simulated dive: a spot near Ulsoor Lake, Bangalore (approximate).
HOME_LAT, HOME_LON = 12.9835, 77.6195
PLOT_WINDOW_S = 60.0
TELEMETRY_HZ = 25
# X11 sends a fake release+press pair for every auto-repeat of a held key, and Qt does not always
# flag them as auto-repeat. A release is therefore only acted on if no new press follows quickly.
RELEASE_DEBOUNCE_MS = 50

# key -> (axis, direction). Heave is DOWN-positive: F dives, R rises.
KEYMAP = {
    Qt.Key.Key_W: (SURGE, +1), Qt.Key.Key_Up: (SURGE, +1),
    Qt.Key.Key_S: (SURGE, -1), Qt.Key.Key_Down: (SURGE, -1),
    Qt.Key.Key_D: (SWAY, +1), Qt.Key.Key_Right: (SWAY, +1),
    Qt.Key.Key_A: (SWAY, -1), Qt.Key.Key_Left: (SWAY, -1),
    Qt.Key.Key_F: (HEAVE, +1), Qt.Key.Key_R: (HEAVE, -1),
    Qt.Key.Key_E: (YAW, +1), Qt.Key.Key_Q: (YAW, -1),
}
KEYMAP = {int(k): v for k, v in KEYMAP.items()}
KEY_SPACE = int(Qt.Key.Key_Space)

STYLE = """
QWidget { background:#14181f; color:#dbe2ea; font-size:13px; }
QGroupBox { border:1px solid #2b323d; border-radius:6px; margin-top:14px; padding:10px 6px 6px 6px; font-weight:600; }
QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 4px; color:#8fa3b8; }
QPushButton { background:#232a35; border:1px solid #394353; border-radius:5px; padding:7px 10px; }
QPushButton:hover { background:#2c3544; }
QPushButton:checked { background:#1f6feb; border-color:#388bfd; color:white; }
QPushButton:disabled { color:#5b6573; background:#1a1f27; border-color:#262d38; }
QPushButton#armBtn:enabled { background:#1d6f42; border-color:#2ea043; color:white; font-weight:700; }
QPushButton#disarmBtn:enabled { background:#a4161a; border-color:#e5383b; color:white; font-weight:700; }
QDoubleSpinBox { background:#1b212b; border:1px solid #394353; border-radius:4px; padding:4px; }
QPushButton#linkBtn:checked { background:#b45309; border-color:#f59e0b; color:white; font-weight:700; }
QFrame#tile { background:#1b212b; border:1px solid #2b323d; border-radius:6px; }
QSplitter::handle { background:#2b323d; }
QStatusBar { color:#8fa3b8; }
QScrollBar:vertical { width:10px; background:#14181f; }
QScrollBar::handle:vertical { background:#394353; border-radius:4px; min-height:24px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0; }
"""


class Readout(QFrame):
    """A small tile: caption on top, big value underneath."""

    def __init__(self, caption: str):
        super().__init__()
        self.setObjectName("tile")
        self.setMinimumHeight(50)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(0)
        cap = QLabel(caption)
        cap.setStyleSheet("color:#8fa3b8; font-size:11px; background:transparent;")
        self.value = QLabel("--")
        font = QFont("Consolas")
        font.setStyleHint(QFont.StyleHint.Monospace)
        font.setPointSize(14)
        font.setBold(True)
        self.value.setFont(font)
        self.value.setStyleSheet("background:transparent;")
        lay.addWidget(cap)
        lay.addWidget(self.value)

    def set(self, text: str, color: str | None = None) -> None:
        self.value.setText(text)
        self.value.setStyleSheet(f"background:transparent; color:{color or '#dbe2ea'};")


class ThrustBar(QWidget):
    """Centre-zero bar showing actual thrust (-100%..+100%) for one axis."""

    def __init__(self, name: str):
        super().__init__()
        self.name, self.value = name, 0.0
        self.setFixedHeight(20)

    def set_value(self, v: float) -> None:
        if abs(v - self.value) > 1e-3:
            self.value = v
            self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        w, h = self.width(), self.height()
        left, right = 52, 44
        bar_w = w - left - right
        p.fillRect(left, 3, bar_w, h - 6, QColor("#232a35"))
        mid = left + bar_w / 2
        fill = self.value * bar_w / 2
        p.fillRect(int(min(mid, mid + fill)), 3, int(abs(fill)), h - 6,
                   QColor("#2ec4b6" if self.value >= 0 else "#ff9f1c"))
        p.setPen(QColor("#5b6573"))
        p.drawLine(int(mid), 1, int(mid), h - 1)
        p.setPen(QColor("#8fa3b8"))
        p.drawText(0, 0, left - 4, h, Qt.AlignmentFlag.AlignVCenter, self.name)
        p.setPen(QColor("#dbe2ea"))
        p.drawText(w - right + 4, 0, right - 4, h, Qt.AlignmentFlag.AlignVCenter,
                   f"{self.value * 100:+.0f}%")


class MainWindow(QMainWindow):
    # Requests to the simulation thread
    arm_requested = Signal()
    disarm_requested = Signal()
    mode_requested = Signal(str)
    depth_target_requested = Signal(float)
    heading_target_requested = Signal(float)
    manual_requested = Signal(float, float, float, float)
    logging_requested = Signal(str)
    logging_stop_requested = Signal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROV Telemetry Ground Station")
        self.resize(1360, 860)
        self.setMinimumSize(1100, 700)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setStyleSheet(STYLE)

        self._held: set[int] = set()
        self._release_timers: dict[int, QTimer] = {}
        self._latest: Telemetry | None = None
        self._logging = False
        self._was_armed: bool | None = None
        self._link_lost = False                  # failsafe test: operator commands not delivered
        n = int(PLOT_WINDOW_S * TELEMETRY_HZ * 1.2)
        self._ts: deque = deque(maxlen=n)
        self._ds: deque = deque(maxlen=n)

        root = QWidget()
        self.setCentralWidget(root)
        outer = QHBoxLayout(root)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)
        scroll = QScrollArea()
        scroll.setWidget(self._build_left_panel())
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setFixedWidth(372)
        outer.addWidget(scroll)
        outer.addWidget(self._build_right_panel(), 1)
        self.statusBar().showMessage("Starting...")

        # Keyboard: an application-wide filter so keys work wherever focus is
        QApplication.instance().installEventFilter(self)
        QApplication.instance().focusChanged.connect(self._on_focus_changed)
        # While a key is held we re-send the command at 20 Hz. This is the
        # "operator input" heartbeat that keeps the 1 s watchdog satisfied.
        self._heartbeat = QTimer(self)
        self._heartbeat.setInterval(50)
        self._heartbeat.timeout.connect(self._on_heartbeat)
        self._heartbeat.start()

    # ================================================================ layout
    def _build_left_panel(self) -> QWidget:
        panel = QWidget()
        col = QVBoxLayout(panel)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(6)

        # --- status ---
        box = QGroupBox("Vehicle status")
        v = QVBoxLayout(box)
        self.armed_badge = QLabel("DISARMED")
        self.armed_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.armed_badge.setFixedHeight(34)
        v.addWidget(self.armed_badge)
        self.mode_label = QLabel("Mode: Manual")
        self.mode_label.setStyleSheet("font-weight:600;")
        v.addWidget(self.mode_label)
        grid = QGridLayout()
        grid.setSpacing(6)
        self.r_roll, self.r_pitch, self.r_yaw = Readout("ROLL"), Readout("PITCH"), Readout("YAW")
        self.r_depth, self.r_batt = Readout("DEPTH"), Readout("BATTERY")
        grid.addWidget(self.r_roll, 0, 0)
        grid.addWidget(self.r_pitch, 0, 1)
        grid.addWidget(self.r_yaw, 0, 2)
        grid.addWidget(self.r_depth, 1, 0, 1, 1)
        grid.addWidget(self.r_batt, 1, 1, 1, 2)
        v.addLayout(grid)
        self.pos_label = QLabel("Position  N +0.0 m   E +0.0 m   Speed 0.00 m/s")
        self.pos_label.setStyleSheet("color:#8fa3b8;")
        v.addWidget(self.pos_label)
        col.addWidget(box)

        # --- arm / disarm + mode ---
        box = QGroupBox("Control   (Space = emergency disarm)")
        v = QVBoxLayout(box)
        h = QHBoxLayout()
        self.arm_btn, self.disarm_btn = QPushButton("ARM"), QPushButton("DISARM")
        self.arm_btn.setObjectName("armBtn")
        self.disarm_btn.setObjectName("disarmBtn")
        for b in (self.arm_btn, self.disarm_btn):
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)     # keep keyboard focus on the window
            b.setMinimumHeight(34)
            h.addWidget(b)
        self.arm_btn.clicked.connect(self.arm_requested)
        self.disarm_btn.clicked.connect(self.disarm_requested)
        v.addLayout(h)
        h = QHBoxLayout()
        self._mode_group = QButtonGroup(self)
        self._mode_buttons: dict[Mode, QPushButton] = {}
        for m in Mode:
            b = QPushButton(m.value)
            b.setCheckable(True)
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            b.clicked.connect(lambda _=False, mode=m: self._on_mode_clicked(mode))
            self._mode_group.addButton(b)
            self._mode_buttons[m] = b
            h.addWidget(b)
        self._mode_buttons[Mode.MANUAL].setChecked(True)
        v.addLayout(h)
        col.addWidget(box)

        # --- targets ---
        box = QGroupBox("Hold targets   (Enter or Set to apply)")
        g = QGridLayout(box)
        self.depth_spin = QDoubleSpinBox()
        self.depth_spin.setRange(0.0, 50.0)
        self.depth_spin.setDecimals(1)
        self.depth_spin.setSingleStep(0.5)
        self.depth_spin.setSuffix(" m")
        self.heading_spin = QDoubleSpinBox()
        self.heading_spin.setRange(0.0, 359.9)
        self.heading_spin.setDecimals(1)
        self.heading_spin.setSingleStep(5.0)
        self.heading_spin.setWrapping(True)
        self.heading_spin.setSuffix(" \u00b0")
        for sp in (self.depth_spin, self.heading_spin):
            sp.setFocusPolicy(Qt.FocusPolicy.ClickFocus)    # never grabs focus on its own
        set_d, set_h = QPushButton("Set"), QPushButton("Set")
        for b in (set_d, set_h):
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        g.addWidget(QLabel("Target depth"), 0, 0)
        g.addWidget(self.depth_spin, 0, 1)
        g.addWidget(set_d, 0, 2)
        g.addWidget(QLabel("Target heading"), 1, 0)
        g.addWidget(self.heading_spin, 1, 1)
        g.addWidget(set_h, 1, 2)
        self.target_label = QLabel("")
        self.target_label.setStyleSheet("color:#8fa3b8;")
        g.addWidget(self.target_label, 2, 0, 1, 3)
        set_d.clicked.connect(self._apply_depth)
        set_h.clicked.connect(self._apply_heading)
        self.depth_spin.editingFinished.connect(self._apply_depth)
        self.heading_spin.editingFinished.connect(self._apply_heading)
        col.addWidget(box)

        # --- manual control + thrust display ---
        box = QGroupBox("Manual control")
        v = QVBoxLayout(box)
        keys = QLabel("W/S fwd/back   A/D left/right\nF dive   R rise   Q/E turn   (arrow keys also work)")
        keys.setStyleSheet("color:#8fa3b8; font-size:11px;")
        v.addWidget(keys)
        self.kb_label = QLabel("Keyboard control: ACTIVE")
        self.kb_label.setStyleSheet("color:#2ec4b6; font-weight:600;")
        v.addWidget(self.kb_label)
        h = QHBoxLayout()
        h.addWidget(QLabel("Key thrust"))
        self.power = QSlider(Qt.Orientation.Horizontal)
        self.power.setRange(10, 100)
        self.power.setValue(60)
        self.power.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.power_label = QLabel("60%")
        self.power.valueChanged.connect(lambda x: self.power_label.setText(f"{x}%"))
        h.addWidget(self.power, 1)
        h.addWidget(self.power_label)
        v.addLayout(h)
        self.bars = {SURGE: ThrustBar("Surge"), SWAY: ThrustBar("Sway"),
                     HEAVE: ThrustBar("Heave"), YAW: ThrustBar("Yaw")}
        for bar in self.bars.values():
            v.addWidget(bar)
        col.addWidget(box)

        # --- logging + failsafe test ---
        row = QHBoxLayout()
        self.log_btn = QPushButton("Start CSV log")
        self.log_btn.setCheckable(True)
        self.log_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.log_btn.clicked.connect(self._toggle_logging)
        self.link_btn = QPushButton("Simulate link loss")
        self.link_btn.setObjectName("linkBtn")
        self.link_btn.setCheckable(True)
        self.link_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.link_btn.setToolTip("Stops delivering operator commands so you can watch the 1 s watchdog "
                                 "cut thrust. Space (emergency disarm) still works.")
        self.link_btn.clicked.connect(self._toggle_link_loss)
        row.addWidget(self.log_btn)
        row.addWidget(self.link_btn)
        col.addLayout(row)
        col.addStretch(1)
        return panel

    def _build_right_panel(self) -> QWidget:
        split = QSplitter(Qt.Orientation.Vertical)
        # depth vs time plot
        self.plot = pg.PlotWidget(background="#1b212b")
        item = self.plot.getPlotItem()
        item.setTitle("Depth vs time (last 60 s)", color="#8fa3b8", size="11pt")
        item.setLabel("left", "Depth", units="m")
        item.setLabel("bottom", "Time", units="s")
        item.showGrid(x=True, y=True, alpha=0.25)
        item.invertY(True)                       # depth increases DOWNWARDS
        item.setMouseEnabled(False, False)
        item.hideButtons()
        self.curve = item.plot(pen=pg.mkPen("#2ec4b6", width=2))
        self.target_line = pg.InfiniteLine(angle=0, pen=pg.mkPen("#ff9f1c", width=1.5,
                                                                 style=Qt.PenStyle.DashLine))
        self.target_line.setVisible(False)
        item.addItem(self.target_line)
        split.addWidget(self.plot)
        # map
        self.map = MapWidget(HOME_LAT, HOME_LON, zoom=19)
        split.addWidget(self.map)
        # event log: why did something happen? (arm/disarm, mode, watchdog, battery failsafes)
        self.event_log = QTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.event_log.document().setMaximumBlockCount(300)
        self.event_log.setPlaceholderText("Events and failsafe messages appear here")
        self.event_log.setStyleSheet(
            "background:#1b212b; border:1px solid #2b323d; font-family:'DejaVu Sans Mono',Consolas,monospace;"
            " font-size:12px;")
        split.addWidget(self.event_log)
        split.setSizes([320, 400, 120])
        return split

    # ================================================================ operator actions
    def _on_mode_clicked(self, mode: Mode) -> None:
        self.mode_requested.emit(mode.value)
        if self._latest:                         # pre-fill spinners with the current state
            self.depth_spin.setValue(round(self._latest.depth, 1))
            self.heading_spin.setValue(round(self._latest.yaw, 1) % 360.0)

    def _apply_depth(self) -> None:
        self.depth_target_requested.emit(self.depth_spin.value())
        self.setFocus()                          # give keys back to the window

    def _apply_heading(self) -> None:
        self.heading_target_requested.emit(self.heading_spin.value())
        self.setFocus()

    def _toggle_link_loss(self) -> None:
        self._link_lost = self.link_btn.isChecked()
        self.link_btn.setText("Restore link" if self._link_lost else "Simulate link loss")
        t = self._latest.t if self._latest else 0.0
        self._add_event(t, "LINK LOSS SIMULATED: operator commands no longer delivered (Space still disarms)"
                        if self._link_lost else "LINK RESTORED: operator commands delivered again")

    def on_event(self, t: float, message: str) -> None:
        self._add_event(t, message)

    def _add_event(self, t: float, message: str) -> None:
        if message.startswith(("WATCHDOG", "ARM REFUSED", "LINK LOSS", "DISARMED (battery")):
            colour = "#e5383b"                                   # red: a failsafe acted
        elif message.startswith(("BATTERY LOW", "DISARMED")):
            colour = "#ff9f1c"                                   # amber: warning / stop
        else:
            colour = "#9fb3c8"
        stamp = f"{int(t // 60):02d}:{t % 60:04.1f}"
        self.event_log.append(f'<span style="color:#5b6573">[{stamp}]</span> '
                              f'<span style="color:{colour}">{escape(message)}</span>')

    def _toggle_logging(self) -> None:
        if self.log_btn.isChecked():
            name = datetime.now().strftime("rov_%Y%m%d_%H%M%S.csv")
            self.logging_requested.emit(str(Path(__file__).parent / "logs" / name))
        else:
            self.logging_stop_requested.emit()

    def on_logging_changed(self, active: bool, text: str) -> None:
        self._logging = active
        self.log_btn.setChecked(active)
        self.log_btn.setText("Stop CSV log" if active else "Start CSV log")
        if active:
            self.statusBar().showMessage(f"Logging to {text}")
        elif text:
            self.statusBar().showMessage(text)
        else:
            self.statusBar().showMessage("Log saved.")

    # ================================================================ keyboard
    def _on_focus_changed(self, _old, new) -> None:
        typing = isinstance(new, (QAbstractSpinBox, QLineEdit))
        self.kb_label.setText("Keyboard control: PAUSED (typing - press Enter)" if typing
                              else "Keyboard control: ACTIVE")
        self.kb_label.setStyleSheet(
            f"color:{'#ff9f1c' if typing else '#2ec4b6'}; font-weight:600;")

    def eventFilter(self, obj, ev):
        et = ev.type()
        if et in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            # Don't hijack typing in the target boxes
            if isinstance(QApplication.focusWidget(), (QAbstractSpinBox, QLineEdit)):
                return False
            key = ev.key()
            if key == KEY_SPACE:
                if et == QEvent.Type.KeyPress and not ev.isAutoRepeat():
                    self.disarm_requested.emit()          # emergency stop
                return True
            if key in KEYMAP:
                if et == QEvent.Type.KeyPress:
                    self._cancel_release(key)             # a press right after a release = auto-repeat
                    if key not in self._held:
                        self._held.add(key)
                        self._send_manual()
                elif not ev.isAutoRepeat():
                    self._schedule_release(key)
                return True
        elif et == QEvent.Type.WindowDeactivate and obj is self:
            for k in list(self._release_timers):          # lost focus with a key down -> neutral
                self._cancel_release(k)
            self._held.clear()
            self._send_manual()
        return super().eventFilter(obj, ev)

    def _schedule_release(self, key: int) -> None:
        if key in self._release_timers:
            return
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.setInterval(RELEASE_DEBOUNCE_MS)
        timer.timeout.connect(lambda k=key: self._finish_release(k))
        self._release_timers[key] = timer
        timer.start()

    def _cancel_release(self, key: int) -> None:
        timer = self._release_timers.pop(key, None)
        if timer:
            timer.stop()
            timer.deleteLater()

    def _finish_release(self, key: int) -> None:
        self._release_timers.pop(key, None)
        self._held.discard(key)
        self._send_manual()                               # sends neutral once everything is released

    def _send_manual(self) -> None:
        if self._link_lost:                       # failsafe test: pretend the link is down
            return
        cmd = [0.0] * 4
        for k in self._held:
            axis, sign = KEYMAP[k]
            cmd[axis] += sign
        scale = self.power.value() / 100.0
        self.manual_requested.emit(*(max(-1.0, min(1.0, c)) * scale for c in cmd))

    def _on_heartbeat(self) -> None:
        if self._held:
            self._send_manual()

    # ================================================================ telemetry -> screen
    def on_telemetry(self, t: Telemetry) -> None:
        self._latest = t

        if t.armed:
            self.armed_badge.setText("ARMED")
            self.armed_badge.setStyleSheet(
                "background:#a4161a; color:white; font-size:16px; font-weight:700; border-radius:5px;")
        else:
            self.armed_badge.setText("DISARMED")
            self.armed_badge.setStyleSheet(
                "background:#232a35; color:#8fa3b8; font-size:16px; font-weight:700; border-radius:5px;")
        self.arm_btn.setEnabled(not t.armed)
        self.disarm_btn.setEnabled(t.armed)
        if t.armed != self._was_armed:                      # only when the state changes
            self._was_armed = t.armed
            if not self._logging:
                self.statusBar().showMessage(
                    "Armed. Manual: W/A/S/D move, R/F up/down, Q/E turn. Space = emergency disarm."
                    if t.armed else "Disarmed: thrust is forced to zero. Click ARM to enable the thrusters.")

        self.mode_label.setText(f"Mode: {t.mode.value}")
        self._mode_buttons[t.mode].setChecked(True)
        if t.mode == Mode.DEPTH_HOLD:
            self.target_label.setText(f"Holding depth {t.target_depth:.1f} m")
        elif t.mode == Mode.HEADING_HOLD:
            self.target_label.setText(f"Holding heading {t.target_heading:.0f}\u00b0")
        else:
            self.target_label.setText("Targets are used in the hold modes")

        self.r_roll.set(f"{t.roll:+.1f}\u00b0")
        self.r_pitch.set(f"{t.pitch:+.1f}\u00b0")
        self.r_yaw.set(f"{t.yaw:05.1f}\u00b0")
        self.r_depth.set(f"{t.depth:.2f} m")
        colour = "#e5383b" if t.battery_v < 14.4 else "#ff9f1c" if t.battery_v < 15.2 else None
        self.r_batt.set(f"{t.battery_v:.2f} V", colour)
        self.pos_label.setText(
            f"Position  N {t.north:+.1f} m   E {t.east:+.1f} m   Speed {t.speed:.2f} m/s")
        for axis, bar in self.bars.items():
            bar.set_value(t.thrust[axis])

        # depth plot
        self._ts.append(t.t)
        self._ds.append(t.depth)
        self.curve.setData(list(self._ts), list(self._ds))
        item = self.plot.getPlotItem()
        item.setXRange(max(0.0, t.t - PLOT_WINDOW_S), max(PLOT_WINDOW_S, t.t), padding=0)
        ymax = max(2.0, max(self._ds) * 1.15)
        if t.mode == Mode.DEPTH_HOLD:
            self.target_line.setValue(t.target_depth)
            ymax = max(ymax, t.target_depth * 1.15)
        self.target_line.setVisible(t.mode == Mode.DEPTH_HOLD)
        item.setYRange(-0.08 * ymax, ymax, padding=0)   # small margin so depth 0 is visible

        self.map.set_position(t.north, t.east, t.yaw)


def _check_pyside_refcount_bug() -> None:
    """Some PySide6 builds on Python 3.10/3.11 leak references to None on every call that returns
    nothing (e.g. PySide6 6.12.0 on 3.10, and 6.5.3-6.7.0). The interpreter then aborts with
    'Fatal Python error: none_dealloc' after a few seconds of running. Detect it up front and
    explain the fix, instead of crashing mysteriously later."""
    import PySide6
    from PySide6.QtCore import QObject
    probe = QObject()
    before = sys.getrefcount(None)
    for _ in range(300):
        probe.setObjectName("probe")          # returns None
    if sys.getrefcount(None) - before < -100:
        sys.exit(
            f"ERROR: PySide6 {PySide6.__version__} on Python {sys.version.split()[0]} has a known "
            "reference-counting bug and would crash.\n"
            'Fix:  pip install "PySide6>=6.8,<6.12"   (best inside a fresh virtual environment)')


def create_parts(argv=None):
    """Builds app, window and the simulation thread, and wires them together."""
    app = QApplication.instance() or QApplication(argv if argv is not None else sys.argv)
    app.setApplicationName("ROVGroundStation")
    pg.setConfigOptions(antialias=True)

    win = MainWindow()
    worker = SimWorker()
    thread = QThread()
    worker.moveToThread(thread)

    # UI -> worker (queued automatically because they live in different threads)
    win.arm_requested.connect(worker.arm)
    win.disarm_requested.connect(worker.disarm)
    win.mode_requested.connect(worker.set_mode)
    win.depth_target_requested.connect(worker.set_target_depth)
    win.heading_target_requested.connect(worker.set_target_heading)
    win.manual_requested.connect(worker.set_manual_command)
    win.logging_requested.connect(worker.start_logging)
    win.logging_stop_requested.connect(worker.stop_logging)
    # worker -> UI
    worker.telemetry.connect(win.on_telemetry)
    worker.logging_changed.connect(win.on_logging_changed)
    worker.event_logged.connect(win.on_event)

    thread.started.connect(worker.start)

    def shutdown() -> None:
        if thread.isRunning():
            QMetaObject.invokeMethod(worker, "shutdown", Qt.ConnectionType.BlockingQueuedConnection)
            thread.quit()
            thread.wait(3000)

    app.aboutToQuit.connect(shutdown)
    return app, win, worker, thread, shutdown


def main() -> int:
    _check_pyside_refcount_bug()
    app, win, _worker, thread, _shutdown = create_parts()
    thread.start()
    win.show()
    win.setFocus()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
