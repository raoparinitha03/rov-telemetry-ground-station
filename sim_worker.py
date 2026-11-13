"""
sim_worker.py - runs the simulator on its OWN THREAD and talks to the UI only
through Qt signals/slots.

Why a separate thread?  The UI thread must never be blocked by physics (or by
disk writes for CSV logging), and the physics must keep a steady 50 Hz even
while the UI is busy repainting the map or plot.

Thread-safety without locks: the UI never calls the simulator directly. It
emits signals; Qt queues them and runs the slots below inside the worker
thread. So the simulator is only ever touched by ONE thread.
"""
from __future__ import annotations

import csv
import time
from pathlib import Path

from PySide6.QtCore import QObject, QTimer, Qt, Signal, Slot

from rov_sim import Mode, RovSimulator, Telemetry

SIM_DT = 0.02          # 50 Hz fixed physics step
EMIT_EVERY = 2         # send telemetry every 2nd step -> 25 Hz to the UI
MAX_CATCHUP = 0.25     # s; if the thread stalls, don't try to simulate more than this at once


class SimWorker(QObject):
    telemetry = Signal(object)            # Telemetry snapshot (25 Hz)
    logging_changed = Signal(bool, str)   # (is_logging, file path or error text)
    event_logged = Signal(float, str)     # (sim time, message) for the event log

    def __init__(self, sim: RovSimulator | None = None):
        super().__init__()
        self.sim = sim or RovSimulator()
        self._timer: QTimer | None = None
        self._last = 0.0
        self._accum = 0.0
        self._steps = 0
        self._log_file = None
        self._log_writer = None

    # ---- lifecycle ---------------------------------------------------------
    @Slot()
    def start(self) -> None:
        # The QTimer is created HERE (not in __init__) so it belongs to the worker thread.
        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.setInterval(10)
        self._timer.timeout.connect(self._tick)
        self._last = time.perf_counter()
        self._timer.start()
        self.telemetry.emit(self.sim.telemetry())
        self._forward_events()

    @Slot()
    def shutdown(self) -> None:
        if self._timer:
            self._timer.stop()
        self.stop_logging()

    def _tick(self) -> None:
        """Fixed-timestep loop: simulate in exact SIM_DT chunks, however irregular the timer is."""
        now = time.perf_counter()
        self._accum += min(now - self._last, MAX_CATCHUP)
        self._last = now
        while self._accum >= SIM_DT:
            self.sim.step(SIM_DT)
            self._accum -= SIM_DT
            self._steps += 1
            if self._steps % EMIT_EVERY == 0:
                snap = self.sim.telemetry()
                self.telemetry.emit(snap)
                self._log(snap)
        self._forward_events()

    def _forward_events(self) -> None:
        for t, message in self.sim.pop_events():
            self.event_logged.emit(t, message)

    # ---- commands from the UI (all run in the worker thread) ---------------
    @Slot()
    def arm(self) -> None:
        self.sim.arm()

    @Slot()
    def disarm(self) -> None:
        self.sim.disarm()

    @Slot(str)
    def set_mode(self, mode_name: str) -> None:
        self.sim.set_mode(Mode(mode_name))

    @Slot(float)
    def set_target_depth(self, metres: float) -> None:
        self.sim.set_target_depth(metres)

    @Slot(float)
    def set_target_heading(self, deg: float) -> None:
        self.sim.set_target_heading(deg)

    @Slot(float, float, float, float)
    def set_manual_command(self, surge: float, sway: float, heave: float, yaw: float) -> None:
        self.sim.set_manual_command(surge, sway, heave, yaw)

    # ---- optional CSV logging ---------------------------------------------
    @Slot(str)
    def start_logging(self, path: str) -> None:
        self.stop_logging()
        try:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            self._log_file = open(path, "w", newline="")
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow([
                "t_s", "armed", "mode", "roll_deg", "pitch_deg", "yaw_deg", "depth_m",
                "battery_v", "north_m", "east_m", "thr_surge", "thr_sway", "thr_heave",
                "thr_yaw", "target_depth_m", "target_heading_deg", "speed_mps"])
            self.logging_changed.emit(True, path)
        except OSError as exc:
            self._log_file = self._log_writer = None
            self.logging_changed.emit(False, f"Could not open log file: {exc}")

    @Slot()
    def stop_logging(self) -> None:
        if self._log_file:
            self._log_file.close()
            self._log_file = self._log_writer = None
            self.logging_changed.emit(False, "")

    def _log(self, t: Telemetry) -> None:
        if not self._log_writer:
            return
        self._log_writer.writerow([
            f"{t.t:.2f}", int(t.armed), t.mode.value, f"{t.roll:.2f}", f"{t.pitch:.2f}",
            f"{t.yaw:.2f}", f"{t.depth:.3f}", f"{t.battery_v:.2f}", f"{t.north:.2f}",
            f"{t.east:.2f}", *(f"{x:.3f}" for x in t.thrust),
            f"{t.target_depth:.2f}", f"{t.target_heading:.1f}", f"{t.speed:.3f}"])
