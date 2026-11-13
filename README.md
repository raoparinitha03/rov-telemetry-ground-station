# ROV Telemetry Ground Station

A native desktop application (Python + Qt/PySide6) that simulates a small underwater vehicle (ROV)
and lets an operator monitor and control it. No hardware, no browser, no web server.

## Setup and run

Requires Python 3.10 or newer and an internet connection the first time (to download map tiles).

```bash
python -m venv .venv
# Windows:  .venv\Scripts\activate        Linux/macOS:  source .venv/bin/activate
pip install -r requirements.txt
python main.py
```

Run the unit tests (no GUI needed):

```bash
python -m unittest -v
```

### Troubleshooting

* `python` not found on Windows: use `py -3` instead (e.g. `py -3 -m venv .venv`).
* `Fatal Python error: none_dealloc`, or the app exits saying PySide6 has a reference-counting bug:
  some PySide6 builds leak references to `None` on Python 3.10/3.11 (seen with PySide6 6.12.0 on
  Python 3.10; also 6.5.3-6.7.0). Install a good version: `pip install "PySide6>=6.8,<6.12"`
  (best inside a fresh venv). `requirements.txt` already pins this range.
* PowerShell says "running scripts is disabled" when activating the venv: run
  `Set-ExecutionPolicy -Scope Process Bypass` in that window, then activate again.
* Linux error "Could not load the Qt platform plugin xcb": install the missing system library,
  e.g. `sudo apt install libxcb-cursor0` (needed by Qt 6.5+ on Debian/Ubuntu).
* The map stays dark grey: map tiles are downloaded from openstreetmap.org, so internet is needed the
  first time (check firewall/proxy). Already viewed areas are cached on disk and work offline.
* Keys do nothing: click an empty part of the window first. Keys are ignored while a target box is
  being edited; press Enter to return keyboard control. Arm the vehicle first.

## Using it

1. Click **ARM**. While disarmed the thrusters are forced to zero, whatever you press.
2. **Manual mode** (keyboard): `W/S` forward/back, `A/D` left/right, `F` dive, `R` rise, `Q/E` turn
   (arrow keys also work for forward/back/left/right). The *Key thrust* slider sets how hard a key pushes.
3. **Depth Hold / Heading Hold**: pick the mode, type a target, press Enter or **Set**. The target
   starts at the current depth/heading so switching modes never causes a jump.
4. **Space** is an emergency disarm. **Start CSV log** records telemetry to `logs/`.
5. **Simulate link loss** stops delivering operator commands, so you can watch the 1 s watchdog cut
   thrust even while a key is still held. Press it again to restore the link. The **event log**
   (bottom right) records every arm/disarm, mode change, watchdog trip and battery failsafe with a timestamp.
6. Mouse wheel over the map zooms it.

The window shows armed state, mode, roll, pitch, yaw, depth and battery voltage live, the actual
thrust on each axis, a 60 s depth-versus-time plot (with the target as a dashed line in Depth Hold),
and an OpenStreetMap view with the vehicle, its trail and the start point.

## Requirements checklist

| Requirement | Where |
|---|---|
| Surge/sway/heave/yaw motion, drag, thruster delay | `rov_sim.py` -> `_update_thrusters`, `_update_dynamics` |
| Live armed, mode, roll, pitch, yaw, depth, battery | `main.py` -> `on_telemetry` |
| Arm/Disarm + Manual / Depth Hold / Heading Hold | `main.py` (buttons), `rov_sim.py` (logic) |
| Keyboard control | `main.py` -> `eventFilter` |
| Hold modes with operator-set targets | `rov_sim.py` -> `PID`, `_compute_command` |
| Zero thrust when disarmed | `rov_sim.py` -> `disarm`, `_compute_command`, `_update_thrusters` |
| Manual mode: zero thrust after 1 s without input | `rov_sim.py` -> `_compute_command` (watchdog) |
| Live depth-vs-time plot | `main.py` (pyqtgraph) |
| OpenStreetMap view inside the app | `map_widget.py` |
| Responsive UI | `sim_worker.py` (simulation on its own thread) |
| Extras (beyond the brief) | CSV logging, 24 unit tests, event log, low-battery failsafe, link-loss test button |

## Design

```
 keyboard / buttons ──signals──▶ SimWorker (separate QThread, 50 Hz) ──Telemetry (25 Hz)──▶ MainWindow
                                  └─ RovSimulator (pure Python, no Qt)                      labels, plot, map, bars
```

* **Simulation is separate from the UI.** `rov_sim.py` imports no Qt. It is unit-tested on its own,
  and the safety rules live there, so a UI bug cannot produce thrust while disarmed.
* **Own thread, no locks.** The UI never calls the simulator directly. It emits Qt signals, which are
  queued and executed inside the worker thread, so only one thread ever touches the simulator.
  The worker uses a fixed 20 ms physics step, independent of timer jitter.
* **Vehicle model.** Per axis: `m·dv/dt = thrust·Fmax − c₁·v − c₂·|v|·v` (linear + quadratic drag),
  integrated with semi-implicit Euler. Thrust follows the command through a first-order lag
  (`τ = 0.25 s`). Roll/pitch lean with acceleration plus a small sea-state bob (cosmetic).
  Battery voltage falls with charge used and sags under load.
* **Hold modes.** A PID per mode. Depth Hold drives heave from depth error; Heading Hold drives yaw
  from heading error wrapped to ±180° so it always turns the short way. Derivative uses the measured
  velocity (no kick when the target changes). Integral windup is limited by integral separation
  (only integrates near the target), clamping, and conditional integration while saturated.
  In a hold mode only the held axis is automatic; the other axes stay under keyboard control.
* **Safety.**
  * Disarmed: commanded and actual thrust are forced to exactly zero every step, and on the disarm
    call itself (no lag).
  * Manual mode: if no operator input arrives for 1 s, thrust is cut to zero. While a key is held the
    UI re-sends the command at 20 Hz (a heartbeat); on release it sends an explicit neutral command.
    If the window loses focus with a key down, it also sends neutral.
  * Re-arming clears any stale operator input.
* **Failsafes and event log (extra).** Real ground stations tell the operator *why* the vehicle acted,
  so the simulator records timestamped events (`ARMED`, `MODE: ...`, `DISARMED (reason)`, `WATCHDOG ...`,
  `BATTERY LOW`) and the worker forwards them to the UI. The watchdog event is reported once per silence and
  only when input stops mid-command; a normal key release sends an explicit neutral command and is not a fault.
  Battery failsafes: a warning at 14.4 V and an automatic disarm at 13.5 V, with arming refused below that
  (pre-arm check). They use the *resting* (open-circuit) voltage so a burst of thrust, which sags the
  terminal voltage, cannot cause a false alarm. **Simulate link loss** is a test aid: it blocks operator
  commands at the UI so the watchdog can be seen working live; Space still disarms.
* **Map.** Drawn natively with Qt: Web-Mercator maths picks the needed 256 px OSM tiles, downloads
  them asynchronously (with a proper User-Agent), caches them on disk (works offline for
  previously viewed areas) and paints them with the vehicle track. No browser engine is embedded.
  The start point is an approximate location near Ulsoor Lake, Bangalore (`HOME_LAT/HOME_LON` in `main.py`).

## Testing

`python -m unittest -v` runs 24 tests covering: no thrust while disarmed, immediate cut on disarm,
the 1 s watchdog, command clamping, thruster lag, terminal speed and coasting, depth limits,
battery drain, depth/heading hold convergence and wrap-around, rejection of a constant buoyancy
disturbance, mode-switch behaviour, PID anti-windup, and the event log / failsafes (event order, watchdog
reported once and never falsely, low-battery warning and auto-disarm, arming refused on a critical battery,
no false alarm under heavy thrust).
The full application was also exercised end-to-end headlessly (real worker thread, simulated key
presses, arm/disarm, both hold modes, watchdog timing, CSV logging, shutdown).

## AI tool disclosure

> **EDIT THIS SECTION so it is accurate to what you actually did before submitting.**

* **Tool used:** Claude (Anthropic), via the claude.ai chat interface.
* **How it was used:** to plan the architecture, generate the initial code for all modules and the unit
  tests, and to explain the code to me. It also ran the tests and screenshots while building.
* **What I did myself:** [e.g. read every file, ran the app, changed gains/limits to see the effect,
  wrote/modified X, recorded the video, wrote the answers I give in the interview.]

## Known limitations / ideas

* Keyboard only (no joystick); Qt 6 has no built-in gamepad module, so this would need another library.
* Physics is deliberately simple (no buoyancy dynamics, tether, or water current). Roll/pitch are cosmetic.
* The simulator could be wrapped as a ROS 2 node (commands in via subscriptions, `Telemetry` out via a
  publisher) without changing `rov_sim.py`.
