"""
rov_sim.py - the vehicle "model".

Pure Python, NO Qt imports. That is deliberate:
  * it can be unit-tested without a GUI (see tests/test_rov_sim.py)
  * the GUI is just a viewer/controller; the safety rules live HERE, so a
    UI bug can never produce thrust while disarmed
  * the same class could later be wrapped as a ROS 2 node (commands in via
    subscriptions, Telemetry out via a publisher)

Axis order everywhere: SURGE (forward +), SWAY (right +), HEAVE (DOWN +, same
sign as depth), YAW (clockwise / turn right +).
Commands and thrust are normalised to [-1, +1].
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

SURGE, SWAY, HEAVE, YAW = range(4)
ZERO4 = (0.0, 0.0, 0.0, 0.0)


class Mode(str, Enum):
    MANUAL = "Manual"
    DEPTH_HOLD = "Depth Hold"
    HEADING_HOLD = "Heading Hold"


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def wrap180(deg: float) -> float:
    """Wrap an angle to [-180, 180). Used so heading error takes the SHORT way round."""
    return (deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class Params:
    """All tunable numbers in one place. Tuples are ordered (surge, sway, heave, yaw)."""

    # --- vehicle dynamics -------------------------------------------------
    mass: tuple = (22.0, 26.0, 24.0, 3.0)          # kg (incl. added mass); yaw: kg*m^2
    max_thrust: tuple = (80.0, 60.0, 70.0, 10.0)   # N; yaw: N*m
    drag_lin: tuple = (12.0, 16.0, 16.0, 2.0)      # N per (m/s)    -> viscous drag
    drag_quad: tuple = (35.0, 45.0, 45.0, 4.0)     # N per (m/s)^2  -> pressure drag
    thruster_tau: float = 0.25                     # s, thruster response delay
    max_depth: float = 50.0                        # m

    # --- safety -----------------------------------------------------------
    input_timeout: float = 1.0                     # s, Manual-mode watchdog

    # --- hold controllers: (Kp, Ki, Kd) -----------------------------------
    depth_gains: tuple = (0.9, 0.05, 1.6)          # error in m, rate in m/s
    heading_gains: tuple = (0.035, 0.004, 0.03)    # error in deg, rate in deg/s
    depth_i_zone: float = 0.5                      # m   : integral only active this close to target
    heading_i_zone: float = 10.0                   # deg : (stops windup during the big move)
    net_buoyancy: float = 0.0                      # N, + = sinks. A disturbance the I-term must fight

    # --- cosmetic attitude (roll/pitch react to acceleration + sea-state bob)
    roll_gain: float = 4.0                         # deg per m/s^2
    pitch_gain: float = 3.0                        # deg per m/s^2
    attitude_tau: float = 0.4                      # s
    bob_amp: float = 0.6                           # deg

    # --- battery (4S Li-ion-ish) ------------------------------------------
    v_full: float = 16.8
    v_empty: float = 13.2
    capacity_ah: float = 2.0                       # small on purpose so the drop is visible
    idle_current: float = 1.5                      # A
    thruster_current: float = 6.0                  # A per axis at full thrust
    r_internal: float = 0.04                       # ohm -> voltage sag under load
    battery_low_v: float = 14.4                    # V (resting): warn the operator
    battery_critical_v: float = 13.5               # V (resting): auto-disarm and refuse to arm


class PID:
    """PID with derivative-on-measurement and two anti-windup measures:
    integral separation (i_zone) and conditional integration while saturated."""

    def __init__(self, kp: float, ki: float, kd: float, out_limit: float = 1.0,
                 i_limit: float = 0.4, i_zone: float = math.inf):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit, self.i_limit, self.i_zone = out_limit, i_limit, i_zone
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def update(self, error: float, measurement_rate: float, dt: float) -> float:
        # D term uses the measured rate (velocity), not d(error)/dt. This avoids a
        # huge "derivative kick" when the operator changes the target suddenly.
        p = self.kp * error
        d = -self.kd * measurement_rate
        # Integral separation: far from the target the I-term would only wind up
        # (and cause overshoot), so it is frozen until we are close.
        step_i = self.ki * error * dt if abs(error) <= self.i_zone else 0.0
        candidate_i = clamp(self.integral + step_i, -self.i_limit, self.i_limit)
        out = p + candidate_i + d
        sat = clamp(out, -self.out_limit, self.out_limit)
        # Conditional integration: while saturated, only let the integral change
        # if that helps bring the output back inside the limits.
        if out == sat or (out > sat and error < 0) or (out < sat and error > 0):
            self.integral = candidate_i
        return sat


@dataclass(frozen=True)
class Telemetry:
    """Immutable snapshot sent to the UI (safe to hand across threads)."""
    t: float
    armed: bool
    mode: Mode
    roll: float          # deg
    pitch: float         # deg
    yaw: float           # deg, 0..360 (heading)
    depth: float         # m, positive down
    battery_v: float     # V
    north: float         # m from start point
    east: float          # m from start point
    thrust: tuple        # actual thrust (surge, sway, heave, yaw) in [-1, 1]
    target_depth: float
    target_heading: float
    speed: float = 0.0   # m/s, horizontal (surge+sway)


class RovSimulator:
    def __init__(self, params: Params | None = None):
        self.p = params or Params()
        self.t = 0.0
        self.armed = False
        self.mode = Mode.MANUAL
        self.target_depth = 0.0
        self.target_heading = 0.0

        self.events: list = []           # (sim time, message); drained by pop_events()
        self._wd_tripped = False         # watchdog event already reported for this silence
        self._low_warned = False

        self._input = ZERO4              # last operator command
        self._input_time = -math.inf     # sim-time when it arrived (for the watchdog)

        self.thrust = [0.0] * 4          # ACTUAL thrust (after thruster lag)
        self.vel = [0.0] * 4             # u, v, w in m/s ; r in rad/s
        self._acc = [0.0] * 4
        self.north = 0.0
        self.east = 0.0
        self.depth = 0.0
        self.yaw = 0.0                   # deg
        self._roll = 0.0
        self._pitch = 0.0
        self.soc = 1.0
        self._voltage = self._battery_voltage(self.p.idle_current)

        self._depth_pid = PID(*self.p.depth_gains, i_zone=self.p.depth_i_zone)
        self._heading_pid = PID(*self.p.heading_gains, i_zone=self.p.heading_i_zone)
        self._log_event("System ready - DISARMED")

    # ------------------------------------------------------------------ commands
    def _log_event(self, text: str) -> None:
        self.events.append((self.t, text))
        del self.events[:-200]            # never grows without bound

    def pop_events(self) -> list:
        """Return and clear the events recorded since the last call (the worker forwards them to the UI)."""
        events, self.events = self.events, []
        return events

    def arm(self) -> None:
        if self.armed:
            return
        if self._open_circuit() < self.p.battery_critical_v:
            self._log_event("ARM REFUSED: battery critical")      # pre-arm check
            return
        self.armed = True
        self._wd_tripped = False
        self._log_event("ARMED")
        self._input, self._input_time = ZERO4, -math.inf   # stale input can't leak in
        self.thrust = [0.0] * 4
        self._reset_controllers()

    def disarm(self, reason: str = "operator command") -> None:
        if self.armed:
            self._log_event(f"DISARMED ({reason})")
        self.armed = False
        self._input, self._input_time = ZERO4, -math.inf
        self.thrust = [0.0] * 4          # thrust goes to zero IMMEDIATELY (no lag)
        self._reset_controllers()

    def set_mode(self, mode: Mode) -> None:
        if mode != self.mode:
            self.mode = mode
            self._log_event(f"MODE: {mode.value}")
            self._reset_controllers()

    def set_target_depth(self, metres: float) -> None:
        self.target_depth = clamp(metres, 0.0, self.p.max_depth)

    def set_target_heading(self, deg: float) -> None:
        self.target_heading = deg % 360.0

    def set_manual_command(self, surge: float, sway: float, heave: float, yaw: float) -> None:
        """Operator input. Every call re-arms the 1 s watchdog."""
        self._input = tuple(clamp(c, -1.0, 1.0) for c in (surge, sway, heave, yaw))
        self._input_time = self.t

    def _reset_controllers(self) -> None:
        """Bumpless transfer: PID state cleared and the targets start at the current state."""
        self._depth_pid.reset()
        self._heading_pid.reset()
        self.target_depth = self.depth
        self.target_heading = self.yaw

    # ------------------------------------------------------------------ one step
    def step(self, dt: float) -> None:
        self.t += dt
        cmd, cut = self._compute_command(dt)
        self._update_thrusters(cmd, cut, dt)
        self._update_dynamics(dt)
        self._update_attitude(dt)
        self._update_battery(dt)

    def _compute_command(self, dt: float):
        """Returns (command[4], cut). cut=True means force actual thrust to zero now."""
        if not self.armed:
            return ZERO4, True

        timed_out = (self.t - self._input_time) > self.p.input_timeout
        cmd = list(ZERO4 if timed_out else self._input)

        # Report a watchdog trip once per silence. Only a REAL trip counts: the last command was
        # non-neutral and then input stopped. (Releasing a key sends an explicit neutral command.)
        if not timed_out:
            self._wd_tripped = False
        elif not self._wd_tripped and any(self._input):
            self._wd_tripped = True
            what = ("thrust cut to zero" if self.mode == Mode.MANUAL
                    else "operator commands zeroed (hold mode keeps its axis)")
            self._log_event(f"WATCHDOG: no operator input for {self.p.input_timeout:.1f} s - {what}")

        if self.mode == Mode.DEPTH_HOLD:
            err = self.target_depth - self.depth
            cmd[HEAVE] = self._depth_pid.update(err, self.vel[HEAVE], dt)
        elif self.mode == Mode.HEADING_HOLD:
            err = wrap180(self.target_heading - self.yaw)
            cmd[YAW] = self._heading_pid.update(err, math.degrees(self.vel[YAW]), dt)

        # Watchdog: Manual mode + no operator input for > timeout -> thrust is cut.
        cut = self.mode == Mode.MANUAL and timed_out
        return cmd, cut

    def _update_thrusters(self, cmd, cut: bool, dt: float) -> None:
        if cut:
            self.thrust = [0.0] * 4
            return
        # First-order lag, exact discretisation: thrust moves a fraction of the
        # way to the command each step, independent of dt.
        alpha = 1.0 - math.exp(-dt / self.p.thruster_tau)
        for i in range(4):
            self.thrust[i] += alpha * (cmd[i] - self.thrust[i])

    def _update_dynamics(self, dt: float) -> None:
        p = self.p
        for i in range(4):
            v = self.vel[i]
            force = self.thrust[i] * p.max_thrust[i]
            if i == HEAVE:
                force += p.net_buoyancy                        # constant disturbance (0 by default)
            drag = p.drag_lin[i] * v + p.drag_quad[i] * abs(v) * v
            self._acc[i] = (force - drag) / p.mass[i]          # Newton: m*a = F - drag
            self.vel[i] = v + self._acc[i] * dt                # semi-implicit Euler

        u, v, w, r = self.vel
        self.yaw = (self.yaw + math.degrees(r) * dt) % 360.0
        psi = math.radians(self.yaw)
        # rotate body-frame velocity into the world (north/east) frame
        self.north += (u * math.cos(psi) - v * math.sin(psi)) * dt
        self.east += (u * math.sin(psi) + v * math.cos(psi)) * dt

        self.depth += w * dt
        if self.depth < 0.0:                       # can't go above the surface
            self.depth, self.vel[HEAVE] = 0.0, max(0.0, self.vel[HEAVE])
        elif self.depth > p.max_depth:             # can't go through the seabed
            self.depth, self.vel[HEAVE] = p.max_depth, min(0.0, self.vel[HEAVE])

    def _update_attitude(self, dt: float) -> None:
        """Cosmetic: roll/pitch lean with acceleration (first-order lag)."""
        p = self.p
        roll_target = -p.roll_gain * (self._acc[SWAY] + self.vel[SURGE] * self.vel[YAW])
        pitch_target = p.pitch_gain * self._acc[SURGE]
        alpha = 1.0 - math.exp(-dt / p.attitude_tau)
        self._roll += alpha * (clamp(roll_target, -25, 25) - self._roll)
        self._pitch += alpha * (clamp(pitch_target, -25, 25) - self._pitch)

    def _open_circuit(self) -> float:
        """Resting battery voltage (no load sag). Failsafes use this so a hard thrust burst
        does not trigger a false low-battery alarm."""
        p = self.p
        return p.v_empty + (p.v_full - p.v_empty) * self.soc

    def _battery_voltage(self, current: float) -> float:
        return max(0.0, self._open_circuit() - self.p.r_internal * current)

    def _update_battery(self, dt: float) -> None:
        p = self.p
        current = p.idle_current + p.thruster_current * sum(abs(x) for x in self.thrust)
        self.soc = max(0.0, self.soc - current * dt / (p.capacity_ah * 3600.0))
        self._voltage = self._battery_voltage(current)

        resting = self._open_circuit()
        if not self._low_warned and resting < p.battery_low_v:
            self._low_warned = True
            self._log_event(f"BATTERY LOW: {resting:.1f} V resting - return the vehicle soon")
        if self.armed and resting < p.battery_critical_v:
            self.disarm(f"battery critical, {resting:.1f} V resting")

    # ------------------------------------------------------------------ output
    def telemetry(self) -> Telemetry:
        bob_r = self.p.bob_amp * math.sin(2 * math.pi * 0.31 * self.t + 0.7)
        bob_p = self.p.bob_amp * 0.6 * math.sin(2 * math.pi * 0.23 * self.t + 2.1)
        return Telemetry(
            t=self.t, armed=self.armed, mode=self.mode,
            roll=self._roll + bob_r, pitch=self._pitch + bob_p, yaw=self.yaw,
            depth=self.depth, battery_v=self._voltage,
            north=self.north, east=self.east, thrust=tuple(self.thrust),
            target_depth=self.target_depth, target_heading=self.target_heading,
            speed=math.hypot(self.vel[SURGE], self.vel[SWAY]),
        )
