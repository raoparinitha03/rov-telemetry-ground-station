"""Run from the project folder:  python -m unittest -v"""
import unittest

from dataclasses import replace

from rov_sim import HEAVE, SURGE, YAW, Mode, Params, PID, RovSimulator, wrap180

DT = 0.02


def run(sim, seconds, command=None):
    """Advance the sim. If `command` is given it is re-sent every step (like a held key)."""
    for _ in range(round(seconds / DT)):
        if command is not None:
            sim.set_manual_command(*command)
        sim.step(DT)


class SafetyTests(unittest.TestCase):
    def test_no_thrust_while_disarmed(self):
        sim = RovSimulator()
        for _ in range(100):
            sim.set_manual_command(1, 1, 1, 1)
            sim.step(DT)
            self.assertEqual(sim.thrust, [0.0] * 4)

    def test_disarm_cuts_thrust_immediately(self):
        sim = RovSimulator()
        sim.arm()
        run(sim, 2.0, (1, 0, 0, 0))
        self.assertGreater(sim.thrust[SURGE], 0.9)
        sim.disarm()
        self.assertEqual(sim.thrust, [0.0] * 4)   # before any further step
        sim.step(DT)
        self.assertEqual(sim.thrust, [0.0] * 4)

    def test_disarm_clears_stale_input_on_rearm(self):
        sim = RovSimulator()
        sim.arm()
        sim.set_manual_command(1, 0, 0, 0)
        sim.disarm()
        sim.arm()
        run(sim, 0.5)                               # no new input after re-arming
        self.assertEqual(sim.thrust, [0.0] * 4)

    def test_manual_watchdog_zeroes_thrust_after_one_second(self):
        sim = RovSimulator()
        sim.arm()
        run(sim, 2.0, (1, 0, 0, 0))                 # held key for 2 s
        self.assertGreater(sim.thrust[SURGE], 0.9)
        run(sim, 0.9)                               # 0.9 s of silence: still thrusting
        self.assertGreater(sim.thrust[SURGE], 0.0)
        run(sim, 0.2)                               # total 1.1 s of silence
        self.assertEqual(sim.thrust, [0.0] * 4)

    def test_commands_are_clamped(self):
        sim = RovSimulator()
        sim.arm()
        run(sim, 3.0, (50, 0, 0, 0))
        self.assertLessEqual(sim.thrust[SURGE], 1.0)


class DynamicsTests(unittest.TestCase):
    def test_thruster_delay(self):
        sim = RovSimulator()
        sim.arm()
        sim.set_manual_command(1, 0, 0, 0)
        sim.step(DT)
        self.assertLess(sim.thrust[SURGE], 0.2)     # does NOT jump to 1.0
        run(sim, 2.0, (1, 0, 0, 0))
        self.assertGreater(sim.thrust[SURGE], 0.99)

    def test_forward_motion_reaches_terminal_speed_and_coasts_to_stop(self):
        sim = RovSimulator()
        sim.arm()
        run(sim, 8.0, (1, 0, 0, 0))
        top = sim.vel[SURGE]
        self.assertGreater(top, 0.8)
        self.assertLess(top, 2.0)
        self.assertGreater(sim.north, 5.0)          # yaw 0 -> moving north
        run(sim, 0.5, (1, 0, 0, 0))
        self.assertAlmostEqual(sim.vel[SURGE], top, delta=0.02)   # steady state
        sim.set_manual_command(0, 0, 0, 0)          # explicit neutral
        run(sim, 0.8)
        run(sim, 6.0, (0, 0, 0, 0))
        self.assertLess(abs(sim.vel[SURGE]), 0.01)

    def test_sideways_and_heading_frame(self):
        sim = RovSimulator()
        sim.arm()
        run(sim, 4.0, (0, 1, 0, 0))                 # sway right while facing north
        self.assertGreater(sim.east, 1.0)
        self.assertAlmostEqual(sim.north, 0.0, delta=0.01)

    def test_dive_and_surface_limits(self):
        sim = RovSimulator()
        sim.arm()
        run(sim, 4.0, (0, 0, 1, 0))
        self.assertGreater(sim.depth, 1.0)
        run(sim, 30.0, (0, 0, -1, 0))
        self.assertEqual(sim.depth, 0.0)            # stops at the surface, never negative

    def test_battery_drains_faster_under_load(self):
        idle, busy = RovSimulator(), RovSimulator()
        idle.arm()
        busy.arm()
        run(idle, 10.0)
        run(busy, 10.0, (1, 1, 1, 1))
        self.assertLess(busy.telemetry().battery_v, idle.telemetry().battery_v)
        self.assertLess(busy.soc, idle.soc)


class HoldModeTests(unittest.TestCase):
    def test_depth_hold_reaches_target_without_big_overshoot(self):
        sim = RovSimulator()
        sim.arm()
        sim.set_mode(Mode.DEPTH_HOLD)
        sim.set_target_depth(5.0)
        peak = 0.0
        for _ in range(round(40 / DT)):
            sim.step(DT)
            peak = max(peak, sim.depth)
        self.assertAlmostEqual(sim.depth, 5.0, delta=0.05)
        self.assertLess(peak, 5.05)   # essentially no overshoot

    def test_depth_hold_rejects_constant_buoyancy_disturbance(self):
        sim = RovSimulator(replace(Params(), net_buoyancy=8.0))   # vehicle is 8 N heavy
        sim.arm()
        sim.set_mode(Mode.DEPTH_HOLD)
        sim.set_target_depth(3.0)
        run(sim, 60.0)
        self.assertAlmostEqual(sim.depth, 3.0, delta=0.05)         # I-term removes the offset

    def test_depth_hold_ignores_operator_heave_but_keeps_other_axes(self):
        sim = RovSimulator()
        sim.arm()
        sim.set_mode(Mode.DEPTH_HOLD)
        sim.set_target_depth(0.0)
        run(sim, 5.0, (1, 0, 1, 0))                 # operator pushes down + forward
        self.assertLess(sim.depth, 0.1)             # PID refuses to dive
        self.assertGreater(sim.north, 1.0)          # but forward still works

    def test_heading_hold_takes_short_way_across_north(self):
        sim = RovSimulator()
        sim.arm()
        sim.yaw = 350.0
        sim.set_mode(Mode.HEADING_HOLD)
        sim.set_target_heading(10.0)
        run(sim, 0.5)
        # shortest path is +20 deg (clockwise through 0), never the long way round
        self.assertGreater(sim.vel[YAW], 0.0)
        run(sim, 20.0)
        self.assertAlmostEqual(wrap180(sim.yaw - 10.0), 0.0, delta=1.0)

    def test_heading_hold_large_turn_settles(self):
        sim = RovSimulator()
        sim.arm()
        sim.set_mode(Mode.HEADING_HOLD)
        sim.set_target_heading(180.0)
        run(sim, 25.0)
        self.assertAlmostEqual(wrap180(sim.yaw - 180.0), 0.0, delta=1.0)

    def test_switching_mode_starts_from_current_state(self):
        sim = RovSimulator()
        sim.arm()
        run(sim, 3.0, (0, 0, 1, 0))
        d = sim.depth
        sim.set_mode(Mode.DEPTH_HOLD)
        self.assertEqual(sim.target_depth, d)       # no sudden jump on mode change

    def test_hold_modes_produce_no_thrust_when_disarmed(self):
        sim = RovSimulator()
        sim.set_mode(Mode.DEPTH_HOLD)
        sim.set_target_depth(10.0)
        run(sim, 2.0)
        self.assertEqual(sim.thrust, [0.0] * 4)
        self.assertEqual(sim.depth, 0.0)


class EventAndFailsafeTests(unittest.TestCase):
    @staticmethod
    def messages(sim):
        return [m for _, m in sim.pop_events()]

    def test_events_for_arm_mode_disarm(self):
        sim = RovSimulator()
        self.assertEqual(self.messages(sim), ["System ready - DISARMED"])
        sim.arm()
        sim.set_mode(Mode.DEPTH_HOLD)
        sim.disarm()
        self.assertEqual(self.messages(sim), ["ARMED", "MODE: Depth Hold", "DISARMED (operator command)"])
        self.assertEqual(self.messages(sim), [])                 # pop_events clears the list

    def test_watchdog_event_only_when_input_stops_mid_command(self):
        sim = RovSimulator()
        sim.arm()
        sim.pop_events()
        run(sim, 1.0, (1, 0, 0, 0))
        run(sim, 3.0)                                            # input just stops
        self.assertEqual(sum("WATCHDOG" in m for m in self.messages(sim)), 1)   # reported once

        calm = RovSimulator()
        calm.arm()
        calm.pop_events()
        run(calm, 1.0, (1, 0, 0, 0))
        calm.set_manual_command(0, 0, 0, 0)                      # normal key release = explicit neutral
        run(calm, 3.0)
        self.assertFalse(any("WATCHDOG" in m for m in self.messages(calm)))

    def test_low_battery_warns_once_then_auto_disarms(self):
        sim = RovSimulator()
        sim.arm()
        sim.pop_events()
        sim.soc = 0.30                                           # resting 14.28 V < 14.4 V
        run(sim, 1.0)
        msgs = self.messages(sim)
        self.assertEqual(sum("BATTERY LOW" in m for m in msgs), 1)
        self.assertTrue(sim.armed)
        sim.soc = 0.05                                           # resting 13.38 V < 13.5 V
        run(sim, 0.1)
        self.assertFalse(sim.armed)
        self.assertTrue(any("DISARMED (battery critical" in m for m in self.messages(sim)))
        self.assertEqual(sim.thrust, [0.0] * 4)

    def test_cannot_arm_with_critical_battery(self):
        sim = RovSimulator()
        sim.soc = 0.05
        sim.pop_events()
        sim.arm()
        self.assertFalse(sim.armed)
        self.assertTrue(any("ARM REFUSED" in m for m in self.messages(sim)))

    def test_heavy_thrust_does_not_cause_false_battery_alarm(self):
        sim = RovSimulator()
        sim.arm()
        sim.pop_events()
        run(sim, 20.0, (1, 1, 1, 1))                             # load sag must not trip the failsafe
        self.assertTrue(sim.armed)
        self.assertFalse(any("BATTERY" in m for m in self.messages(sim)))


class PIDTests(unittest.TestCase):
    def test_output_is_limited_and_integral_does_not_wind_up(self):
        pid = PID(1.0, 1.0, 0.0, out_limit=1.0, i_limit=0.4)
        for _ in range(1000):
            out = pid.update(error=100.0, measurement_rate=0.0, dt=0.02)
        self.assertEqual(out, 1.0)
        self.assertLessEqual(pid.integral, 0.4)


class HelperTests(unittest.TestCase):
    def test_wrap180(self):
        self.assertEqual(wrap180(190), -170)
        self.assertEqual(wrap180(-190), 170)
        self.assertEqual(wrap180(10 - 350), 20)


if __name__ == "__main__":
    unittest.main()
