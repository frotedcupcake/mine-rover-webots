"""RETURN mode: triggers, Danger hold and re-arm, HOME, blocked both ways, battery latch, link.

The rig is a 1-D tunnel along +x (entrance wall at x = 0, end wall at x = length).  Position
and gyro yaw are integrated from the motor commands, so the navigator is driven by nothing
but what the real rover has: two sonar distances, the IR bits, gas, IMU, battery, link, time.
"""
import math

import pytest
from conftest import DT, mk

from rover_core.battery import CRITICAL, OK, RETURN, WARN, BatteryMonitor, BatteryStatus, volts_to_counts
from rover_core.fusion import HazardFusion
from rover_core.navigator import (AUTO, AVOID, BACKOFF, CRUISE, DIR_IN, DIR_RETURN, HOLD, HOME, MANUAL, STOP,
                                  STUCK, SWEEP, Navigator, wrap180)

YAW_RATE = 80.0 / 1.1        # deg/s per unit of (right - left): a 0.55 spin turns 80 deg/s
TOP_SPEED = 0.4              # m/s at command 1.0
IR_AHEAD = 0.13              # down IRs ahead of the rover centre


class TripRig:
    def __init__(self, cfg, length=8.0, x=0.6, mode=AUTO):
        cfg["navigator"]["start_mode"] = mode
        self.cfg = cfg
        self.nav = Navigator(cfg)
        self.fusion = HazardFusion(cfg)
        self.monitor = BatteryMonitor(cfg)
        self.length, self.x, self.yaw = length, x, 0.0
        self.x_max = x
        self.link_x = x + cfg["link"]["link_range_m"]
        self.link = None             # None: from the position; True/False: forced
        self.battery = None          # a BatteryStatus handed straight to the navigator ...
        self.volts = None            # ... or a pack voltage fed through the real BatteryMonitor
        self.gas = 20.0              # ppm, or a function of t
        self.pit = None              # (x_lo, x_hi, "both" | "left" | "right")
        self.over = {}               # reading overrides
        self.t = 0.0
        self.cmd = None
        self.sweeps, self.flushes, self.hist = [], [], []
        self.sweep_busy_until = -1.0

    # ---- world ----
    def scene(self):
        h = wrap180(self.yaw)
        if abs(h) <= 40:
            d = (self.length - self.x) / math.cos(math.radians(h))
        elif abs(h) >= 140:
            d = self.x / math.cos(math.radians(abs(h) - 180))
        else:
            d = 0.45                 # facing a side wall
        us = d if d < 3.9 else None
        return dict(us_left=us, us_right=us)

    def floor(self):
        kw = {}
        if self.pit:
            lo, hi, side = self.pit
            ir = self.x + IR_AHEAD * math.cos(math.radians(self.yaw))
            if lo < ir < hi:
                if side in ("both", "left"):
                    kw.update(floor_lost_left=True, floor_raw_left=True)
                if side in ("both", "right"):
                    kw.update(floor_lost_right=True, floor_raw_right=True)
        return kw

    def step(self, keys=(), **kw):
        c = self.cmd
        if c is not None:
            self.yaw += YAW_RATE * (c.right - c.left) * DT
            self.x += TOP_SPEED * (c.left + c.right) / 2.0 * math.cos(math.radians(self.yaw)) * DT
            self.x_max = max(self.x_max, self.x)
        gas = self.gas(self.t) if callable(self.gas) else self.gas
        r = mk(self.t, **{**dict(yaw_deg=self.yaw, gas_ppm=gas), **self.scene(), **self.floor(), **self.over, **kw})
        a = self.fusion.update(r)
        batt = self.battery
        if self.volts is not None:
            batt = self.monitor.update(self.t, volts_to_counts(self.volts, self.cfg["battery"]))
        link = self.link if self.link is not None else self.x <= self.link_x
        busy = self.t < self.sweep_busy_until
        self.cmd = self.nav.step(self.t, r, a, keys=keys, sweep_busy=busy, battery=batt, link_up=link)
        if self.cmd.start_sweep:
            self.sweeps.append((round(self.t, 2), self.cmd.start_sweep))
            self.sweep_busy_until = self.t + 1.0
        if self.cmd.flush:
            self.flushes.append((round(self.t, 2), self.cmd.state))
        lost = r.floor_lost_left or r.floor_lost_right
        self.hist.append((self.t, self.cmd.state, self.cmd.direction, self.cmd.left, self.cmd.right, lost))
        self.t = round(self.t + DT, 6)
        return self.cmd

    def run(self, secs, keys=(), **kw):
        for _ in range(int(round(secs / DT))):
            self.step(keys=keys, **kw)
        return self.cmd

    def until(self, cond, max_s=120.0):
        for _ in range(int(round(max_s / DT))):
            self.step()
            if cond():
                return self.cmd
        raise AssertionError(f"not reached within {max_s} s: state {self.cmd.state} dir {self.cmd.direction} "
                             f"note {self.cmd.note!r} x {self.x:.2f} yaw {self.yaw:.0f}")

    def states(self):
        return [h[1] for h in self.hist]

    def facing_home(self, tol=45):
        return abs(wrap180(self.yaw - 180)) <= tol


@pytest.fixture
def rig(cfg):
    return TripRig(cfg)


def returning(rig):
    return rig.cmd.direction == DIR_RETURN


def batt(level):
    return BatteryStatus(level=level, volts=10.0, avg_v=10.0)


# ---------------------------------------------------------------- trip in
def test_trip_starts_inward_and_link_up_alone_never_means_home(rig):
    c = rig.run(5.0)                                  # inside link range the whole time
    assert (c.state, c.direction, c.return_reason) == (CRUISE, DIR_IN, None)
    assert HOME not in rig.states() and not rig.flushes


# ---------------------------------------------------------------- trigger: blocked ahead
def test_dead_end_gives_return_not_stuck(cfg):
    rig = TripRig(cfg, length=6.0)
    rig.until(lambda: returning(rig), 60)
    assert rig.cmd.return_reason == "blocked ahead"
    assert rig.x > 5.0                                # it did drive to the end wall first
    assert STUCK not in rig.states()
    assert [f[1] for f in rig.flushes] == [AVOID]     # log flushed once, on entering RETURN
    # the eight-second spin came first
    t_avoid = next(h[0] for h in rig.hist if h[1] == AVOID)
    assert rig.t - t_avoid == pytest.approx(cfg["navigator"]["avoid_timeout_s"], abs=0.3)


def test_return_steers_on_route_plus_180_with_the_same_sector(cfg):
    rig = TripRig(cfg, length=6.0)
    rig.until(lambda: returning(rig), 60)
    assert wrap180(rig.cmd.route - 180.0) == pytest.approx(0.0, abs=2.0)
    rig.until(lambda: rig.cmd.state == CRUISE, 10)    # turned round by the normal AVOID
    assert rig.facing_home()
    x0 = rig.x
    rig.run(2.0)
    assert rig.x < x0 - 0.3                           # and is driving back out


def test_home_when_the_link_comes_back(cfg):
    rig = TripRig(cfg, length=8.0)
    rig.until(lambda: rig.cmd.state == HOME, 120)
    assert rig.cmd.direction == DIR_RETURN and "link" in rig.cmd.note
    assert rig.x == pytest.approx(rig.link_x, abs=0.05)
    assert (rig.cmd.left, rig.cmd.right) == (0.0, 0.0)
    assert [f[1] for f in rig.flushes] == [AVOID, HOME]
    rig.run(5.0)
    assert rig.cmd.state == HOME and (rig.cmd.left, rig.cmd.right) == (0.0, 0.0)


def test_home_by_time_when_the_link_never_returns(cfg):
    cfg["navigator"]["mission_time_s"] = 10.0
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.until(lambda: returning(rig), 15)
    out_s = rig.nav.out_s
    assert out_s == pytest.approx(10.0, abs=0.15)
    rig.until(lambda: rig.cmd.state == HOME, 30)
    assert "time" in rig.cmd.note
    assert rig.nav.ret_s == pytest.approx(cfg["navigator"]["return_time_factor"] * out_s, abs=0.1)
    assert rig.flushes[-1][1] == HOME


def test_blocked_at_once_inside_link_range_turns_round_before_home(cfg):
    rig = TripRig(cfg, length=2.0)                    # end wall 1.4 m from the start: link never drops
    rig.until(lambda: returning(rig), 30)
    assert rig.cmd.state != HOME                      # not HOME on the spot
    rig.until(lambda: rig.cmd.state == HOME, 10)
    assert rig.facing_home()


# ---------------------------------------------------------------- blocked both ways
def test_blocked_again_on_the_way_back_is_stuck_blocked_both_ways(cfg):
    rig = TripRig(cfg, length=8.0)
    rig.link = False
    rig.until(lambda: returning(rig), 60)
    rig.until(lambda: rig.cmd.state == CRUISE, 10)
    back_wall = rig.x - 1.0                           # a barrier comes down behind the rover

    def scene():
        h = wrap180(rig.yaw)
        if abs(h) >= 140:
            d = (rig.x - back_wall) / math.cos(math.radians(abs(h) - 180))
        elif abs(h) <= 40:
            d = (rig.length - rig.x) / math.cos(math.radians(h))
        else:
            d = 0.45
        return dict(us_left=d if d < 3.9 else None, us_right=d if d < 3.9 else None)
    rig.scene = scene
    rig.until(lambda: rig.cmd.state == STUCK, 30)
    assert "blocked both ways" in rig.cmd.note
    assert (rig.cmd.left, rig.cmd.right) == (0.0, 0.0)
    assert rig.flushes[-1][1] == STUCK                # full log flushed on the way into STUCK
    assert rig.cmd.direction == DIR_RETURN
    rig.run(5.0)
    assert rig.cmd.state == STUCK and HOME not in rig.states()


# ---------------------------------------------------------------- trigger: pits
def test_full_width_pit_backs_off_then_returns(cfg):
    rig = TripRig(cfg, length=8.0)
    rig.pit = (5.0, 5.8, "both")
    rig.until(lambda: returning(rig), 40)
    assert rig.cmd.return_reason == "full-width pit"
    st = rig.states()
    assert SWEEP in st and BACKOFF in st and STUCK not in st
    assert rig.sweeps[0][1] == "FLOOR_LOST"
    rev = [h for h in rig.hist if h[1] == BACKOFF and h[3] < 0 and h[4] < 0]
    assert 0.6 <= len(rev) * DT <= 0.85               # reversed off the edge before turning
    assert rig.x + IR_AHEAD < 5.0
    rig.until(lambda: rig.cmd.state == HOME, 60)
    assert rig.x == pytest.approx(rig.link_x, abs=0.05)


def test_one_sided_pit_sidesteps_and_carries_on_inward(cfg):
    rig = TripRig(cfg, length=30.0)
    rig.run(1.0)
    rig.over = dict(floor_lost_left=True, floor_raw_left=True)
    rig.run(0.3)
    rig.over = {}
    rig.run(12.0)
    assert rig.cmd.direction == DIR_IN and rig.cmd.state == CRUISE


def test_never_forward_over_the_pit_on_the_way_in_or_back(cfg):
    rig = TripRig(cfg, length=8.0)
    rig.pit = (5.0, 5.8, "both")
    rig.until(lambda: rig.cmd.state == HOME, 90)
    assert all(not (h[5] and (h[3] > 0 or h[4] > 0)) for h in rig.hist)
    assert rig.x_max + IR_AHEAD < 5.0 + 0.03          # the IR found the edge; the rover never went on


def test_three_pits_in_two_minutes_returns(cfg):
    rig = TripRig(cfg, length=60.0, x=10.0)
    rig.link = False
    for i in range(3):
        rig.run(1.0)
        rig.over = dict(floor_lost_left=True, floor_raw_left=True)
        rig.run(0.3)
        rig.over = {}
        if i < 2:
            rig.run(12.0)
            assert (rig.cmd.state, rig.cmd.direction) == (CRUISE, DIR_IN)
    assert rig.t < 120
    rig.until(lambda: returning(rig), 5)              # the third: sweep, reverse, then RETURN
    assert rig.cmd.return_reason == "3 pits in 2 minutes"
    assert STUCK not in rig.states()


def test_pit_count_starts_again_on_the_return_leg(cfg):
    cfg["navigator"]["mission_time_s"] = 30.0
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    for _ in range(2):                                # two pits on the way in
        rig.run(1.0)
        rig.over = dict(floor_lost_left=True, floor_raw_left=True)
        rig.run(0.3)
        rig.over = {}
        rig.run(12.0)
    rig.until(lambda: returning(rig), 10)
    assert rig.cmd.return_reason == "mission timer"
    rig.until(lambda: rig.cmd.state == CRUISE, 10)
    rig.over = dict(floor_lost_right=True, floor_raw_right=True)   # the same pit, passed again
    rig.run(0.3)
    rig.over = {}
    rig.run(8.0)
    assert rig.cmd.state in (CRUISE, HOME) and STUCK not in rig.states()


def test_full_width_pit_on_the_way_back_is_blocked_both_ways(cfg):
    rig = TripRig(cfg, length=8.0)
    rig.link = False
    rig.until(lambda: returning(rig), 60)
    rig.until(lambda: rig.cmd.state == CRUISE, 10)
    rig.pit = (rig.x - 1.8, rig.x - 1.0, "both")      # the floor has gone behind the rover
    edge = rig.pit[1]
    rig.until(lambda: rig.cmd.state == STUCK, 30)
    assert "blocked both ways" in rig.cmd.note
    assert rig.flushes[-1][1] == STUCK
    assert rig.x - IR_AHEAD > edge                    # backed off the edge before stopping for good


# ---------------------------------------------------------------- trigger: slope
def test_too_steep_slope_backs_off_until_level_then_returns(cfg):
    rig = TripRig(cfg, length=30.0, x=10.0)
    rig.link = False
    rig.run(1.0)
    rig.over = dict(pitch_deg=24.0)
    rig.run(2.2)                                      # sweep, then reversing while still tilted
    assert rig.cmd.state == BACKOFF and rig.cmd.left < 0 and rig.cmd.right < 0
    assert rig.cmd.direction == DIR_IN
    rig.over = dict(pitch_deg=2.0)
    rig.until(lambda: returning(rig), 3)
    assert rig.cmd.return_reason == "slope too steep"


def test_side_slope_turns_away_and_carries_on(cfg):
    rig = TripRig(cfg, length=30.0, x=10.0)
    rig.run(1.0)
    rig.over = dict(roll_deg=24.0)
    rig.run(2.2)
    rig.over = {}
    rig.run(8.0)
    assert rig.cmd.direction == DIR_IN


# ---------------------------------------------------------------- trigger: battery
def test_battery_at_the_return_level_starts_return(cfg):
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.run(3.0)
    rig.battery = batt(RETURN)
    rig.step()
    assert returning(rig) and rig.cmd.return_reason == "battery reserve"
    assert len(rig.flushes) == 1


def test_battery_warn_adds_no_driving_behaviour_beyond_return(cfg):
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.run(3.0)
    rig.battery = batt(WARN)                          # warn is telemetry only
    rig.until(lambda: rig.cmd.state == CRUISE and returning(rig), 10)
    assert rig.cmd.left > 0 and STUCK not in rig.states()


def test_voltage_dips_below_the_threshold_then_recovers_rover_keeps_returning(cfg):
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.volts = 10.9
    rig.run(8.0)
    assert rig.cmd.direction == DIR_IN
    rig.volts = 10.3                                  # below 10.5 V for long enough to move the 5 s mean
    rig.until(lambda: returning(rig), 8)
    assert rig.cmd.return_reason == "battery reserve"
    rig.volts = 11.2                                  # pack recovers
    rig.run(12.0)
    assert rig.monitor.status.avg_v > 11.0
    assert rig.monitor.status.level == RETURN
    assert rig.cmd.direction == DIR_RETURN and rig.cmd.return_reason == "battery reserve"
    assert rig.cmd.state == CRUISE and rig.facing_home()


def test_short_sag_does_not_start_return(cfg):
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.volts = 10.9
    rig.run(8.0)
    rig.volts = 10.2
    rig.run(1.0)
    rig.volts = 10.9
    rig.run(8.0)
    assert rig.cmd.direction == DIR_IN


def test_battery_critical_stops_for_good_and_flushes(cfg):
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.volts = 11.0
    rig.run(6.0)
    rig.volts = 9.4
    rig.until(lambda: rig.cmd.state == STUCK, 8)
    assert "battery critical" in rig.cmd.note
    assert (rig.cmd.left, rig.cmd.right) == (0.0, 0.0)
    assert rig.flushes[-1][1] == STUCK
    n_flush = len(rig.flushes)
    rig.volts = 11.8                                  # recovers once the motors are off
    rig.run(15.0)
    assert rig.monitor.status.level == CRITICAL
    assert rig.cmd.state == STUCK and "battery critical" in rig.cmd.note
    rig.link = True
    rig.step(keys={"R"})                              # operator resume is refused
    rig.run(1.0)
    assert rig.cmd.state == STUCK and (rig.cmd.left, rig.cmd.right) == (0.0, 0.0)
    assert len(rig.flushes) == n_flush                # flushed once, not every cycle


def test_battery_critical_stops_manual_driving_too(cfg):
    rig = TripRig(cfg, length=60.0, x=20.0, mode=MANUAL)
    rig.link = True
    c = rig.run(1.0, keys={"W"})
    assert c.left > 0
    rig.battery = batt(CRITICAL)
    c = rig.run(1.0, keys={"W"})
    assert (c.left, c.right) == (0.0, 0.0) and "battery critical" in c.note


# ---------------------------------------------------------------- trigger: mission timer
def test_mission_timer_starts_return(cfg):
    cfg["navigator"]["mission_time_s"] = 6.0
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.run(5.9)
    assert rig.cmd.direction == DIR_IN
    rig.run(0.2)
    assert returning(rig) and rig.cmd.return_reason == "mission timer"


def test_approved_defaults_are_in_the_config(cfg):
    n = cfg["navigator"]
    assert (n["mission_time_s"], n["danger_hold_s"], n["return_time_factor"]) == (900, 30, 1.2)
    assert cfg["link"]["link_range_m"] == 3.0


def test_two_triggers_in_the_same_step_give_one_return(cfg):
    cfg["navigator"]["mission_time_s"] = 4.0
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.run(3.95)
    rig.battery = batt(RETURN)                        # battery and timer fire together
    rig.run(0.2)
    assert returning(rig) and rig.cmd.return_reason == "battery reserve"
    assert len(rig.flushes) == 1
    assert wrap180(rig.cmd.route - 180.0) == pytest.approx(0.0, abs=2.0)     # flipped once, not twice


def test_return_trigger_during_a_sweep_waits_for_the_sweep(cfg):
    cfg["navigator"]["mission_time_s"] = 5.0
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.run(4.6)
    rig.gas = 400.0                                   # CAUTION sweep from ~4.65 s to ~5.65 s
    rig.run(0.6)
    assert rig.cmd.state == SWEEP and rig.cmd.direction == DIR_IN and rig.t > 5.0
    rig.until(lambda: returning(rig), 2)
    assert rig.t >= rig.sweeps[-1][0] + 1.0
    assert len([s for s in rig.sweeps if s[1] == "GAS_ELEVATED"]) == 1


def test_sweep_in_the_middle_of_the_turn_round_resumes_the_turn(cfg):
    cfg["navigator"]["mission_time_s"] = 5.0
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.until(lambda: returning(rig), 6)
    rig.run(0.8)                                      # part-way round (about 60 of 180 degrees)
    assert rig.cmd.state == AVOID and not rig.facing_home(90)
    rig.gas = 400.0                                   # CAUTION sweep request while turning
    rig.until(lambda: rig.cmd.state == SWEEP, 1)
    rig.until(lambda: rig.cmd.state != SWEEP, 3)
    assert rig.cmd.state == AVOID                     # back to turning, not cruising the wrong way
    rig.until(lambda: rig.cmd.state == CRUISE, 10)
    assert rig.facing_home()


def test_turn_round_is_not_left_before_facing_the_new_route(cfg):
    cfg["navigator"]["mission_time_s"] = 5.0
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.scene = lambda: dict(us_left=None, us_right=None)        # open ground: every direction reads clear
    rig.until(lambda: returning(rig), 6)
    rig.until(lambda: rig.cmd.state == CRUISE, 10)
    assert abs(wrap180(rig.yaw - 180)) <= cfg["navigator"]["return_turn_tol_deg"] + 5


# ---------------------------------------------------------------- Danger: hold, then return
def danger_rig(cfg, lead_s=40.0, **kw):
    """Drive in for `lead_s`, then meet rising gas; returns the rig at the start of the hold."""
    rig = TripRig(cfg, length=90.0, x=20.0, **kw)
    rig.link = False
    rig.run(lead_s)
    t0 = rig.t
    rig.gas = lambda t: min(1500.0, 200.0 + 150.0 * (t - t0))
    rig.until(lambda: rig.cmd.state == HOLD, 15)
    return rig


def test_danger_stops_sweeps_holds_30_s_then_returns(cfg):
    rig = danger_rig(cfg)
    assert "GAS_BUILDUP" in [s[1] for s in rig.sweeps]
    i_hold = len(rig.hist) - 1
    assert rig.hist[i_hold - 1][1] == SWEEP           # the sweep finished first
    t_hold = rig.hist[i_hold][0]
    rig.run(29.8)
    assert rig.cmd.state == HOLD and rig.cmd.direction == DIR_IN
    assert all(h[3] == 0.0 and h[4] == 0.0 for h in rig.hist[i_hold:])
    rig.until(lambda: returning(rig), 0.5)
    assert rig.t - DT - t_hold == pytest.approx(cfg["navigator"]["danger_hold_s"], abs=0.11)
    assert rig.cmd.return_reason == "danger hold finished"
    assert len(rig.flushes) == 1
    rig.until(lambda: rig.cmd.state == CRUISE, 10)
    assert rig.facing_home() and rig.cmd.left > 0


def test_holds_and_sweeps_do_not_count_as_driving_time(cfg):
    rig = danger_rig(cfg)
    rig.until(lambda: returning(rig), 31)
    drove = DT * len([h for h in rig.hist if h[1] in (CRUISE, AVOID, BACKOFF) and h[2] == DIR_IN])
    assert rig.nav.out_s == pytest.approx(drove, abs=0.1)
    assert rig.nav.out_s < rig.t - 31.0               # the 30 s hold and the sweeps are not in it


def test_the_gas_being_left_does_not_stop_the_rover_again(cfg):
    rig = danger_rig(cfg)
    rig.until(lambda: returning(rig), 31)
    t0 = rig.t
    rig.gas = lambda t: min(2600.0, 1500.0 + 150.0 * (t - t0))   # still above 800 and rising again
    seen = set()
    for _ in range(int(10 / DT)):
        rig.step()
        seen.add(rig.fusion._active_prev and "GAS_BUILDUP" in rig.fusion._active_prev)
    assert True in seen                               # fusion did call it DANGER again ...
    assert HOLD not in [h[1] for h in rig.hist if h[0] >= t0]   # ... but the rover kept retreating
    assert rig.cmd.state == CRUISE and rig.cmd.left > 0


def test_danger_re_arms_after_the_gas_falls_below_the_threshold_and_rises_again(cfg):
    rig = danger_rig(cfg)
    rig.until(lambda: returning(rig), 31)
    rig.until(lambda: rig.cmd.state == CRUISE, 10)
    rig.gas = 100.0                                   # clear air: below the 800 ppm threshold
    rig.run(25.0)
    assert rig.cmd.state == CRUISE
    t0 = rig.t
    rig.gas = lambda t: min(1800.0, 100.0 + 200.0 * (t - t0))    # a second pocket on the way back
    rig.until(lambda: rig.cmd.state == HOLD, 15)
    assert [s[1] for s in rig.sweeps].count("GAS_BUILDUP") == 2
    t_hold = rig.t - DT
    rig.until(lambda: rig.cmd.state != HOLD, 31)
    assert rig.t - DT - t_hold == pytest.approx(30.0, abs=0.11)
    # ... then it keeps returning: same direction, same reason, no second turn-round, not stuck
    assert (rig.cmd.state, rig.cmd.direction) == (CRUISE, DIR_RETURN)
    assert rig.cmd.return_reason == "danger hold finished"
    assert rig.facing_home() and rig.cmd.left > 0
    assert STUCK not in rig.states()


def test_noise_around_the_threshold_during_the_hold_does_not_re_arm(cfg):
    """Found in Webots (gas_return n1s0): the rover held at ~800 ppm, one sample of 791 re-armed
    the stop and it held again 0.3 m further into the same pocket."""
    rig = TripRig(cfg, length=90.0, x=20.0)
    rig.link = False
    rig.run(40.0)
    t0 = rig.t
    rig.gas = lambda t: min(800.0, 200.0 + 150.0 * (t - t0)) + (-10.0 if int(t * 20) % 7 == 0 else 6.0)
    rig.until(lambda: rig.cmd.state == HOLD, 15)
    rig.until(lambda: returning(rig), 31)
    t1 = rig.t
    rig.gas = lambda t: min(1400.0, 800.0 + 100.0 * (t - t1))    # driving on: the reading climbs again
    rig.run(15.0)
    assert HOLD not in [h[1] for h in rig.hist if h[0] >= t1]
    assert cfg["navigator"]["danger_rearm_ratio"] == 0.8


def test_re_arm_needs_the_reading_below_80_percent_of_the_threshold(cfg):
    rig = danger_rig(cfg)
    rig.until(lambda: returning(rig), 31)
    rig.until(lambda: rig.cmd.state == CRUISE, 10)
    rig.gas = 700.0                                   # below 800 but not below 640: still the same gas
    rig.run(8.0)
    t0 = rig.t
    rig.gas = lambda t: min(1500.0, 700.0 + 150.0 * (t - t0))
    rig.run(10.0)
    assert HOLD not in [h[1] for h in rig.hist if h[0] >= t0]
    rig.gas = 600.0                                   # below 640: cleared
    rig.run(8.0)
    t1 = rig.t
    rig.gas = lambda t: min(1500.0, 600.0 + 150.0 * (t - t1))
    rig.until(lambda: rig.cmd.state == HOLD, 15)


def test_extreme_gas_is_also_left_after_the_hold(cfg):
    rig = TripRig(cfg, length=90.0, x=20.0)
    rig.link = False
    rig.run(40.0)
    rig.gas = 3500.0                                  # steady but extreme: DANGER the whole time
    rig.until(lambda: rig.cmd.state == HOLD, 5)
    rig.until(lambda: returning(rig), 31)
    rig.until(lambda: rig.cmd.state == CRUISE, 10)
    rig.run(5.0)
    assert rig.cmd.state == CRUISE and rig.cmd.left > 0          # still DANGER, but it is retreating


def test_operator_resume_from_hold_still_works_with_the_link_up(cfg):
    rig = danger_rig(cfg)
    rig.link = True
    rig.run(12.0)                                     # gas steady at 1500 by now: DANGER has lapsed
    rig.step(keys={"R"})
    rig.run(0.5)
    assert (rig.cmd.state, rig.cmd.direction) == (CRUISE, DIR_IN) and rig.cmd.left > 0


# ---------------------------------------------------------------- link and commands
def test_rover_commands_need_the_link(cfg):
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.run(1.0)
    for key in ("X", "SPACE", "M", "L", "C", "R"):
        c = rig.step(keys={key})
        rig.step()
        assert c.mode == AUTO and not c.lamp_toggle, key
    rig.run(1.5)
    assert rig.cmd.state == CRUISE and not rig.sweeps  # C did not start a sweep
    assert rig.step(keys={"D"}).drop_rock             # simulation control: always works
    rig.link = True
    rig.step()
    assert rig.step(keys={"X"}).mode == STOP


def test_manual_drive_keys_need_the_link(cfg):
    rig = TripRig(cfg, length=60.0, x=20.0, mode=MANUAL)
    rig.link = True
    assert rig.run(0.5, keys={"W"}).left > 0
    rig.link = False
    rig.run(0.6, keys={"W"})                          # dead-man: no command reaches the rover
    assert (rig.cmd.left, rig.cmd.right) == (0.0, 0.0)
    assert rig.cmd.mode == MANUAL


def test_r_from_home_starts_a_new_trip_in(cfg):
    rig = TripRig(cfg, length=8.0)
    rig.until(lambda: rig.cmd.state == HOME, 120)
    rig.step(keys={"R"})
    assert (rig.cmd.direction, rig.cmd.return_reason) == (DIR_IN, None)
    # route flipped back towards the mine (the turn-round ends within return_turn_tol_deg of it)
    assert abs(wrap180(rig.cmd.route)) <= cfg["navigator"]["return_turn_tol_deg"] + 3.0
    assert rig.nav.out_s < 0.2 and rig.nav.ret_s == 0.0
    rig.until(lambda: rig.cmd.state == CRUISE, 10)
    assert abs(wrap180(rig.yaw)) <= 45
    x0 = rig.x
    rig.run(3.0)
    assert rig.x > x0 + 0.5


def test_r_from_home_is_refused_with_a_low_battery_or_without_the_link(cfg):
    cfg["navigator"]["mission_time_s"] = 5.0
    rig = TripRig(cfg, length=60.0, x=20.0)
    rig.link = False
    rig.until(lambda: rig.cmd.state == HOME, 40)      # home by time, link still down
    rig.step(keys={"R"})
    rig.step()
    assert rig.cmd.state == HOME
    rig.link = True
    rig.battery = batt(RETURN)
    rig.step(keys={"R"})
    assert rig.cmd.state == HOME and "refused" in rig.cmd.note
    rig.step()
    rig.battery = batt(OK)
    rig.step(keys={"R"})
    assert rig.cmd.direction == DIR_IN and rig.cmd.state != HOME
