"""Navigator tests: transitions, guards, forward block, dead-man, keyboard modes."""
import pytest
from conftest import DT, mk

from rover_core.fusion import HazardFusion
from rover_core.navigator import (AUTO, AVOID, BACKOFF, CRUISE, HOLD, MANUAL, STOP, STUCK, SWEEP,
                                  MotorGuard, Navigator, snap)


class Rig:
    """Navigator + fusion driven at 20 Hz with scripted readings."""

    def __init__(self, cfg, mode=AUTO):
        cfg["navigator"]["start_mode"] = mode
        self.cfg = cfg
        self.nav = Navigator(cfg)
        self.fusion = HazardFusion(cfg)
        self.t = 0.0
        self.sweeps = []
        self.sweep_busy_until = -1.0
        self.cmd = None

    def step(self, keys=(), **kw):
        r = mk(self.t, **kw)
        a = self.fusion.update(r)
        busy = self.t < self.sweep_busy_until
        self.cmd = self.nav.step(self.t, r, a, keys=keys, sweep_busy=busy)
        if self.cmd.start_sweep:
            self.sweeps.append((round(self.t, 2), self.cmd.start_sweep))
            self.sweep_busy_until = self.t + 1.0        # pretend a sweep takes 1 s
        self.t = round(self.t + DT, 6)
        return self.cmd

    def run(self, secs, keys=(), **kw):
        make = kw.pop("make", None)
        for _ in range(int(round(secs / DT))):
            extra = make(self.t) if make else {}
            self.step(keys=keys, **{**kw, **extra})
        return self.cmd


@pytest.fixture
def rig(cfg):
    return Rig(cfg)


# ---------------------------------------------------------------- cruise
def test_auto_starts_cruising_forward(rig):
    c = rig.run(1.0)
    assert (c.mode, c.state) == (AUTO, CRUISE)
    assert c.left == pytest.approx(0.7) and c.right == pytest.approx(0.7)


def test_waits_while_not_ready(cfg):
    nav = Navigator(cfg)
    f = HazardFusion(cfg)
    r = mk(0.0)
    c = nav.step(0.0, r, f.update(r), ready=False)
    assert c.state == "CALIBRATING" and (c.left, c.right) == (0.0, 0.0)


def test_ramps_up_but_stops_instantly(rig):
    first = rig.step()
    assert 0 < first.left < 0.7                       # ramping
    rig.run(1.0)
    assert rig.cmd.left == pytest.approx(0.7)
    c = rig.step(age_gas=5.0)                         # stale -> stop this very cycle
    assert (c.left, c.right) == (0.0, 0.0)


def test_slows_and_steers_to_the_open_side(rig):
    c = rig.run(1.0, us_right=0.5)                    # obstacle to the right, left open
    assert c.state == CRUISE
    assert c.right > c.left                           # turning left
    assert max(c.left, c.right) < 0.8
    c2 = Rig(rig.cfg).run(1.0, us_left=0.5)
    assert c2.left > c2.right                         # turning right


def test_commands_never_sit_in_the_dead_zone(rig):
    for d in (0.75, 0.6, 0.5, 0.4, 0.33):
        c = Rig(rig.cfg).run(1.0, us_right=d)
        for v in (c.left, c.right):
            assert v == 0.0 or abs(v) >= 0.42
    assert snap(0.3, 0.42) == 0.42 and snap(0.1, 0.42) == 0.0 and snap(-0.3, 0.42) == -0.42


def test_heading_hold_corrects_drift(rig):
    rig.run(0.5)
    c = rig.run(0.5, yaw_deg=-5.0)                    # drifted right -> steer left
    assert c.right > c.left


# ---------------------------------------------------------------- avoid
def test_obstacle_within_30cm_spins_to_the_open_side(rig):
    rig.run(0.5)
    c = rig.run(0.3, us_right=0.25, us_left=0.9)
    assert c.state == AVOID
    assert c.left < 0 < c.right                       # spin left, away from the right-hand obstacle


def test_front_ir_triggers_avoid_away_from_that_side(rig):
    rig.run(0.5)
    c = rig.run(0.3, ir_front_left=True)
    assert c.state == AVOID and c.left > 0 > c.right  # spin right


def test_avoid_exits_after_clear_for_quarter_second(rig):
    rig.run(0.5)
    rig.run(0.5, us_right=0.25, us_left=0.9)
    assert rig.cmd.state == AVOID
    rig.run(0.2, us_right=0.6, us_left=1.5)           # clear, but not yet for 0.25 s
    assert rig.cmd.state == AVOID
    rig.run(0.2, us_right=0.6, us_left=1.5)
    assert rig.cmd.state == CRUISE


def test_avoid_needs_real_room_ahead_not_just_50cm(rig):
    rig.run(0.5)
    rig.run(0.5, us_right=0.25, us_left=0.25)
    rig.run(1.0, us_right=0.55, us_left=0.6)          # facing a side wall: >50 cm but nothing open
    assert rig.cmd.state == AVOID


def test_avoid_reverses_direction_then_times_out_stuck(rig):
    rig.run(0.5)
    yaw = {"v": 0.0}

    def make(t):
        if rig.cmd is not None and rig.cmd.state == AVOID:
            yaw["v"] += 4.0 * (1 if rig.cmd.right > rig.cmd.left else -1)   # 80 deg/s
        return dict(us_left=0.25, us_right=0.2, yaw_deg=yaw["v"])
    dirs = []
    for _ in range(int(9 / DT)):
        rig.step(**make(rig.t))
        if rig.cmd.state == AVOID:
            dirs.append(1 if rig.cmd.right > rig.cmd.left else -1)
    assert 1 in dirs and -1 in dirs                   # tried both ways
    assert rig.cmd.state == STUCK
    assert (rig.cmd.left, rig.cmd.right) == (0.0, 0.0)


# ---------------------------------------------------------------- backoff
def run_backoff(rig, side_kw, yaw_rate=90.0, secs=14):
    """Drive into a pit edge; simulate yaw from the spin command. Returns visited (state, sign) list."""
    yaw = {"v": 0.0}
    seen = []
    rig.run(0.5)
    t_edge = rig.t
    for _ in range(int(secs / DT)):
        lost = rig.t - t_edge < 0.4 or (rig.cmd.state == BACKOFF and rig.nav._bo["phase"] == "WAIT")
        kw = dict(side_kw) if lost else {}
        if rig.cmd.left * rig.cmd.right < 0:
            yaw["v"] += yaw_rate * DT * (1 if rig.cmd.right > 0 else -1)
        c = rig.step(yaw_deg=yaw["v"], **kw)
        seen.append((c.state, c.left, c.right, yaw["v"]))
    return seen


def test_floor_lost_cuts_forward_drive_the_same_cycle(rig):
    rig.run(1.0)
    c = rig.step(floor_raw_left=True)                 # raw, not yet debounced
    assert c.left <= 0 and c.right <= 0


def test_pit_on_left_backs_off_sweeps_reverses_turns_right(rig):
    seen = run_backoff(rig, dict(floor_lost_left=True, floor_raw_left=True))
    states = [s[0] for s in seen]
    assert BACKOFF in states and SWEEP in states
    assert rig.sweeps and rig.sweeps[0][1] == "FLOOR_LOST"
    rev = [s for s in seen if s[0] == BACKOFF and s[1] < 0 and s[2] < 0]
    assert 0.6 <= len(rev) * DT <= 0.85               # reversed ~0.7 s
    turn = [s for s in seen if s[0] == BACKOFF and s[1] > 0 > s[2]]
    assert turn                                       # spun right (away from the left edge)
    assert -75 < min(s[3] for s in seen) < -55        # ~60 deg to the right ...
    side = [s for s in seen if s[0] == BACKOFF and s[1] > 0 and s[2] > 0]
    assert 1.0 <= len(side) * DT <= 1.5               # ... a short sidestep ...
    assert abs(seen[-1][3]) < 12                      # ... then facing the original way again
    assert states[-1] == CRUISE
    # while biased away from the left edge the rover hugs the other side
    assert rig.nav._bias_side == "left"


def test_pit_on_both_sides_turns_160(rig):
    seen = run_backoff(rig, dict(floor_lost_left=True, floor_lost_right=True, floor_raw_left=True, floor_raw_right=True))
    assert 150 < abs(seen[-1][3]) < 175


def test_bias_keeps_rover_away_from_the_pit_side(rig):
    yaw = run_backoff(rig, dict(floor_lost_left=True, floor_raw_left=True), secs=7)[-1][3]
    assert rig.cmd.state == CRUISE and rig.nav._bias_until > rig.t
    c = rig.step(yaw_deg=yaw)                         # nothing in view, but still inside the bias window
    assert c.left > c.right                           # steering right, away from the left edge
    rig.run(9.0, yaw_deg=yaw)                         # bias expired: straight again
    assert rig.cmd.left == pytest.approx(rig.cmd.right)


def test_never_forward_while_floor_lost_in_any_state(rig):
    rig.run(0.5)
    for _ in range(int(6 / DT)):
        c = rig.step(floor_lost_left=True, floor_raw_left=True)
        assert c.left <= 0 and c.right <= 0


def test_three_pits_in_two_minutes_is_stuck_route_blocked(rig):
    for i in range(3):
        rig.run(0.3, floor_lost_left=True, floor_lost_right=True, floor_raw_left=True, floor_raw_right=True)
        if i < 2:
            assert rig.cmd.state in (BACKOFF, SWEEP)
            yaw = 0.0
            for _ in range(int(12 / DT)):
                if rig.cmd.left * rig.cmd.right < 0:
                    yaw += 6.0
                rig.step(yaw_deg=yaw)
            assert rig.cmd.state == CRUISE
    assert rig.t < 120
    rig.run(1.5)                                      # (a sweep of the third pit may run first)
    assert rig.cmd.state == STUCK
    assert "route blocked" in rig.cmd.note
    assert (rig.cmd.left, rig.cmd.right) == (0.0, 0.0)


def test_tilt_backs_off_until_level(rig):
    rig.run(0.5)
    rig.run(0.2, pitch_deg=24)
    assert rig.cmd.state in (BACKOFF, SWEEP)
    rig.run(1.2, pitch_deg=24)                        # sweep (1 s) then reversing
    assert rig.cmd.state == BACKOFF and rig.cmd.left < 0 and rig.cmd.right < 0
    rig.run(1.0, pitch_deg=24)                        # still tilted: keeps reversing past 0.7 s
    assert rig.cmd.left < 0 and rig.cmd.right < 0
    rig.run(0.2, pitch_deg=2)                         # level again -> turn
    assert rig.cmd.left * rig.cmd.right < 0
    assert ("TILT" in [s[1] for s in rig.sweeps])


# ---------------------------------------------------------------- danger / hold / sweep
def test_danger_sweeps_then_holds_until_r(rig):
    rig.run(1.0)
    rig.run(8.0, make=lambda t: dict(gas_ppm=200 + 150 * t))
    assert ("GAS_BUILDUP" in [s[1] for s in rig.sweeps])
    assert rig.cmd.state == HOLD
    assert (rig.cmd.left, rig.cmd.right) == (0.0, 0.0)
    rig.run(5.0, gas_ppm=1500)                        # steady now: DANGER lapses, HOLD stays
    assert rig.cmd.state == HOLD
    rig.step(keys={"R"}, gas_ppm=1500)
    rig.run(0.5, gas_ppm=1500)
    assert rig.cmd.state == CRUISE and rig.cmd.left > 0


def test_resume_refused_while_still_danger(rig):
    rig.run(1.0)
    rig.run(3.0, gas_ppm=3500)
    assert rig.cmd.state in (HOLD, SWEEP)
    rig.run(2.0, gas_ppm=3500)
    rig.step(keys={"R"}, gas_ppm=3500)
    assert rig.cmd.state == HOLD and "refused" in rig.cmd.note


def test_caution_sweep_pauses_then_resumes(rig):
    rig.run(1.0)
    rig.run(0.2, gas_ppm=400)
    assert rig.cmd.state == SWEEP and rig.cmd.left == 0
    assert rig.sweeps[-1][1] == "GAS_ELEVATED"
    rig.run(1.5, gas_ppm=400)
    assert rig.cmd.state == CRUISE and rig.cmd.left > 0


def test_c_key_requests_a_sweep(rig):
    rig.run(1.0)
    rig.step(keys={"C"})
    rig.step()
    assert rig.sweeps and rig.sweeps[-1][1] == "MANUAL"


def test_stale_data_stops(rig):
    rig.run(1.5)
    c = rig.run(0.5, age_us=0.8)
    assert (c.left, c.right) == (0.0, 0.0)
    c = rig.run(3.0)                                  # data back; finding held 2 s, then go
    assert c.left > 0


# ---------------------------------------------------------------- dead-man
def test_motor_guard_deadman_400ms():
    g = MotorGuard(0.4)
    g.set(1.0, 0.7, 0.7)
    assert g.get(1.2) == (0.7, 0.7)
    assert g.get(1.39) == (0.7, 0.7)
    assert g.get(1.45) == (0.0, 0.0)                  # no fresh command for > 0.4 s
    g.set(2.0, 0.7, -0.7)
    assert g.get(2.0, floor_lost=True) == (0.0, -0.7)  # forward cut the same step


def test_manual_deadman(cfg):
    rig = Rig(cfg, mode=MANUAL)
    rig.run(0.5, keys={"W"})
    assert rig.cmd.left > 0.4
    rig.run(0.3)                                      # key released < 0.4 s ago: command still valid
    assert rig.cmd.left > 0
    rig.run(0.3)
    assert (rig.cmd.left, rig.cmd.right) == (0.0, 0.0)


# ---------------------------------------------------------------- manual + keyboard
def test_manual_drive_keys(cfg):
    rig = Rig(cfg, mode=MANUAL)
    c = rig.run(0.5, keys={"W"})
    assert c.left == pytest.approx(c.right) and c.left > 0
    c = Rig(cfg, mode=MANUAL).run(0.5, keys={"S"})
    assert c.left < 0 and c.right < 0
    c = Rig(cfg, mode=MANUAL).run(0.5, keys={"A"})
    assert c.left < 0 < c.right                       # spin left
    c = Rig(cfg, mode=MANUAL).run(0.5, keys={"D"})
    assert c.left > 0 > c.right
    c = Rig(cfg, mode=MANUAL).run(0.5, keys={"W", "A"})
    assert 0 < c.left < c.right


def test_manual_refuses_forward_over_a_drop(cfg):
    rig = Rig(cfg, mode=MANUAL)
    c = rig.run(0.5, keys={"W"}, floor_lost_right=True, floor_raw_right=True)
    assert (c.left, c.right) == (0.0, 0.0) and c.refused == "drop ahead"
    c = rig.run(0.5, keys={"S"}, floor_lost_right=True, floor_raw_right=True)
    assert c.left < 0 and c.right < 0                 # reversing away is allowed


def test_manual_refuses_forward_within_15cm(cfg):
    rig = Rig(cfg, mode=MANUAL)
    c = rig.run(0.5, keys={"W"}, us_left=0.12)
    assert (c.left, c.right) == (0.0, 0.0) and "15 cm" in c.refused
    c = Rig(cfg, mode=MANUAL).run(0.5, keys={"W"}, ir_front_right=True)
    assert (c.left, c.right) == (0.0, 0.0)
    c = Rig(cfg, mode=MANUAL).run(0.5, keys={"W"}, us_left=0.4)
    assert c.left > 0


def test_mode_keys_and_estop(rig):
    rig.run(1.0)
    c = rig.step(keys={"SPACE"})
    assert c.mode == STOP and (c.left, c.right) == (0.0, 0.0)
    assert rig.run(0.5).left == 0.0
    c = rig.step(keys={"M"})
    assert c.mode == MANUAL
    rig.step()
    c = rig.step(keys={"A"})                          # in MANUAL, A steers - it does not switch mode
    assert c.mode == MANUAL
    rig.step()
    rig.step(keys={"X"})
    assert rig.cmd.mode == STOP
    rig.step()
    c = rig.step(keys={"A"})                          # outside MANUAL, A = AUTO
    assert c.mode == AUTO
    rig.run(0.5)
    assert rig.cmd.state == CRUISE and rig.cmd.left > 0


def test_d_key_drops_rock_outside_manual_and_l_toggles_lamp(rig):
    rig.run(0.5)
    assert rig.step(keys={"D"}).drop_rock
    assert not rig.step(keys={"D"}).drop_rock         # held key: one request only
    rig.step()
    assert rig.step(keys={"L"}).lamp_toggle
    m = Rig(rig.cfg, mode=MANUAL)
    assert not m.step(keys={"D"}).drop_rock
    assert m.step(keys={"D", "SHIFT_D"}).drop_rock
