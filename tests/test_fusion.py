"""Fusion table tests: every proposal row, the roof-fall rules, hysteresis/hold, cooldown."""
from conftest import mk, run

from rover_core import CAUTION, DANGER, SAFE
from rover_core.fusion import (CLEAR, FLOOR_LOST, GAS_BUILDUP, GAS_ELEVATED, GAS_EXTREME, OBSTACLE,
                               ROOF_FALL, STALE_DATA, TILT, HazardFusion, least_squares_slope)


def last(seq):
    return seq[-1]


# ---------------------------------------------------------------- proposal row 4
def test_clear_is_safe(cfg):
    a = last(run(HazardFusion(cfg), 0, 3, lambda t: mk(t)))
    assert (a.level, a.reason) == (SAFE, CLEAR)
    assert a.sweep_request is None


def test_row4_ultrasonic_obstacle_floor_and_gas_normal_is_safe(cfg):
    out = run(HazardFusion(cfg), 0, 3, lambda t: mk(t, us_left=0.30, us_right=0.9))
    a = last(out)
    assert (a.level, a.reason) == (SAFE, OBSTACLE)
    assert "steer around" in a.response
    assert all(x.sweep_request is None for x in out)


def test_obstacle_threshold_is_35cm(cfg):
    assert last(run(HazardFusion(cfg), 0, 1, lambda t: mk(t, us_right=0.36))).reason == CLEAR
    assert last(run(HazardFusion(cfg), 0, 1, lambda t: mk(t, us_right=0.34))).reason == OBSTACLE
    assert last(run(HazardFusion(cfg), 0, 1, lambda t: mk(t, ir_front_left=True))).reason == OBSTACLE


# ---------------------------------------------------------------- proposal row 3
def test_row3_floor_lost_is_caution(cfg):
    out = run(HazardFusion(cfg), 0, 1, lambda t: mk(t, floor_lost_left=t >= 0.5))
    a = last(out)
    assert (a.level, a.reason) == (CAUTION, FLOOR_LOST)
    assert "back off" in a.response
    assert [x.sweep_request for x in out].count(FLOOR_LOST) == 1


def test_row3_tilt_too_steep_is_caution(cfg):
    assert last(run(HazardFusion(cfg), 0, 1, lambda t: mk(t, pitch_deg=19))).level == SAFE
    a = last(run(HazardFusion(cfg), 0, 1, lambda t: mk(t, pitch_deg=22)))
    assert (a.level, a.reason) == (CAUTION, TILT)
    a = last(run(HazardFusion(cfg), 0, 1, lambda t: mk(t, roll_deg=-23)))
    assert (a.level, a.reason) == (CAUTION, TILT)


# ---------------------------------------------------------------- proposal row 1
def test_row1_gas_above_threshold_and_rising_is_danger(cfg):
    out = run(HazardFusion(cfg), 0, 12, lambda t: mk(t, gas_ppm=200 + 100 * t))
    first = next(x for x in out if x.level == DANGER)
    assert first.reason == GAS_BUILDUP
    assert first.gas_ppm >= 800 and first.gas_slope >= 15
    assert "stop" in first.response
    assert 5.9 < first.t < 6.2          # crosses 800 ppm at t = 6 s


def test_gas_above_threshold_but_steady_is_not_danger(cfg):
    out = run(HazardFusion(cfg), 0, 10, lambda t: mk(t, gas_ppm=1200))
    assert all(x.level != DANGER for x in out)
    assert last(out).reason == GAS_ELEVATED          # addition: steady elevated gas = CAUTION


def test_gas_rising_but_below_threshold_is_not_danger(cfg):
    out = run(HazardFusion(cfg), 0, 6, lambda t: mk(t, gas_ppm=20 + 40 * t))
    assert all(x.level != DANGER for x in out)


def test_gas_slow_rise_below_slope_threshold_is_not_danger(cfg):
    out = run(HazardFusion(cfg), 0, 10, lambda t: mk(t, gas_ppm=900 + 5 * t))
    assert all(x.level != DANGER for x in out)


def test_least_squares_slope():
    assert abs(least_squares_slope([(t / 10, 3 + 25 * t / 10) for t in range(40)]) - 25) < 1e-9
    assert least_squares_slope([(0, 1)]) == 0.0


# ---------------------------------------------------------------- proposal row 2
def test_row2_impact_plus_new_obstacle_is_roof_fall(cfg):
    def make(t):
        return mk(t, impact=abs(t - 2.0) < 1e-6, us_left=(0.30 if t >= 2.3 else None))
    out = run(HazardFusion(cfg), 0, 4, make)
    first = next(x for x in out if x.level == DANGER)
    assert first.reason == ROOF_FALL
    assert 2.25 < first.t < 2.4
    assert "camera sweep" in first.response
    assert first.sweep_request == ROOF_FALL


def test_roof_fall_when_obstacle_was_30cm_farther(cfg):
    def make(t):
        return mk(t, impact=abs(t - 2.0) < 1e-6, us_left=(0.40 if t >= 2.2 else 1.20))
    assert any(x.reason == ROOF_FALL for x in run(HazardFusion(cfg), 0, 4, make))


def test_bumping_something_already_seen_is_not_roof_fall(cfg):
    # obstacle was already at 0.45 m before the impact and is at 0.40 m afterwards
    def make(t):
        return mk(t, impact=abs(t - 2.0) < 1e-6, us_left=(0.45 if t < 2.0 else 0.40))
    out = run(HazardFusion(cfg), 0, 6, make)
    assert all(x.reason != ROOF_FALL for x in out)
    assert all(x.level == SAFE for x in out)


def test_impact_without_new_obstacle_stays_safe(cfg):
    out = run(HazardFusion(cfg), 0, 6, lambda t: mk(t, impact=abs(t - 2.0) < 1e-6))
    assert all(x.level == SAFE for x in out)


def test_new_obstacle_too_late_after_impact_is_not_roof_fall(cfg):
    def make(t):
        return mk(t, impact=abs(t - 2.0) < 1e-6, us_left=(0.30 if t >= 5.5 else None))
    assert all(x.reason != ROOF_FALL for x in run(HazardFusion(cfg), 0, 8, make))


def test_new_obstacle_beyond_60cm_is_not_roof_fall(cfg):
    def make(t):
        return mk(t, impact=abs(t - 2.0) < 1e-6, us_left=(0.70 if t >= 2.3 else None))
    assert all(x.reason != ROOF_FALL for x in run(HazardFusion(cfg), 0, 6, make))


def test_obstacle_without_impact_is_not_roof_fall(cfg):
    out = run(HazardFusion(cfg), 0, 6, lambda t: mk(t, us_left=(0.30 if t >= 2.3 else None)))
    assert all(x.level == SAFE for x in out)


def test_obstacle_swinging_into_view_after_a_bump_is_not_roof_fall(cfg):
    # (addition) the rover bumps, then turns 40 degrees and the sonar picks up a wall
    def make(t):
        return mk(t, impact=abs(t - 2.0) < 1e-6, yaw_deg=(40.0 if t >= 2.3 else 0.0),
                  us_left=(0.40 if t >= 2.5 else None))
    assert all(x.level == SAFE for x in run(HazardFusion(cfg), 0, 6, make))


def test_roof_fall_seen_only_by_forward_ir(cfg):
    # (addition) rock lands in the sonar blind strip dead ahead: the front IR is the evidence
    def make(t):
        return mk(t, impact=abs(t - 2.0) < 1e-6, ir_front_left=t >= 2.4)
    assert any(x.reason == ROOF_FALL for x in run(HazardFusion(cfg), 0, 4, make))

    def already(t):       # IR was already tripped before the knock: just a bump
        return mk(t, impact=abs(t - 2.0) < 1e-6, ir_front_left=t >= 1.8)
    assert all(x.reason != ROOF_FALL for x in run(HazardFusion(cfg), 0, 6, already))


def test_roof_fall_latches_15s(cfg):
    def make(t):
        return mk(t, impact=abs(t - 2.0) < 1e-6, us_left=(0.30 if 2.3 <= t < 3.0 else None))
    out = run(HazardFusion(cfg), 0, 20, make)
    t_on = next(x.t for x in out if x.reason == ROOF_FALL)
    t_off = max(x.t for x in out if x.reason == ROOF_FALL)
    assert 14.8 < t_off - t_on < 15.2
    assert last(out).level == SAFE


# ---------------------------------------------------------------- additions
def test_addition_extreme_gas_is_danger_even_if_steady(cfg):
    a = last(run(HazardFusion(cfg), 0, 8, lambda t: mk(t, gas_ppm=3200)))
    assert (a.level, a.reason) == (DANGER, GAS_EXTREME)


def test_addition_elevated_gas_caution_with_hysteresis(cfg):
    f = HazardFusion(cfg)
    assert last(run(f, 0, 2, lambda t: mk(t, gas_ppm=290))).level == SAFE
    assert last(run(f, 2.05, 4, lambda t: mk(t, gas_ppm=310))).reason == GAS_ELEVATED
    # stays on between 240 (80 %) and 300 ...
    assert last(run(f, 4.05, 8, lambda t: mk(t, gas_ppm=250))).reason == GAS_ELEVATED
    # ... and clears below 240
    assert last(run(f, 8.05, 9, lambda t: mk(t, gas_ppm=230))).level == SAFE


def test_addition_stale_data_is_caution_and_stop(cfg):
    out = run(HazardFusion(cfg), 0, 3, lambda t: mk(t, age_gas=(0.6 if t > 2 else 0.05)))
    a = last(out)
    assert (a.level, a.reason) == (CAUTION, STALE_DATA)
    assert a.stop
    assert all(x.sweep_request is None for x in out)
    assert not out[20].stop


def test_stale_not_judged_during_startup_grace(cfg):
    a = HazardFusion(cfg).update(mk(0.0, age_gas=99))
    assert a.level == SAFE


def test_gas_warmup_none_is_not_a_hazard(cfg):
    a = last(run(HazardFusion(cfg), 0, 2, lambda t: mk(t, gas_ppm=None)))
    assert a.level == SAFE and a.gas_ppm is None


# ---------------------------------------------------------------- hold / cooldown / priority
def test_findings_hold_for_2s(cfg):
    f = HazardFusion(cfg)
    out = run(f, 0, 5, lambda t: mk(t, floor_lost_right=1.0 <= t < 1.2))
    on = [x.t for x in out if x.reason == FLOOR_LOST]
    assert abs(min(on) - 1.0) < 1e-6
    assert abs(max(on) - 3.10) < 0.06    # last true at 1.15 s, held for 2 s
    assert last(out).level == SAFE
    assert FLOOR_LOST not in out[30].raw and FLOOR_LOST in out[30].hazards    # t=1.5: held, not raw


def test_sweep_cooldown_is_20s_per_hazard_type(cfg):
    f = HazardFusion(cfg)

    def make(t):
        pit = (1 <= t < 1.2) or (8 <= t < 8.2) or (25 <= t < 25.2)
        tilt = 10 <= t < 10.2
        return mk(t, floor_lost_left=pit, pitch_deg=25 if tilt else 0)
    out = run(f, 0, 30, make)
    sweeps = [(round(x.t, 2), x.sweep_request) for x in out if x.sweep_request]
    assert sweeps == [(1.0, FLOOR_LOST), (10.0, TILT), (25.0, FLOOR_LOST)]   # pit at 8 s is inside the cooldown


def test_danger_outranks_caution_outranks_safe(cfg):
    f = HazardFusion(cfg)
    out = run(f, 0, 12, lambda t: mk(t, gas_ppm=200 + 100 * t, floor_lost_left=True, us_left=0.2))
    a = last(out)
    assert a.level == DANGER and a.reason == GAS_BUILDUP
    assert a.hazards[:2] == [GAS_BUILDUP, FLOOR_LOST] and OBSTACLE in a.hazards


def test_assessment_carries_readings_every_cycle(cfg):
    r = mk(1.0, us_left=0.5, gas_ppm=123.0, pitch_deg=3)
    a = HazardFusion(cfg).update(r)
    assert a.readings is r and a.front_dist == 0.5 and a.gas_ppm == 123.0 and a.tilt_deg == 3
