"""Battery: Uno frame conversion, 5 s average, latched thresholds, and the simulated pack."""
import copy

import pytest
from conftest import DT

from rover_core import config as rc_config
from rover_core.battery import (CRITICAL, OK, RETURN, WARN, BatteryMonitor, BatterySim, counts_to_volts,
                                volts_to_counts)


def feed(mon, t0, secs, volts):
    """Feed a constant pack voltage for `secs`; returns (t_end, last status)."""
    b = mon.b
    st = None
    n = int(round(secs / DT))
    for i in range(n):
        st = mon.update(round(t0 + i * DT, 6), volts_to_counts(volts, b))
    return round(t0 + n * DT, 6), st


@pytest.fixture
def mon(cfg):
    return BatteryMonitor(cfg)


# ---------------------------------------------------------------- conversion
def test_frame_field_is_adc_counts_times_ten_not_volts(cfg):
    b = cfg["battery"]
    # full pack: 12.6 V / 3.2 = 3.9375 V on A1 -> 805.6 counts of 1023 at 5.0 V -> batt_x10 8056
    assert volts_to_counts(12.6, b) == 8056
    assert counts_to_volts(8056, b) == pytest.approx(12.6, abs=0.002)
    # V = (batt_x10 / 10) / 1023 * 5.0 * 3.2 * scale
    assert counts_to_volts(7000, b) == pytest.approx(700 / 1023 * 5.0 * 3.2)
    assert counts_to_volts(10230, b) == pytest.approx(16.0)


def test_calibration_scale_multiplies_the_voltage(cfg):
    b = dict(cfg["battery"], scale=1.02)
    assert counts_to_volts(7000, b) == pytest.approx(700 / 1023 * 5.0 * 3.2 * 1.02)


def test_config_holds_the_agreed_starting_thresholds(cfg):
    b = cfg["battery"]
    assert (b["return_v"], b["warn_v"], b["critical_v"]) == (10.5, 10.2, 9.6)
    assert (b["adc_ref_v"], b["adc_max"], b["divider_ratio"], b["scale"]) == (5.0, 1023, 3.2, 1.0)
    assert b["avg_window_s"] == 5.0


@pytest.mark.parametrize("bad", [dict(return_v=10.2), dict(warn_v=9.6), dict(critical_v=10.3), dict(warn_v=10.6)])
def test_config_load_fails_unless_return_above_warn_above_critical(bad):
    with pytest.raises(ValueError, match="return_v > warn_v > critical_v"):
        rc_config.load(overrides={"battery": bad})


# ---------------------------------------------------------------- 5 s average
def test_healthy_pack_reads_ok(mon):
    _, st = feed(mon, 0.0, 10.0, 11.4)
    assert st.level == OK
    assert st.avg_v == pytest.approx(11.4, abs=0.01)
    assert st.volts == pytest.approx(11.4, abs=0.01)


def test_nothing_is_judged_before_the_window_is_full(mon):
    _, st = feed(mon, 0.0, 4.0, 9.0)                  # far below critical, but only 4 s of data
    assert st.level == OK and st.avg_v is None
    _, st = feed(mon, 4.0, 1.2, 9.0)
    assert st.level == CRITICAL


def test_a_short_load_sag_does_not_trigger(mon):
    t, _ = feed(mon, 0.0, 8.0, 10.9)
    t, st = feed(mon, t, 1.5, 10.2)                   # motors pull the pack down for 1.5 s
    assert st.volts < 10.5 and st.avg_v > 10.5
    t, st = feed(mon, t, 6.0, 10.9)
    assert st.level == OK


def test_sustained_low_voltage_triggers_return_on_the_average(mon):
    t, _ = feed(mon, 0.0, 6.0, 10.9)
    t, st = feed(mon, t, 2.0, 10.4)                   # average still above 10.5
    assert st.level == OK
    t, st = feed(mon, t, 3.2, 10.4)
    assert st.level == RETURN
    assert st.t_return is not None


def test_exactly_at_the_threshold_is_not_below_it(mon):
    b = mon.b
    counts = volts_to_counts(10.5, b)
    v = counts_to_volts(counts, b)                    # what 10.5 V looks like after the ADC
    mon.b = dict(b, return_v=v)
    st = None
    for i in range(200):
        st = mon.update(i * DT, counts)
    assert st.level == OK


def test_levels_step_down_return_then_warn_then_critical(mon):
    t, st = feed(mon, 0.0, 6.0, 10.45)
    assert st.level == RETURN
    t, st = feed(mon, t, 6.0, 10.15)
    assert st.level == WARN
    t, st = feed(mon, t, 6.0, 9.5)
    assert st.level == CRITICAL
    assert st.t_return < st.t_warn < st.t_critical


# ---------------------------------------------------------------- latch
def test_return_latches_when_the_voltage_recovers(mon):
    t, _ = feed(mon, 0.0, 6.0, 10.9)
    t, st = feed(mon, t, 6.0, 10.3)                   # driving: below the return threshold
    assert st.level == RETURN
    t, st = feed(mon, t, 20.0, 11.0)                  # motors stop, the pack recovers
    assert st.avg_v > 10.9
    assert st.level == RETURN                         # ... but the trigger stays


def test_critical_latches_when_the_voltage_recovers(mon):
    t, st = feed(mon, 0.0, 6.0, 9.4)
    assert st.level == CRITICAL
    t, st = feed(mon, t, 20.0, 11.5)
    assert st.level == CRITICAL


def test_a_deep_one_second_dip_latches_return_but_never_critical(mon):
    t, _ = feed(mon, 0.0, 6.0, 10.8)
    t, st = feed(mon, t, 1.0, 9.0)                    # 1 s at 9.0 V: 5 s mean = 10.44 -> RETURN, not CRITICAL
    assert st.level == RETURN
    t, st = feed(mon, t, 10.0, 10.8)
    assert st.level == RETURN                         # latched, and never escalated


def test_missing_samples_are_ignored(mon):
    t, _ = feed(mon, 0.0, 6.0, 11.0)
    st = mon.update(t, None)
    assert st.level == OK and st.avg_v == pytest.approx(11.0, abs=0.01)


# ---------------------------------------------------------------- simulated pack
def test_sim_full_pack_is_12v6_at_rest_and_outputs_counts(cfg):
    sim = BatterySim(cfg, charge=1.0, seed=1)
    x10 = sim.update(DT, 0.0, 0.0)
    assert isinstance(x10, int) and 7000 < x10 < 8200           # ADC counts x 10, not volts
    assert sim.open_circuit_v() == pytest.approx(12.6, abs=0.01)
    assert counts_to_volts(x10, cfg["battery"]) == pytest.approx(12.6, abs=0.15)


def test_sim_sags_a_few_tenths_under_motor_load(cfg):
    sim = BatterySim(cfg, charge=0.6, seed=1)
    idle = sum(sim.terminal_v(0.0, 0.0) for _ in range(5)) / 5
    drive = sum(sim.terminal_v(0.7, 0.7) for _ in range(5)) / 5
    assert 0.15 <= idle - drive <= 0.5


def test_sim_voltage_falls_as_charge_is_used(cfg):
    sim = BatterySim(cfg, charge=1.0, seed=1)
    v0 = sim.terminal_v(0.7, 0.7)
    for _ in range(int(1800 / 1.0)):                  # half an hour of driving in 1 s steps
        sim.update(1.0, 0.7, 0.7)
    s = cfg["battery"]["sim"]
    used = s["drive_a"] * 0.5 * 1000 / s["capacity_mah"]
    assert sim.charge == pytest.approx(1.0 - used, abs=0.01)
    assert sim.terminal_v(0.7, 0.7) < v0 - 0.5


def test_sim_idle_draw_is_smaller_than_driving_draw(cfg):
    a, b = BatterySim(cfg, charge=1.0, seed=1), BatterySim(cfg, charge=1.0, seed=1)
    for _ in range(600):
        a.update(1.0, 0.0, 0.0)
        b.update(1.0, 0.7, 0.7)
    s = cfg["battery"]["sim"]
    assert (1.0 - a.charge) / (1.0 - b.charge) == pytest.approx(s["idle_a"] / s["drive_a"], rel=0.02)


def test_sim_starting_charge_and_sudden_drop(cfg):
    c = copy.deepcopy(cfg)
    c["battery"]["sim"]["start_charge"] = 0.5
    sim = BatterySim(c, seed=1)
    assert sim.charge == 0.5
    v_half = sim.open_circuit_v()
    sim.set_charge(0.02)                              # the test hook for a sudden drop
    assert sim.open_circuit_v() < v_half - 1.0
    sim.set_charge(-1)
    assert sim.charge == 0.0


def test_sim_and_monitor_agree_and_a_low_pack_latches_return_while_driving(cfg):
    sim = BatterySim(cfg, charge=1.0, seed=3)
    mon = BatteryMonitor(cfg)
    st = None
    for i in range(200):
        st = mon.update(i * DT, sim.update(DT, 0.7, 0.7))
    assert st.level == OK
    assert st.avg_v == pytest.approx(sim.terminal_v(0.7, 0.7, noise=False), abs=0.05)
    # nearly empty: under load the average is below 10.5 V; at rest it climbs back above it
    sim.set_charge(0.07)
    t = 200 * DT
    for i in range(160):
        st = mon.update(t + i * DT, sim.update(DT, 0.7, 0.7))
    assert st.level in (RETURN, WARN)
    level = st.level
    for i in range(400):
        st = mon.update(t + 8 + i * DT, sim.update(DT, 0.0, 0.0))
    assert st.level == level                          # latched although the resting voltage is higher
