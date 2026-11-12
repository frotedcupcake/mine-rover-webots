import json
import math
import os

import numpy as np
from conftest import mk

from rover_core import caption
from rover_core.fusion import HazardFusion
from rover_core.gas_model import GasField, GasSensorModel, GasSource
from rover_core.hazard_log import CSV_FIELDS, HazardLog
from rover_core.navigator import Command
from rover_core.sweep import SweepSequencer


# ---------------------------------------------------------------- sweep
def simulate_sweep(seq, speed_deg_s=60.0, dt=0.05, t0=10.0):
    pan, t, shots, lamp_on = 0.0, t0, [], []
    seq.start(t, "TEST")
    for _ in range(2000):
        st = seq.update(t, pan)
        if st.finished:
            break
        lamp_on.append(st.lamp)
        if st.capture is not None:
            shots.append((st.capture, st.angle_deg, round(pan, 1), t))
        step = speed_deg_s * dt
        pan += max(-step, min(step, st.target_deg - pan))
        t += dt
    return shots, pan, t - t0, lamp_on


def test_sweep_visits_five_angles_and_returns_to_zero():
    seq = SweepSequencer()
    shots, pan, dur, lamp = simulate_sweep(seq)
    assert [s[1] for s in shots] == [-90, -45, 0, 45, 90]
    assert all(abs(s[2] - s[1]) <= 2.0 for s in shots)     # camera really was there
    assert abs(pan) <= 2.0 and not seq.busy and seq.count == 1
    assert all(lamp)                                       # lamp on throughout
    assert 7.0 < dur < 12.0                                # 360 deg of travel at 60 deg/s + settles


def test_sweep_settles_before_each_shot():
    seq = SweepSequencer(settle_s=0.35)
    shots, *_ = simulate_sweep(seq)
    # -90 -> -45 takes 0.75 s of travel, then >= 0.35 s settle
    assert shots[1][3] - shots[0][3] >= 0.75 + 0.35 - 0.06


def test_sweep_cannot_restart_while_busy_and_times_out():
    seq = SweepSequencer(timeout_s=5.0)
    assert seq.start(0.0, "A")
    assert not seq.start(0.1, "B")
    st = seq.update(6.0, 0.0)          # pan never moved
    assert st.finished and not seq.busy


# ---------------------------------------------------------------- gas
def test_gas_plume_is_gaussian_and_ramps_up():
    src = GasSource(position=[5, 0, 0], peak_ppm=2000, sigma_m=1.0, ramp_up_s=4.0, trigger_distance_m=3.0)
    field = GasField([src])
    assert field.ppm_at([0, 0, 0], 0.0) == 0.0             # outside trigger distance: dormant
    assert src.triggered_at is None
    field.ppm_at([2.5, 0, 0], 10.0)                        # trigger
    assert src.triggered_at == 10.0
    assert field.ppm_at([5, 0, 0], 12.0) == 1000.0         # half way up the ramp, at the source
    assert field.ppm_at([5, 0, 0], 20.0) == 2000.0
    assert abs(field.ppm_at([4, 0, 0], 20.0) - 2000 * math.exp(-0.5)) < 1e-6
    assert abs(field.ppm_at([5, 2, 0], 20.0) - 2000 * math.exp(-2.0)) < 1e-6


def test_gas_sources_load_from_yaml(tmp_path):
    p = tmp_path / "g.yaml"
    p.write_text("sources:\n- name: a\n  position: [1, 2, 0]\n  peak_ppm: 900\n  sigma_m: 0.5\n"
                 "  ramp_up_s: 0\n  trigger_distance_m: 9\n")
    f = GasField.from_yaml(str(p))
    assert len(f.sources) == 1 and f.ppm_at([1, 2, 0], 0.0) == 900
    assert GasField.from_yaml(str(tmp_path / "missing.yaml")).ppm_at([0, 0, 0], 0) == 0.0


def test_gas_sensor_warmup_lag_and_noise():
    s = GasSensorModel(noise_ppm=0.0, lag_s=1.0, warmup_s=2.0, seed=1)
    assert s.update(0.0, 0.0) is None and s.update(1.9, 0.0) is None
    t, y = 2.0, None
    while t < 3.0 - 1e-9:                                  # step input of 1000 ppm for 1 s
        y = s.update(t, 1000.0)
        t += 0.05
    assert 550 < y < 700                                   # ~63 % after one time constant
    for _ in range(200):
        y = s.update(t, 1000.0)
        t += 0.05
    assert abs(y - 1000) < 2
    noisy = GasSensorModel(noise_ppm=6.0, lag_s=0.0, warmup_s=0.0, seed=2)
    vals = [noisy.update(i * 0.05, 500.0) for i in range(400)]
    assert 4 < np.std(vals) < 8 and abs(np.mean(vals) - 500) < 2


def test_plume_through_sensor_and_fusion_gives_danger_on_approach(cfg):
    """Drive at 0.2 m/s into a plume: CAUTION (elevated) first, then DANGER gas build-up."""
    field = GasField([GasSource(position=[5, 0, 0], peak_ppm=2500, sigma_m=1.0, ramp_up_s=5, trigger_distance_m=3.5)])
    sensor = GasSensorModel(seed=5, **{k.replace("_ppm", "_ppm"): v for k, v in cfg["gas"].items()})
    fusion = HazardFusion(cfg)
    x, t, seen = 0.5, 0.0, []
    while x < 4.5:
        a = fusion.update(mk(t, gas_ppm=sensor.update(t, field.ppm_at([x, 0, 0], t))))
        if not seen or seen[-1] != a.label:
            seen.append(a.label)
        if a.level == "DANGER":
            break
        x += 0.2 * 0.05
        t += 0.05
    assert seen == ["SAFE CLEAR", "CAUTION GAS_ELEVATED", "DANGER GAS_BUILDUP"]
    assert 2.5 < x < 4.2


# ---------------------------------------------------------------- log / caption
def test_hazard_log_rows_events_summary(tmp_path, cfg):
    log = HazardLog(str(tmp_path), csv_hz=5)
    fusion = HazardFusion(cfg)
    t = 0.0
    for i in range(41):                                    # 2 s at 20 Hz
        r = mk(t, us_left=0.8, floor_lost_left=(t >= 1.0))
        a = fusion.update(r)
        log.row(t, r, a, Command(left=0.5, right=0.5, mode="AUTO", state="CRUISE"))
        if a.sweep_request:
            log.event(t, "hazard", level=a.level, reason=a.reason, response=a.response)
        t = round(t + 0.05, 6)
    img = np.zeros((480, 640, 3), np.uint8)
    name = "s01_1_m090.jpg"
    assert caption.save_captioned(os.path.join(log.sweep_dir, name), img, 1.0, "CAUTION", "FLOOR_LOST", 35.0, 0.8, 3.0, -90)
    log.image(1.2, name, 1, -90, "CAUTION", "FLOOR_LOST", "FLOOR_LOST")
    path = log.write_summary("unit test run")
    log.close()

    rows = open(tmp_path / "hazard_log.csv").read().strip().splitlines()
    assert rows[0].split(",") == CSV_FIELDS
    assert 10 <= len(rows) - 1 <= 11                       # 5 rows per second for 2 s
    events = [json.loads(line) for line in open(tmp_path / "events.jsonl")]
    assert [e["type"] for e in events] == ["hazard", "image"]
    assert events[0]["reason"] == "FLOOR_LOST" and events[0]["t"] == 1.0
    page = open(path).read()
    assert "FLOOR_LOST" in page and f"sweeps/{name}" in page and "CAUTION" in page
    assert os.path.getsize(os.path.join(log.sweep_dir, name)) > 1000


def test_caption_text():
    l1, l2 = caption.caption_lines(12.34, "DANGER", "GAS_BUILDUP", 912.4, 0.42, 3.2, 45)
    assert "DANGER GAS_BUILDUP" in l1 and "12.3" in l1
    assert "912 ppm" in l2 and "42 cm" in l2 and "tilt 3 deg" in l2
    assert "no echo" in caption.caption_lines(0, "SAFE", "CLEAR", None, None, 0, 0)[1]
