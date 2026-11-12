import math

from rover_core.attitude import G, AttitudeFilter, ImuErrorModel

DT = 0.005


def gravity_body(pitch_deg, roll_deg):
    """Specific force in body axes (X fwd, Y left, Z up) for a static tilt.
    nose up = +pitch, left side up = +roll."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    return (G * math.sin(p), G * math.cos(p) * math.sin(r), G * math.cos(p) * math.cos(r))


def settle(f, accel, gyro=(0, 0, 0), t0=0.0, secs=4.0):
    t = t0
    for _ in range(int(secs / DT)):
        f.update(t, DT, accel, gyro)
        t += DT
    return t


def test_level_reads_zero():
    f = AttitudeFilter()
    settle(f, (0, 0, G))
    assert f.calibrated
    assert abs(f.pitch_deg) < 0.1 and abs(f.roll_deg) < 0.1 and abs(f.yaw_deg) < 0.1


def test_nose_up_is_positive_pitch_from_accel():
    f = AttitudeFilter()
    settle(f, gravity_body(25, 0))
    assert 24 < f.pitch_deg < 26
    assert abs(f.roll_deg) < 1


def test_left_side_up_is_positive_roll_from_accel():
    f = AttitudeFilter()
    settle(f, gravity_body(0, 15))
    assert 14 < f.roll_deg < 16
    assert abs(f.pitch_deg) < 1


def test_gyro_signs():
    # calibrate level, then rotate using gyro only (accel made untrustworthy -> pure gyro)
    f = AttitudeFilter()
    t = settle(f, (0, 0, G), secs=2.1)
    untrusted = (0, 0, 1.5 * G)
    rate = math.radians(20)
    for _ in range(200):          # 1 s
        # nose up = rotation about -Y; left side up = rotation about +X; left turn = +Z
        f.update(t, DT, untrusted, (rate, -rate, rate))
        t += DT
    assert 19 < f.pitch_deg < 21
    assert 19 < f.roll_deg < 21
    assert 19 < f.yaw_deg < 21


def test_gyro_bias_is_removed_and_yaw_drifts_slowly():
    f = AttitudeFilter(bias_time=2.0)
    bias = (0.01, -0.008, 0.012)
    t = settle(f, (0, 0, G), gyro=bias, secs=2.1)
    assert all(abs(b - e) < 1e-6 for b, e in zip(f.bias, bias))
    settle(f, (0, 0, G), gyro=bias, t0=t, secs=10)
    assert abs(f.yaw_deg) < 0.01


def test_complementary_filter_tracks_accel_slowly():
    f = AttitudeFilter(alpha=0.98)
    t = settle(f, (0, 0, G), secs=2.1)
    f.update(t, DT, gravity_body(10, 0), (0, 0, 0))
    assert 0.1 < f.pitch_deg < 0.3            # one step moves 2 % of the way
    settle(f, gravity_body(10, 0), t0=t, secs=2)
    assert 9.8 < f.pitch_deg < 10.2


def test_impact_spike_and_refractory():
    f = AttitudeFilter(impact_g=0.9, refractory=0.5)
    t = settle(f, (0, 0, G), secs=2.1)
    assert not f.pop_impact()
    f.update(t, DT, (-3 * G, 0, G), (0, 0, 0))            # hard knock
    assert f.pop_impact()
    assert not f.pop_impact()                              # reported once
    f.update(t + 0.1, DT, (-3 * G, 0, G), (0, 0, 0))      # inside refractory
    assert not f.pop_impact()
    f.update(t + 0.7, DT, (-3 * G, 0, G), (0, 0, 0))
    assert f.pop_impact()


def test_hard_braking_is_not_an_impact():
    f = AttitudeFilter(impact_g=0.9)
    t = settle(f, (0, 0, G), secs=2.1)
    f.update(t, DT, (-1.0 * G, 0, G), (0, 0, 0))          # 1 g horizontal: |a| = 1.41 g
    assert not f.pop_impact()


def test_no_impact_during_calibration():
    f = AttitudeFilter()
    f.update(0.0, DT, (-5 * G, 0, G), (0, 0, 0))
    assert not f.pop_impact()


def test_imu_error_model_is_seeded():
    a, b = ImuErrorModel(seed=3), ImuErrorModel(seed=3)
    assert a.bias == b.bias
    assert a.gyro((0, 0, 0)) == b.gyro((0, 0, 0))
    assert ImuErrorModel(seed=4).bias != a.bias
