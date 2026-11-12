from rover_core.conditioning import Debounce, Readings, UltrasonicFilter


def test_ultrasonic_median_of_three():
    f = UltrasonicFilter(max_range=4.0)
    f.update(1.00)
    f.update(1.20)
    assert f.update(1.10) == 1.10
    # a single wild spike is rejected
    assert f.update(0.20) == 1.10
    assert f.update(1.12) == 1.10 or f.update(1.12) == 1.12


def test_ultrasonic_no_echo_is_none():
    f = UltrasonicFilter(max_range=4.0, margin=0.1)
    for _ in range(3):
        out = f.update(4.0)
    assert out is None
    assert f.to_echo(3.95) is None
    assert f.to_echo(float("nan")) is None
    assert f.to_echo(2.0) == 2.0


def test_ultrasonic_errs_closer_with_one_miss():
    f = UltrasonicFilter(max_range=4.0)
    f.update(0.60)
    f.update(4.0)          # missed echo
    assert f.update(0.50) == 0.50      # two echoes + one miss -> the closer echo


def test_ultrasonic_single_echo_among_misses_is_ignored():
    f = UltrasonicFilter(max_range=4.0)
    f.update(4.0)
    f.update(4.0)
    assert f.update(0.4) is None
    assert f.update(0.4) == 0.4        # confirmed by a second echo


def test_debounce_needs_three_in_a_row():
    d = Debounce(3)
    assert d.update(True) is False
    assert d.update(True) is False
    assert d.update(False) is False    # streak broken
    assert d.update(True) is False
    assert d.update(True) is False
    assert d.update(True) is True
    assert d.update(False) is True
    assert d.update(False) is True
    assert d.update(False) is False


def test_readings_helpers():
    r = Readings(us_left=None, us_right=0.7, pitch_deg=-12, roll_deg=5)
    assert r.front_dist == 0.7
    assert r.tilt_deg == 12
    assert Readings().front_dist == float("inf")
