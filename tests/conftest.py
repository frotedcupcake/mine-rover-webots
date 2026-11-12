import os
import sys

import pytest

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from rover_core import config as rc_config          # noqa: E402
from rover_core.conditioning import Readings        # noqa: E402

DT = 0.05   # 20 Hz loop


@pytest.fixture
def cfg():
    return rc_config.load()


def mk(t, **kw):
    """A calm reading (clear tunnel, floor present, 20 ppm gas) with overrides."""
    base = dict(t=t, us_left=None, us_right=None, gas_ppm=20.0)
    base.update(kw)
    return Readings(**base)


def run(fusion, t0, t1, make):
    """Feed fusion at 20 Hz from t0 to t1; `make(t)` returns Readings. Returns all assessments."""
    out = []
    n = int(round((t1 - t0) / DT))
    for i in range(n + 1):
        t = round(t0 + i * DT, 6)
        out.append(fusion.update(make(t)))
    return out
