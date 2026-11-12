"""Signal conditioning for the raw sensors (section 5a of the spec)."""
from collections import deque
from dataclasses import dataclass
from typing import Optional


class UltrasonicFilter:
    """Median-of-3 for an HC-SR04 style sensor.

    A raw reading at (or within `margin` of) max range means "no echo" -> None.
    With three echoes the median is returned.  With two echoes and one miss the
    *closer* echo is returned (err closer).  With one echo or none: None.
    """

    def __init__(self, max_range=4.0, margin=0.1):
        self.max_range = max_range
        self.margin = margin
        self.window = deque(maxlen=3)

    def to_echo(self, raw) -> Optional[float]:
        if raw is None or raw != raw or raw >= self.max_range - self.margin:
            return None
        return max(0.0, float(raw))

    def update(self, raw) -> Optional[float]:
        self.window.append(self.to_echo(raw))
        valid = sorted(v for v in self.window if v is not None)
        if len(valid) == 3:
            return valid[1]
        if len(valid) == 2:
            return valid[0]
        if len(valid) == 1 and len(self.window) == 1:
            return valid[0]          # very first sample: nothing else to go on
        return None


class Debounce:
    """A digital input changes state only after `n` consecutive samples agree."""

    def __init__(self, n=3, initial=False):
        self.n = n
        self.state = initial
        self.count = 0

    def update(self, value: bool) -> bool:
        value = bool(value)
        if value == self.state:
            self.count = 0
        else:
            self.count += 1
            if self.count >= self.n:
                self.state = value
                self.count = 0
        return self.state


@dataclass
class Readings:
    """One conditioned snapshot handed to fusion and the navigator every cycle."""
    t: float = 0.0
    us_left: Optional[float] = None       # m, None = no echo
    us_right: Optional[float] = None
    ir_front_left: bool = False           # debounced "obstacle within trigger distance"
    ir_front_right: bool = False
    floor_lost_left: bool = False         # debounced
    floor_lost_right: bool = False
    floor_raw_left: bool = False          # un-debounced, used by the motor guard
    floor_raw_right: bool = False
    pitch_deg: float = 0.0                # nose up = +
    roll_deg: float = 0.0                 # left side up = +
    yaw_deg: float = 0.0                  # left turn = +, continuous (not wrapped), drifts
    impact: bool = False                  # impact detected since the previous cycle
    gas_ppm: Optional[float] = None       # None while the gas sensor warms up
    age_us: float = 0.0                   # s since the last fresh sample of each kind
    age_imu: float = 0.0
    age_gas: float = 0.0

    @property
    def front_dist(self) -> float:
        """Nearest ultrasonic echo, inf if neither sensor has one."""
        vals = [v for v in (self.us_left, self.us_right) if v is not None]
        return min(vals) if vals else float("inf")

    @property
    def tilt_deg(self) -> float:
        return max(abs(self.pitch_deg), abs(self.roll_deg))
