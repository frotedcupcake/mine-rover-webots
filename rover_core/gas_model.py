"""Gas plume model (world side) and gas sensor model (rover side).

World side:   ppm(pos, t) = sum over sources of peak(t) * exp(-d^2 / (2 sigma^2))
              peak(t) ramps linearly 0 -> peak_ppm over ramp_up_s, starting when the
              rover first comes within trigger_distance_m of the source.
Rover side:   first-order lag (~1 s), additive noise, and a short warm-up during which
              the sensor reports nothing.
"""
import math
import random
from dataclasses import dataclass, field
from typing import List, Optional

import yaml


@dataclass
class GasSource:
    position: List[float]
    peak_ppm: float = 2000.0
    sigma_m: float = 1.0
    ramp_up_s: float = 5.0
    trigger_distance_m: float = 3.0
    name: str = "gas"
    triggered_at: Optional[float] = field(default=None, compare=False)

    def peak_now(self, t):
        if self.triggered_at is None:
            return 0.0
        if self.ramp_up_s <= 0:
            return self.peak_ppm
        return self.peak_ppm * min(1.0, max(0.0, (t - self.triggered_at) / self.ramp_up_s))

    def ppm_at(self, pos, t):
        d = math.dist(pos[:2], self.position[:2])
        if self.triggered_at is None and d <= self.trigger_distance_m:
            self.triggered_at = t
        return self.peak_now(t) * math.exp(-d * d / (2.0 * self.sigma_m ** 2))


class GasField:
    def __init__(self, sources=None, background_ppm=0.0):
        self.sources = list(sources or [])
        self.background_ppm = background_ppm

    @classmethod
    def from_yaml(cls, path):
        try:
            with open(path) as fh:
                data = yaml.safe_load(fh) or {}
        except FileNotFoundError:
            data = {}
        srcs = [GasSource(**s) for s in (data.get("sources") or [])]
        return cls(srcs, float(data.get("background_ppm", 0.0)))

    def ppm_at(self, pos, t):
        return self.background_ppm + sum(s.ppm_at(pos, t) for s in self.sources)

    def nearest_distance(self, pos):
        if not self.sources:
            return float("inf")
        return min(math.dist(pos[:2], s.position[:2]) for s in self.sources)


class GasSensorModel:
    """Rover-side sensor: warm-up, first-order lag, noise.  Returns None while warming up."""

    def __init__(self, noise_ppm=6.0, lag_s=1.0, warmup_s=2.0, seed=0):
        self.noise_ppm = noise_ppm
        self.lag_s = lag_s
        self.warmup_s = warmup_s
        self.rng = random.Random(seed)
        self._t0 = None
        self._t_prev = None
        self._y = None

    def update(self, t, true_ppm) -> Optional[float]:
        if self._t0 is None:
            self._t0 = t
        if self._y is None:
            self._y = float(true_ppm)
        else:
            dt = max(0.0, t - self._t_prev)
            k = dt / (self.lag_s + dt) if self.lag_s > 0 else 1.0
            self._y += k * (true_ppm - self._y)
        self._t_prev = t
        if t - self._t0 < self.warmup_s:
            return None
        return max(0.0, self._y + self.rng.gauss(0, self.noise_ppm))
