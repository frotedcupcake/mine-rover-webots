"""Hazard fusion: conditioned readings -> SAFE / CAUTION / DANGER + reason, every cycle.

Proposal table (section 5b):

| evidence                                   | finding               | response                     |
|--------------------------------------------|-----------------------|------------------------------|
| gas above threshold AND still rising       | gas build-up          | DANGER: stop, alert          |
| impact spike + NEW obstacle ahead          | roof fall             | DANGER: halt, camera sweep   |
| down IR loses floor OR tilt too steep      | pit / unstable ground | CAUTION: back off, reroute   |
| ultrasonic obstacle, floor and gas normal  | passable obstruction  | SAFE: steer around           |

Additions (not in the proposal): GAS_EXTREME, GAS_ELEVATED (with hysteresis), STALE_DATA.
"""
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional

from . import CAUTION, DANGER, SAFE
from .conditioning import Readings

# reason codes, highest priority first
GAS_BUILDUP = "GAS_BUILDUP"
GAS_EXTREME = "GAS_EXTREME"       # addition
ROOF_FALL = "ROOF_FALL"
FLOOR_LOST = "FLOOR_LOST"
TILT = "TILT"
STALE_DATA = "STALE_DATA"         # addition
GAS_ELEVATED = "GAS_ELEVATED"     # addition
OBSTACLE = "OBSTACLE"
CLEAR = "CLEAR"

HAZARD_LEVEL = {
    GAS_BUILDUP: DANGER, GAS_EXTREME: DANGER, ROOF_FALL: DANGER,
    FLOOR_LOST: CAUTION, TILT: CAUTION, STALE_DATA: CAUTION, GAS_ELEVATED: CAUTION,
    OBSTACLE: SAFE, CLEAR: SAFE,
}
PRIORITY = [GAS_BUILDUP, GAS_EXTREME, ROOF_FALL, FLOOR_LOST, TILT, STALE_DATA, GAS_ELEVATED, OBSTACLE, CLEAR]
RESPONSE = {
    GAS_BUILDUP: "gas build-up: stop, alert",
    GAS_EXTREME: "gas extreme: stop, alert",
    ROOF_FALL: "roof fall: halt, camera sweep",
    FLOOR_LOST: "pit / no floor: back off, reroute",
    TILT: "unstable ground (tilt): back off, reroute",
    STALE_DATA: "sensor data stale: stop",
    GAS_ELEVATED: "gas elevated: proceed with caution",
    OBSTACLE: "passable obstruction: steer around",
    CLEAR: "clear",
}
NO_SWEEP = {STALE_DATA, OBSTACLE, CLEAR}


@dataclass
class Assessment:
    t: float = 0.0
    level: str = SAFE
    reason: str = CLEAR
    response: str = RESPONSE[CLEAR]
    hazards: List[str] = field(default_factory=list)   # active (incl. held) findings, by priority
    raw: List[str] = field(default_factory=list)       # findings true this very cycle
    sweep_request: Optional[str] = None                # hazard that wants a camera sweep now
    stop: bool = False                                 # motors must stop (stale data)
    gas_ppm: Optional[float] = None
    gas_slope: float = 0.0                             # ppm/s
    front_dist: float = float("inf")
    tilt_deg: float = 0.0
    readings: Optional[Readings] = None

    @property
    def label(self):
        return f"{self.level} {self.reason}"


def least_squares_slope(samples):
    """Slope of y over t for [(t, y), ...]; 0 if it cannot be computed."""
    n = len(samples)
    if n < 2:
        return 0.0
    mt = sum(s[0] for s in samples) / n
    my = sum(s[1] for s in samples) / n
    den = sum((s[0] - mt) ** 2 for s in samples)
    if den <= 1e-12:
        return 0.0
    return sum((s[0] - mt) * (s[1] - my) for s in samples) / den


class HazardFusion:
    def __init__(self, cfg):
        self.c = cfg["fusion"] if "fusion" in cfg else cfg
        self._gas_hist = deque()
        self._dist_hist = deque()
        self._last_true = {}            # finding -> last time its raw condition held
        self._active_prev = set()
        self._last_sweep = {}           # finding -> time of the last sweep it requested
        self._gas_elevated = False
        self._impact = None             # (t_impact, baseline distance before the impact)
        self._roof_until = -1e9
        self._t0 = None

    # ------------------------------------------------------------------
    def update(self, r: Readings) -> Assessment:
        c, t = self.c, r.t
        if self._t0 is None:
            self._t0 = t
        front = r.front_dist

        # --- gas slope over the look-back window -------------------------------
        if r.gas_ppm is not None:
            self._gas_hist.append((t, r.gas_ppm))
        while self._gas_hist and t - self._gas_hist[0][0] > c["gas_slope_window_s"]:
            self._gas_hist.popleft()
        slope = 0.0
        if len(self._gas_hist) >= 3 and self._gas_hist[-1][0] - self._gas_hist[0][0] >= 0.5 * c["gas_slope_window_s"]:
            slope = least_squares_slope(self._gas_hist)

        # --- roof fall: impact, then a NEW obstacle ------------------------------
        if r.impact:
            pre = [d for (tt, d) in self._dist_hist if t - tt <= c["roof_pre_window_s"]]
            self._impact = (t, min(pre) if pre else float("inf"))
        self._dist_hist.append((t, front))
        while self._dist_hist and t - self._dist_hist[0][0] > 2.0:
            self._dist_hist.popleft()
        if self._impact is not None:
            ti, baseline = self._impact
            dt = t - ti
            if dt > c["roof_window_s"]:
                self._impact = None
            else:
                needed = c["roof_delta_m"] + c.get("roof_approach_mps", 0.0) * dt
                if front < c["roof_obstacle_m"] and baseline - front >= needed:
                    self._roof_until = t + c["roof_latch_s"]
                    self._impact = None

        # --- raw findings ----------------------------------------------------------
        gas = r.gas_ppm
        raw = set()
        if gas is not None:
            if gas >= c["gas_danger_ppm"] and slope >= c["gas_slope_ppm_s"]:
                raw.add(GAS_BUILDUP)
            if gas >= c["gas_extreme_ppm"]:
                raw.add(GAS_EXTREME)
            if self._gas_elevated:
                if gas < c["gas_caution_ppm"] * c["gas_caution_off_ratio"]:
                    self._gas_elevated = False
            elif gas >= c["gas_caution_ppm"]:
                self._gas_elevated = True
            if self._gas_elevated:
                raw.add(GAS_ELEVATED)
        if t < self._roof_until:
            raw.add(ROOF_FALL)
        if r.floor_lost_left or r.floor_lost_right:
            raw.add(FLOOR_LOST)
        if r.tilt_deg > c["tilt_deg"]:
            raw.add(TILT)
        if t - self._t0 >= c.get("startup_grace_s", 0.0):
            if max(r.age_us, r.age_imu, r.age_gas) > c["stale_s"]:
                raw.add(STALE_DATA)
        if front < c["obstacle_m"] or r.ir_front_left or r.ir_front_right:
            raw.add(OBSTACLE)

        # --- hold: CAUTION/DANGER findings persist hold_s after their cause clears ---
        active = set(raw)
        for h in raw:
            self._last_true[h] = t
        for h, tl in self._last_true.items():
            # (GAS_ELEVATED has its own hysteresis and ROOF_FALL its own 15 s latch instead)
            if HAZARD_LEVEL[h] != SAFE and h not in (GAS_ELEVATED, ROOF_FALL) and t - tl < c["hold_s"]:
                active.add(h)

        ordered = [h for h in PRIORITY if h in active]
        reason = ordered[0] if ordered else CLEAR
        level = HAZARD_LEVEL[reason]

        # --- camera sweep on *entering* a CAUTION/DANGER finding, with cooldown ------
        sweep = None
        for h in ordered:
            if h in NO_SWEEP or h in self._active_prev:
                continue
            if t - self._last_sweep.get(h, -1e9) >= c["sweep_cooldown_s"]:
                if sweep is None:
                    sweep = h
                self._last_sweep[h] = t
        self._active_prev = active

        return Assessment(
            t=t, level=level, reason=reason, response=RESPONSE[reason], hazards=ordered,
            raw=[h for h in PRIORITY if h in raw], sweep_request=sweep, stop=STALE_DATA in active,
            gas_ppm=gas, gas_slope=slope, front_dist=front, tilt_deg=r.tilt_deg, readings=r)
