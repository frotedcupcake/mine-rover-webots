"""Battery: what the rover can know (a voltage) and a simulated pack to feed it.

Real rover: 3S2P Li-ion NMC, 4000 mAh, measured through a 22k / 10k divider on the Arduino
Uno's A1 (pack V = pin V x 3.2), 5.0 V reference, 10 bits.  There is no current sensor.  The
Uno sends `batt_x10` = averaged ADC count x 10, NOT volts:

    V = (batt_x10 / 10) / 1023 * 5.0 * 3.2 * scale          (scale 1.0 until calibrated)

BatteryMonitor is the rover side: it converts the counts, keeps a moving average over
`avg_window_s` (motor load pulls the voltage down briefly) and compares that average with
three thresholds from config.  Every level LATCHES: once the average has been below a
threshold, the level never goes back up, whatever the voltage does afterwards.

    OK -> RETURN (go home) -> WARN (telemetry only) -> CRITICAL (rover stops)

BatterySim is the simulation side only: charge is internal to it, the navigator never sees it.
"""
import random
from collections import deque
from dataclasses import dataclass
from typing import Optional

OK, RETURN, WARN, CRITICAL = "OK", "RETURN", "WARN", "CRITICAL"
LEVEL_ORDER = {OK: 0, RETURN: 1, WARN: 2, CRITICAL: 3}


def check_config(b):
    """The thresholds only make sense in this order; refuse to run otherwise."""
    if not (b["return_v"] > b["warn_v"] > b["critical_v"]):
        raise ValueError("battery thresholds must satisfy return_v > warn_v > critical_v "
                         f"(got return {b['return_v']}, warn {b['warn_v']}, critical {b['critical_v']})")


def counts_to_volts(batt_x10, b):
    return (batt_x10 / 10.0) / b["adc_max"] * b["adc_ref_v"] * b["divider_ratio"] * b["scale"]


def volts_to_counts(volts, b):
    """Pack voltage -> the Uno's batt_x10 field (what the ADC and the frame would carry)."""
    x10 = volts / (b["adc_ref_v"] * b["divider_ratio"] * b["scale"]) * b["adc_max"] * 10.0
    return int(round(max(0.0, min(b["adc_max"] * 10.0, x10))))


def at_least(level, floor):
    return LEVEL_ORDER[level] >= LEVEL_ORDER[floor]


@dataclass
class BatteryStatus:
    level: str = OK                       # latched: the worst level reached so far
    volts: Optional[float] = None         # latest reading
    avg_v: Optional[float] = None         # moving average; None until the window is full
    min_avg_v: Optional[float] = None
    t_return: Optional[float] = None      # when each level was first reached
    t_warn: Optional[float] = None
    t_critical: Optional[float] = None


class BatteryMonitor:
    def __init__(self, cfg):
        self.b = cfg["battery"] if "battery" in cfg else cfg
        check_config(self.b)
        self._win = deque()
        self._t0 = None
        self.status = BatteryStatus()

    def update(self, t, batt_x10) -> BatteryStatus:
        st, b = self.status, self.b
        if batt_x10 is None:
            return st
        v = counts_to_volts(batt_x10, b)
        if self._t0 is None:
            self._t0 = t
        self._win.append((t, v))
        while self._win and t - self._win[0][0] >= b["avg_window_s"]:
            self._win.popleft()
        st.volts = v
        if t - self._t0 < b["avg_window_s"] - 1e-9:
            return st                                 # not a full window yet: nothing is judged
        avg = sum(x[1] for x in self._win) / len(self._win)
        st.avg_v = avg
        st.min_avg_v = avg if st.min_avg_v is None else min(st.min_avg_v, avg)
        eps = 1e-9
        if avg < b["return_v"] - eps and st.t_return is None:
            st.t_return = t
        if avg < b["warn_v"] - eps and st.t_warn is None:
            st.t_warn = t
        if avg < b["critical_v"] - eps and st.t_critical is None:
            st.t_critical = t
        st.level = (CRITICAL if st.t_critical is not None else WARN if st.t_warn is not None
                    else RETURN if st.t_return is not None else OK)
        return st


class BatterySim:
    """Simulated pack: open-circuit voltage from the charge left, minus a drop across the
    internal resistance that grows with the motor command.  Output is batt_x10, like the Uno."""

    def __init__(self, cfg, charge=None, seed=0):
        self.b = cfg["battery"]
        self.s = self.b["sim"]
        self.ref_cmd = cfg.get("drive", {}).get("cruise", 0.7)
        self.charge = float(self.s["start_charge"] if charge is None else charge)
        self.rng = random.Random(seed)

    def set_charge(self, charge):
        self.charge = max(0.0, min(1.0, float(charge)))

    def open_circuit_v(self):
        table = self.s["ocv_cell_v"]
        c = max(0.0, min(1.0, self.charge))
        for (c0, v0), (c1, v1) in zip(table, table[1:]):
            if c <= c1:
                return self.s["cells"] * (v0 + (v1 - v0) * (c - c0) / (c1 - c0))
        return self.s["cells"] * table[-1][1]

    def current_a(self, left, right):
        load = min(1.0, (abs(left) + abs(right)) / 2.0 / self.ref_cmd)
        return self.s["idle_a"] + (self.s["drive_a"] - self.s["idle_a"]) * load

    def terminal_v(self, left, right, noise=True):
        v = self.open_circuit_v() - self.current_a(left, right) * self.s["internal_r_ohm"]
        if noise:
            v += self.rng.gauss(0.0, self.s["noise_v"])
        return v

    def update(self, dt, left, right) -> int:
        used_mah = self.current_a(left, right) * dt / 3600.0 * 1000.0
        self.charge = max(0.0, self.charge - used_mah / self.s["capacity_mah"])
        return volts_to_counts(self.terminal_v(left, right), self.b)
