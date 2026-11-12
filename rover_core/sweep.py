"""Camera sweep sequencer (section 5d): -90, -45, 0, 45, 90 deg, settle, capture, return to 0."""
from dataclasses import dataclass
from typing import Optional


@dataclass
class SweepStep:
    busy: bool = False
    target_deg: float = 0.0
    lamp: bool = False
    capture: Optional[int] = None      # index of the angle to photograph this cycle
    angle_deg: float = 0.0             # nominal angle of that photograph
    finished: bool = False             # True on the cycle the sweep completes


class SweepSequencer:
    def __init__(self, angles_deg=(-90, -45, 0, 45, 90), settle_s=0.35, tolerance_deg=2.0, timeout_s=25.0):
        self.angles = list(angles_deg)
        self.settle_s = settle_s
        self.tol = tolerance_deg
        self.timeout_s = timeout_s
        self.busy = False
        self.reason = ""
        self.count = 0                 # sweeps completed
        self._i = 0
        self._t0 = 0.0
        self._arrived = None

    def start(self, t, reason=""):
        if self.busy:
            return False
        self.busy, self.reason = True, reason
        self._i, self._t0, self._arrived = 0, t, None
        return True

    def update(self, t, pan_deg) -> SweepStep:
        if not self.busy:
            return SweepStep()
        if t - self._t0 > self.timeout_s:          # pan jammed: give up, lamp off
            self.busy = False
            return SweepStep(finished=True)
        if self._i >= len(self.angles):             # returning to centre
            if abs(pan_deg) <= self.tol:
                self.busy = False
                self.count += 1
                return SweepStep(finished=True)
            return SweepStep(busy=True, target_deg=0.0, lamp=True)
        target = self.angles[self._i]
        step = SweepStep(busy=True, target_deg=target, lamp=True)
        if abs(pan_deg - target) <= self.tol:
            if self._arrived is None:
                self._arrived = t
            if t - self._arrived >= self.settle_s:
                step.capture, step.angle_deg = self._i, target
                self._i += 1
                self._arrived = None
        else:
            self._arrived = None
        return step
