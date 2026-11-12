"""Navigator: modes STOP / MANUAL / AUTO and the AUTO state machine (section 5c).

AUTO states: CALIBRATING, CRUISE, AVOID, BACKOFF, SWEEP, HOLD, STUCK.

Outputs are left/right motor commands as fractions of top speed.  Guards applied in
every mode, after the state logic:
  * never drive a side forward while a down IR has lost the floor (raw, same cycle)
  * stop on stale data
  * commands ramp up but drop to zero instantly
The 0.4 s dead-man lives in MotorGuard, which the controller consults every 5 ms.
"""
from dataclasses import dataclass
from typing import Optional

from . import DANGER
from .conditioning import Readings
from .fusion import FLOOR_LOST, TILT, Assessment

STOP, MANUAL, AUTO = "STOP", "MANUAL", "AUTO"
CALIBRATING, CRUISE, AVOID, BACKOFF, SWEEP, HOLD, STUCK, IDLE = (
    "CALIBRATING", "CRUISE", "AVOID", "BACKOFF", "SWEEP", "HOLD", "STUCK", "IDLE")
INF = float("inf")


@dataclass
class Command:
    left: float = 0.0
    right: float = 0.0
    mode: str = STOP
    state: str = IDLE
    note: str = ""
    start_sweep: Optional[str] = None     # reason string when a sweep should start now
    lamp_toggle: bool = False
    drop_rock: bool = False               # ask the supervisor for a roof fall (demo key)
    refused: Optional[str] = None         # why a manual forward command was refused


class MotorGuard:
    """Dead-man: the last command is only valid for `deadman_s` seconds."""

    def __init__(self, deadman_s=0.4):
        self.deadman_s = deadman_s
        self._t = -1e9
        self._cmd = (0.0, 0.0)

    def set(self, t, left, right):
        self._t, self._cmd = t, (left, right)

    def get(self, t, floor_lost=False):
        if t - self._t > self.deadman_s:
            return (0.0, 0.0)
        left, right = self._cmd
        if floor_lost:                    # cut forward drive the very step the floor vanishes
            left, right = min(left, 0.0), min(right, 0.0)
        return (left, right)


def snap(x, min_cmd):
    """Keep commands out of the motor dead zone: tiny -> 0, small -> min_cmd."""
    ax = abs(x)
    if ax < 1e-6:
        return 0.0
    if ax >= min_cmd:
        return max(-1.0, min(1.0, x))
    if ax < 0.5 * min_cmd:
        return 0.0
    return min_cmd if x > 0 else -min_cmd


class Navigator:
    def __init__(self, cfg):
        self.cfg = cfg
        self.n = cfg["navigator"]
        self.d = cfg["drive"]
        self.mode = self.n.get("start_mode", AUTO)
        self.state = CALIBRATING if self.mode == AUTO else IDLE
        self.note = ""
        self._prev_keys = set()
        self._out = (0.0, 0.0)            # last shaped output (for the ramp)
        self._t_prev = None
        # cruise
        self._yaw_ref = None
        self._bias_side = None            # 'left' / 'right': keep away from that side
        self._bias_until = -1e9
        # avoid
        self._av = None
        # backoff
        self._bo = None
        self._pit_times = []
        # sweep / hold
        self._pending_sweep = None
        self._sweep_return = CRUISE
        self._sweep_seen_busy = False
        self._sweep_t0 = 0.0
        self._hold_reason = ""
        # manual
        self._manual_cmd = (0.0, 0.0)
        self._manual_t = -1e9

    # ==================================================================
    def step(self, t, r: Readings, a: Assessment, keys=(), sweep_busy=False, ready=True) -> Command:
        keys = set(keys)
        pressed = keys - self._prev_keys          # rising edges
        self._prev_keys = keys
        cmd = Command()

        # ---- keyboard: mode changes and one-shot actions ----
        if "SPACE" in pressed:
            self._set_mode(STOP, "e-stop")
        if "X" in pressed:
            self._set_mode(STOP, "stop mode")
        if "M" in pressed:
            self._set_mode(MANUAL, "manual mode")
        if "SHIFT_A" in pressed or ("A" in pressed and self.mode != MANUAL):
            self._set_mode(AUTO, "auto mode")
        if "SHIFT_D" in pressed or ("D" in pressed and self.mode != MANUAL):
            cmd.drop_rock = True
        if "L" in pressed:
            cmd.lamp_toggle = True
        if "C" in pressed and not sweep_busy:
            self._pending_sweep = "MANUAL"
        if "R" in pressed and self.mode == AUTO and self.state in (HOLD, STUCK):
            if a.level == DANGER:
                self.note = f"resume refused: still {a.label}"
            else:
                self._enter_cruise(r, "resumed")

        # ---- mode logic ----
        if self.mode == AUTO:
            left, right = self._auto(t, r, a, sweep_busy, ready, cmd)
        elif self.mode == MANUAL:
            left, right = self._manual(t, r, keys, cmd)
            if self._pending_sweep and not sweep_busy:
                cmd.start_sweep, self._pending_sweep = self._pending_sweep, None
        else:
            left, right = 0.0, 0.0
            if self._pending_sweep and not sweep_busy:
                cmd.start_sweep, self._pending_sweep = self._pending_sweep, None

        # ---- guards (all modes) ----
        if r.floor_raw_left or r.floor_raw_right or r.floor_lost_left or r.floor_lost_right:
            left, right = min(left, 0.0), min(right, 0.0)
        if a.stop:
            left, right = 0.0, 0.0
        if sweep_busy and self.mode != MANUAL:
            left, right = 0.0, 0.0            # hold still while the camera sweeps
        left, right = self._shape(t, left, right)

        cmd.left, cmd.right = left, right
        cmd.mode, cmd.state, cmd.note = self.mode, self.state, self.note
        return cmd

    # ==================================================================
    def _set_mode(self, mode, note):
        self.mode = mode
        self.note = note
        self._manual_cmd = (0.0, 0.0)
        if mode == AUTO:
            self.state = CRUISE
            self._yaw_ref = None
        else:
            self.state = IDLE

    def _enter_cruise(self, r, note=""):
        self.state = CRUISE
        self._yaw_ref = r.yaw_deg
        self.note = note

    def _shape(self, t, left, right):
        """Ramp up, stop instantly; keep commands out of the dead zone."""
        mc = self.d["min_cmd"]
        dt = 0.05 if self._t_prev is None else max(1e-3, t - self._t_prev)
        self._t_prev = t
        out = []
        for target, prev in zip((snap(left, mc), snap(right, mc)), self._out):
            if target == 0.0 or target * prev < 0:
                val = 0.0 if target == 0.0 else (mc if target > 0 else -mc)
            elif abs(target) <= abs(prev):
                val = target
            else:
                step = self.d["ramp_per_s"] * dt
                mag = min(abs(target), max(abs(prev), mc * 0.8) + step)
                val = mag if target > 0 else -mag
            out.append(val)
        self._out = tuple(out)
        return self._out

    # ==================================================================
    # MANUAL
    def _manual(self, t, r, keys, cmd):
        d = self.d
        v = (d["cruise"] if "W" in keys else 0.0) - (d["reverse"] if "S" in keys else 0.0)
        w = (1.0 if "A" in keys else 0.0) - (1.0 if "D" in keys else 0.0)
        if v == 0.0 and w == 0.0:
            left = right = None
        elif v == 0.0:
            left, right = -w * d["spin"], w * d["spin"]
        else:
            left, right = v - w * 0.3 * abs(v) / d["cruise"], v + w * 0.3 * abs(v) / d["cruise"]
        if left is not None:
            self._manual_cmd, self._manual_t = (left, right), t
        elif t - self._manual_t > d["deadman_s"]:
            self._manual_cmd = (0.0, 0.0)
        left, right = self._manual_cmd
        if t - self._manual_t > d["deadman_s"]:
            left, right = 0.0, 0.0
        # refuse forward over a drop or into something close
        if left > 0 and right > 0:
            if r.floor_lost_left or r.floor_lost_right or r.floor_raw_left or r.floor_raw_right:
                cmd.refused = "drop ahead"
            elif r.front_dist < self.n["manual_block_m"] or r.ir_front_left or r.ir_front_right:
                cmd.refused = "obstacle within 15 cm"
            if cmd.refused:
                left, right = 0.0, 0.0
                self.note = f"forward refused: {cmd.refused}"
        return left, right

    # ==================================================================
    # AUTO
    def _auto(self, t, r, a, sweep_busy, ready, cmd):
        n = self.n
        if not ready:
            self.state = CALIBRATING
            self.note = "calibrating gyro / warming up"
            return 0.0, 0.0
        if self.state in (CALIBRATING, IDLE):
            self._enter_cruise(r, "go")

        if a.sweep_request:
            self._pending_sweep = a.sweep_request

        # --- DANGER: hold position (sweep first if one is due) ---
        if a.level == DANGER and self.state not in (HOLD, SWEEP):
            self.state = HOLD
            self._hold_reason = a.reason
            self.note = f"holding: {a.reason} (press R to resume)"
        if a.level == DANGER and self.state == SWEEP:
            self._sweep_return = HOLD
            self._hold_reason = a.reason

        # --- SWEEP in progress ---
        if self.state == SWEEP:
            if sweep_busy:
                self._sweep_seen_busy = True
            done = (self._sweep_seen_busy and not sweep_busy) or (t - self._sweep_t0 > self.cfg["sweep"]["timeout_s"] + 5)
            if not done:
                return 0.0, 0.0
            self.state = self._sweep_return
            if self.state == CRUISE:
                self._yaw_ref = r.yaw_deg
            if self.state == HOLD:
                self.note = f"holding: {self._hold_reason} (press R to resume)"

        # --- pit / tilt: back off (unless already doing so) ---
        if self.state in (CRUISE, AVOID):
            cause = None
            if r.floor_lost_left or r.floor_lost_right:
                cause = FLOOR_LOST
            elif TILT in a.raw:
                cause = TILT
            if cause:
                self._start_backoff(t, r, cause)

        # --- start a pending sweep (states that can pause for it) ---
        if self._pending_sweep and not sweep_busy and self.state in (CRUISE, AVOID, HOLD, BACKOFF, STUCK):
            if self.state != BACKOFF or self._bo["phase"] == "WAIT":
                cmd.start_sweep, self._pending_sweep = self._pending_sweep, None
                self._sweep_return = CRUISE if self.state == AVOID else self.state
                self._sweep_seen_busy = False
                self._sweep_t0 = t
                self.state = SWEEP
                return 0.0, 0.0

        if self.state == HOLD or self.state == STUCK:
            return 0.0, 0.0
        if self.state == BACKOFF:
            return self._backoff(t, r, a)
        if self.state == AVOID:
            return self._avoid(t, r)
        return self._cruise(t, r)

    # ------------------------------------------------------------------
    def _dists(self, t, r):
        dl = r.us_left if r.us_left is not None else INF
        dr = r.us_right if r.us_right is not None else INF
        if t < self._bias_until:
            if self._bias_side == "left":
                dl = min(dl, self.n["bias_dist_m"])
            elif self._bias_side == "right":
                dr = min(dr, self.n["bias_dist_m"])
        return dl, dr

    def _cruise(self, t, r):
        n, d = self.n, self.d
        dl, dr = self._dists(t, r)
        real_near = r.front_dist
        if real_near < n["avoid_dist_m"] or r.ir_front_left or r.ir_front_right:
            self._start_avoid(t, r)
            return self._avoid(t, r)
        near = min(dl, dr)
        v = d["cruise"] if near >= n["slow_dist_m"] else d["slow"]
        span = n["slow_dist_m"] - n["avoid_dist_m"]
        cl = min(1.0, max(0.0, (n["slow_dist_m"] - dl) / span))
        cr = min(1.0, max(0.0, (n["slow_dist_m"] - dr) / span))
        if cl == 0.0 and cr == 0.0:
            # nothing near: hold the gyro heading so the rover tracks straight
            if self._yaw_ref is None:
                self._yaw_ref = r.yaw_deg
            w = d["heading_hold_gain"] * (self._yaw_ref - r.yaw_deg)
            w = max(-0.15, min(0.15, w))
            self.note = "cruise"
        else:
            w = d["steer_gain"] * (cr - cl)          # + = turn left, away from the right
            self._yaw_ref = r.yaw_deg
            self.note = "steer " + ("left" if w > 0 else "right" if w < 0 else "straight")
        return v - w, v + w

    # ------------------------------------------------------------------
    def _start_avoid(self, t, r):
        dl, dr = self._dists(t, r)
        if abs(dl - dr) > 0.05 and min(dl, dr) < self.n["slow_dist_m"]:
            direction = 1 if dl > dr else -1          # +1 = spin left
        elif r.ir_front_left != r.ir_front_right:
            direction = -1 if r.ir_front_left else 1
        elif t < self._bias_until and self._bias_side:
            direction = -1 if self._bias_side == "left" else 1
        else:
            direction = 1
        self._av = dict(t0=t, yaw0=r.yaw_deg, dir=direction, reversed=False, clear_since=None)
        self.state = AVOID
        self.note = "avoid: spin " + ("left" if direction > 0 else "right")

    def _avoid(self, t, r):
        n, av = self.n, self._av
        dl = r.us_left if r.us_left is not None else INF
        dr = r.us_right if r.us_right is not None else INF
        clear = (min(dl, dr) >= n["clear_dist_m"] and max(dl, dr) >= n["open_dist_m"]
                 and not r.ir_front_left and not r.ir_front_right)
        if clear:
            if av["clear_since"] is None:
                av["clear_since"] = t
            if t - av["clear_since"] >= n["clear_time_s"]:
                self._enter_cruise(r, "avoid done")
                return self._cruise(t, r)
        else:
            av["clear_since"] = None
        if t - av["t0"] > n["avoid_timeout_s"]:
            self.state = STUCK
            self.note = "STUCK: no clear direction found"
            return 0.0, 0.0
        turned = (r.yaw_deg - av["yaw0"]) * av["dir"]
        if not av["reversed"] and turned > n["avoid_reverse_deg"]:
            av["dir"], av["reversed"] = -av["dir"], True
            self.note = "avoid: other way"
        s = self.d["spin"] * av["dir"]
        return -s, s

    # ------------------------------------------------------------------
    def _start_backoff(self, t, r, cause):
        n = self.n
        if cause == FLOOR_LOST:
            self._pit_times = [x for x in self._pit_times if t - x <= n["pit_window_s"]] + [t]
            if len(self._pit_times) >= n["pit_count"]:
                self.state = STUCK
                self.note = "STUCK: route blocked (pits)"
                return
            left, right = r.floor_lost_left, r.floor_lost_right
            if left and right:
                side = "both"
            else:
                side = "left" if left else "right"
        else:
            if abs(r.roll_deg) > abs(r.pitch_deg):
                side = "left" if r.roll_deg > 0 else "right"   # high side = slope on that side
            else:
                side = "both"
        self._bo = dict(phase="WAIT", cause=cause, side=side, t0=t, t_phase=t, yaw0=r.yaw_deg)
        self.state = BACKOFF
        self.note = f"backoff: {cause} ({side})"

    def _backoff(self, t, r, a):
        n, d, bo = self.n, self.d, self._bo
        if bo["phase"] == "WAIT":
            # stopped at the edge; the sweep (if due) was started by _auto() before we get here
            bo["phase"], bo["t_phase"] = "REVERSE", t
        if bo["phase"] == "REVERSE":
            el = t - bo["t_phase"]
            if bo["cause"] == FLOOR_LOST:
                more = el < n["backoff_reverse_s"]
            else:
                more = el < n["backoff_reverse_s"] or (
                    r.tilt_deg > n["tilt_clear_deg"] and el < n["tilt_reverse_max_s"])
            if more:
                return -d["reverse"], -d["reverse"]
            bo["phase"], bo["t_phase"], bo["yaw0"] = "TURN", t, r.yaw_deg
            if bo["side"] == "both":
                dl = r.us_left if r.us_left is not None else INF
                dr = r.us_right if r.us_right is not None else INF
                bo["dir"] = 1 if dl >= dr else -1
                bo["angle"] = n["backoff_turn_both_deg"]
            else:
                bo["dir"] = -1 if bo["side"] == "left" else 1     # turn away from that edge
                bo["angle"] = n["backoff_turn_deg"]
        if bo["phase"] == "TURN":
            turned = (r.yaw_deg - bo["yaw0"]) * bo["dir"]
            if turned >= bo["angle"] or t - bo["t_phase"] > n["backoff_turn_timeout_s"]:
                if bo["side"] in ("left", "right"):
                    self._bias_side, self._bias_until = bo["side"], t + n["bias_s"]
                self._enter_cruise(r, "backoff done")
                return self._cruise(t, r)
            s = d["spin"] * bo["dir"]
            return -s, s
        return 0.0, 0.0
