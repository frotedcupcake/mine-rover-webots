"""Mine rover controller: Webots glue around the pure-Python autonomy in rover_core/.

Per 5 ms simulation step : IMU -> attitude filter, floor-loss motor cut, dead-man.
Per 50 ms (20 Hz)        : condition sensors -> fuse -> navigate -> command motors,
                           camera sweep, logging, telemetry to the supervisor.

The inertial_unit is read for ground-truth error statistics only; the autonomy never
uses it.
"""
import json
import math
import os
import sys
import time

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from controller import Keyboard, Robot  # noqa: E402

from rover_core import caption, config  # noqa: E402
from rover_core.attitude import AttitudeFilter, ImuErrorModel  # noqa: E402
from rover_core.conditioning import Debounce, Readings, UltrasonicFilter  # noqa: E402
from rover_core.fusion import HAZARD_LEVEL, HazardFusion  # noqa: E402
from rover_core.gas_model import GasSensorModel  # noqa: E402
from rover_core.hazard_log import HazardLog  # noqa: E402
from rover_core.navigator import MANUAL, MotorGuard, Navigator  # noqa: E402
from rover_core.sweep import SweepSequencer  # noqa: E402

REMOTE_KEYS = {"resume": "R", "sweep": "C", "auto": "SHIFT_A", "stop": "X", "manual": "M", "estop": "SPACE",
               "lamp": "L"}


class Rover:
    def __init__(self):
        self.robot = Robot()
        r = self.robot
        self.ts = int(r.getBasicTimeStep())
        overrides = json.loads(os.environ.get("MINE_CFG", "{}") or "{}")
        self.cfg = cfg = config.load(overrides=overrides)
        self.seed = int(os.environ.get("MINE_SEED", "0") or 0)
        self.loop_div = max(1, round(1000.0 / cfg["loop_hz"] / self.ts))

        # ---- devices ----
        self.us = {s: r.getDevice(f"us_{s}") for s in ("left", "right")}
        self.irf = {s: r.getDevice(f"ir_front_{s}") for s in ("left", "right")}
        self.ird = {s: r.getDevice(f"ir_down_{s}") for s in ("left", "right")}
        us_ms = max(self.ts, int(round(cfg["ultrasonic"]["sample_ms"] / self.ts)) * self.ts)
        for d in self.us.values():
            d.enable(us_ms)
        for d in self.irf.values():
            d.enable(2 * self.ts)
        for d in self.ird.values():
            d.enable(self.ts)
        self.us_period = us_ms / 1000.0
        self.accel = r.getDevice("accelerometer")
        self.gyro = r.getDevice("gyro")
        self.truth = r.getDevice("inertial_unit")
        self.accel.enable(self.ts)
        self.gyro.enable(self.ts)
        self.truth.enable(self.ts * self.loop_div)
        self.wheels = {n: r.getDevice(f"wheel_{n}") for n in ("fl", "fr", "rl", "rr")}
        for w in self.wheels.values():
            w.setPosition(float("inf"))
            w.setVelocity(0.0)
        self.max_w = self.wheels["fl"].getMaxVelocity()
        self.pan = r.getDevice("pan_motor")
        self.pan_sensor = r.getDevice("pan_sensor")
        self.pan_sensor.enable(self.ts * 2)
        self.camera = r.getDevice("camera")
        self.lamp = r.getDevice("headlamp")
        self.receiver = r.getDevice("receiver")
        self.receiver.enable(self.ts * 2)
        self.emitter = r.getDevice("emitter")
        self.keyboard = r.getKeyboard()
        self.keyboard.enable(self.ts * self.loop_div)

        # ---- autonomy ----
        imu, ir = cfg["imu"], cfg["ir"]
        self.att = AttitudeFilter(imu["alpha"], imu["gyro_bias_time_s"], imu["impact_g"],
                                  imu["impact_refractory_s"], imu["accel_trust_g"])
        self.imu_err = ImuErrorModel(imu["accel_noise_mps2"], imu["gyro_noise_rps"], imu["gyro_bias_rps"], self.seed)
        self.us_f = {s: UltrasonicFilter(cfg["ultrasonic"]["max_range_m"], cfg["ultrasonic"]["no_echo_margin_m"])
                     for s in self.us}
        self.us_val = {s: None for s in self.us}
        self.irf_db = {s: Debounce(ir["debounce"]) for s in self.irf}
        self.ird_db = {s: Debounce(ir["debounce"]) for s in self.ird}
        g = cfg["gas"]
        self.gas_sensor = GasSensorModel(g["noise_ppm"], g["lag_s"], g["warmup_s"], self.seed + 1000)
        self.fusion = HazardFusion(cfg)
        self.nav = Navigator(cfg)
        self.guard = MotorGuard(cfg["drive"]["deadman_s"])
        sw = cfg["sweep"]
        self.seq = SweepSequencer(sw["angles_deg"], sw["settle_s"], sw["tolerance_deg"], sw["timeout_s"])
        mm = cfg["drive"].get("motor_mismatch", 0.0)
        import random
        rng = random.Random(self.seed + 7)
        self.gain = {"l": 1.0 + rng.uniform(-mm, mm), "r": 1.0 + rng.uniform(-mm, mm)}

        # ---- state ----
        self.gas_true = None
        self.t_gas = -1e9
        self.t_us = -1e9
        self.t_imu = 0.0
        self.gas_reading = None
        self.remote = []
        self.requests = []
        self.run_name = os.environ.get("MINE_RUN") or None
        self.log = None
        self.lamp_manual = bool(cfg.get("lamp_default_on", True))
        self.lamp_state = None
        self.sweep_id = 0
        self.sweep_why = ""
        self.cam_on = False
        self.seen_labels = []
        self.seen_nav = []
        self.prev_hazards = set()
        self.prev_nav = None
        self.prev_label = None
        self.next_status = 0.0
        self.next_summary = 5.0
        self.finished = False
        self.att_err = {"pitch": 0.0, "roll": 0.0, "yaw_drift": 0.0}
        self.yaw_truth0 = None
        self.yaw_truth_unwrapped = 0.0
        self.yaw_truth_prev = None
        self.impacts = 0

    # ------------------------------------------------------------------
    def ensure_log(self, t):
        if self.log is not None:
            return True
        if self.finished:
            return False
        if self.run_name is None and t < 0.5:
            return False                       # give the supervisor a moment to name the run
        if self.run_name is None:
            self.run_name = time.strftime("manual_%Y%m%d_%H%M%S")
        run_dir = os.path.join(ROOT, "reports", self.run_name)
        self.log = HazardLog(run_dir, self.cfg["log"]["csv_hz"])
        self.log.event(t, "start", seed=self.seed, mode=self.nav.mode, python=sys.version.split()[0])
        return True

    def drive(self, left, right):
        dz = self.cfg["drive"]["dead_zone"]
        left = 0.0 if abs(left) < dz else max(-1.0, min(1.0, left * self.gain["l"]))
        right = 0.0 if abs(right) < dz else max(-1.0, min(1.0, right * self.gain["r"]))
        self.wheels["fl"].setVelocity(left * self.max_w)
        self.wheels["rl"].setVelocity(left * self.max_w)
        self.wheels["fr"].setVelocity(right * self.max_w)
        self.wheels["rr"].setVelocity(right * self.max_w)

    def read_keys(self):
        keys = set()
        k = self.keyboard.getKey()
        while k != -1:
            shift = bool(k & Keyboard.SHIFT)
            code = k & 0xFFFF
            name = None
            if code == ord(" "):
                name = "SPACE"
            elif 65 <= code <= 90:
                name = chr(code)
            elif self.nav.mode == MANUAL and code in (Keyboard.UP, Keyboard.DOWN, Keyboard.LEFT, Keyboard.RIGHT):
                name = {Keyboard.UP: "W", Keyboard.DOWN: "S", Keyboard.LEFT: "A", Keyboard.RIGHT: "D"}[code]
            if name == "V":
                if "V" not in getattr(self, "_held", set()):
                    self.requests.append("toggle_cam")
            if name:
                keys.add(name)
                if shift and name in ("A", "D"):
                    keys.add("SHIFT_" + name)
            k = self.keyboard.getKey()
        self._held = set(keys)
        for c in self.remote:
            if c in REMOTE_KEYS:
                keys.add(REMOTE_KEYS[c])
        return keys

    def read_radio(self, t):
        self.remote = []
        while self.receiver.getQueueLength() > 0:
            try:
                msg = json.loads(self.receiver.getString())
            except Exception:
                msg = {}
            self.receiver.nextPacket()
            if msg.get("run") and self.run_name is None:
                self.run_name = msg["run"]
            if msg.get("gas") is not None:
                self.gas_true, self.t_gas = float(msg["gas"]), t
            self.remote.extend(msg.get("cmd") or [])

    # ------------------------------------------------------------------
    def cycle(self, t):
        cfg = self.cfg
        self.read_radio(t)
        # ultrasonic: feed the median filter once per fresh 60 ms sample
        if t - self.t_us >= self.us_period - 1e-6:
            for s, dev in self.us.items():
                self.us_val[s] = self.us_f[s].update(dev.getValue())
            self.t_us = t
        irf = {s: self.irf_db[s].update(d.getValue() < cfg["ir"]["front_trigger_m"]) for s, d in self.irf.items()}
        raw_floor = {s: d.getValue() > cfg["ir"]["floor_lost_m"] for s, d in self.ird.items()}
        floor = {s: self.ird_db[s].update(raw_floor[s]) for s in raw_floor}
        if self.gas_true is not None and t - self.t_gas < cfg["fusion"]["stale_s"]:
            self.gas_reading = self.gas_sensor.update(t, self.gas_true)
        impact = self.att.pop_impact()
        r = Readings(
            t=t, us_left=self.us_val["left"], us_right=self.us_val["right"],
            ir_front_left=irf["left"], ir_front_right=irf["right"],
            floor_lost_left=floor["left"], floor_lost_right=floor["right"],
            floor_raw_left=raw_floor["left"], floor_raw_right=raw_floor["right"],
            pitch_deg=self.att.pitch_deg, roll_deg=self.att.roll_deg, yaw_deg=self.att.yaw_deg,
            impact=impact, gas_ppm=self.gas_reading,
            age_us=t - self.t_us, age_imu=t - self.t_imu, age_gas=t - self.t_gas)
        a = self.fusion.update(r)
        keys = self.read_keys()
        ready = self.att.calibrated and r.gas_ppm is not None
        cmd = self.nav.step(t, r, a, keys=keys, sweep_busy=self.seq.busy, ready=ready)
        self.guard.set(t, cmd.left, cmd.right)

        if cmd.lamp_toggle:
            self.lamp_manual = not self.lamp_manual
        if cmd.drop_rock:
            self.requests.append("drop_rock")
        if "finish" in self.remote:
            self.finished = True

        have_log = self.ensure_log(t)
        self.sweep(t, r, a, cmd)
        if have_log:
            self.record(t, r, a, cmd, impact)
        self.truth_stats()
        self.telemetry(t, a, cmd)
        return r, a, cmd

    # ------------------------------------------------------------------
    def sweep(self, t, r, a, cmd):
        if cmd.start_sweep and self.seq.start(t, cmd.start_sweep):
            self.sweep_id += 1
            self.sweep_why = cmd.start_sweep
            self.camera.enable(self.ts * self.loop_div)
            self.cam_on = True
            if self.log:
                self.log.event(t, "sweep_start", sweep=self.sweep_id, why=self.sweep_why, level=a.level, reason=a.reason)
        pan_deg = math.degrees(self.pan_sensor.getValue())
        st = self.seq.update(t, pan_deg)
        if st.busy:
            self.pan.setPosition(math.radians(st.target_deg))
        if st.capture is not None and self.log:
            name = f"sweep{self.sweep_id:02d}_{st.capture + 1}_{st.angle_deg:+04.0f}deg_{self.sweep_why}.jpg"
            ok = False
            try:
                buf = self.camera.getImage()
                if buf:
                    img = caption.bgra_bytes_to_bgr(buf, self.camera.getWidth(), self.camera.getHeight())
                    dist = None if r.front_dist == float("inf") else r.front_dist
                    ok = caption.save_captioned(os.path.join(self.log.sweep_dir, name), img, t, a.level, a.reason,
                                                r.gas_ppm, dist, r.tilt_deg, pan_deg, self.sweep_why,
                                                self.cfg["sweep"]["jpeg_quality"])
            except Exception as e:      # rendering disabled, etc.
                print(f"[rover] image capture failed: {e}", flush=True)
            if ok:
                self.log.image(t, name, self.sweep_id, st.angle_deg, a.level, a.reason, self.sweep_why)
        if st.finished:
            self.pan.setPosition(0.0)
            if self.cam_on:
                self.camera.disable()
                self.cam_on = False
            if self.log:
                self.log.event(t, "sweep_end", sweep=self.sweep_id, images=len(self.log.images))
        lamp = self.lamp_manual or st.busy
        if lamp != self.lamp_state:
            self.lamp.set(1 if lamp else 0)
            self.lamp_state = lamp

    def record(self, t, r, a, cmd, impact):
        log = self.log
        log.row(t, r, a, cmd)
        if impact:
            self.impacts += 1
            log.event(t, "impact", g=round(self.att.last_impact_g, 2))
        label = a.label
        if label != self.prev_label:
            log.event(t, "level", level=a.level, reason=a.reason)
            if label not in self.seen_labels:
                self.seen_labels.append(label)
            self.prev_label = label
        hazards = {h for h in a.hazards if HAZARD_LEVEL[h] != "SAFE"}
        for h in a.hazards:
            if h in hazards and h not in self.prev_hazards:
                log.event(t, "hazard", level=HAZARD_LEVEL[h], reason=h, response=a.response if h == a.reason else "",
                          gas_ppm=None if r.gas_ppm is None else round(r.gas_ppm, 1), gas_slope=round(a.gas_slope, 1),
                          front_m=None if r.front_dist == float("inf") else round(r.front_dist, 2),
                          tilt_deg=round(r.tilt_deg, 1), nav=cmd.state)
                lab = f"{HAZARD_LEVEL[h]} {h}"
                if lab not in self.seen_labels:
                    self.seen_labels.append(lab)
        self.prev_hazards = hazards
        navkey = (cmd.mode, cmd.state)
        if navkey != self.prev_nav:
            log.event(t, "nav", mode=cmd.mode, state=cmd.state, note=cmd.note)
            if cmd.state not in self.seen_nav:
                self.seen_nav.append(cmd.state)
            self.prev_nav = navkey
        if t >= self.next_status:
            self.next_status = t + self.cfg["log"]["status_period_s"]

            def fmt(v):
                return " -- " if v is None else f"{v:4.2f}"
            gas = "warm-up" if r.gas_ppm is None else f"{r.gas_ppm:4.0f}ppm {a.gas_slope:+5.1f}/s"
            print(f"[{t:6.1f}s] {cmd.mode}/{cmd.state:<11} {a.level:<7} {a.reason:<12} us L{fmt(r.us_left)} R{fmt(r.us_right)} m"
                  f"  gas {gas}  tilt {r.tilt_deg:4.1f}  yaw {r.yaw_deg:+7.1f}  cmd {cmd.left:+.2f} {cmd.right:+.2f}"
                  f"  {cmd.note}", flush=True)
        if t >= self.next_summary:
            self.next_summary = t + 10.0
            log.write_summary(f"Mine rover run {self.run_name}")

    def truth_stats(self):
        rpy = self.truth.getRollPitchYaw()
        if rpy[0] != rpy[0] or not self.att.calibrated:
            return
        # Webots: roll about X (left side up = +), pitch about Y (nose DOWN = +), yaw about Z
        self.att_err["roll"] = max(self.att_err["roll"], abs(math.degrees(rpy[0]) - self.att.roll_deg))
        self.att_err["pitch"] = max(self.att_err["pitch"], abs(-math.degrees(rpy[1]) - self.att.pitch_deg))
        if self.yaw_truth_prev is None:
            self.yaw_truth0 = self.att.yaw_deg
        else:
            d = math.degrees(rpy[2] - self.yaw_truth_prev)
            self.yaw_truth_unwrapped += (d + 180) % 360 - 180
        self.yaw_truth_prev = rpy[2]
        self.att_err["yaw_drift"] = (self.att.yaw_deg - self.yaw_truth0) - self.yaw_truth_unwrapped

    def telemetry(self, t, a, cmd):
        msg = {"t": round(t, 3), "level": a.level, "reason": a.reason, "mode": cmd.mode, "nav": cmd.state,
               "note": cmd.note, "images": len(self.log.images) if self.log else 0, "sweeps": self.seq.count,
               "sweeping": self.seq.busy, "seen": self.seen_labels, "nav_seen": self.seen_nav,
               "impacts": self.impacts, "req": self.requests,
               "att_err": {k: round(v, 2) for k, v in self.att_err.items()}}
        self.requests = []
        self.emitter.send(json.dumps(msg))

    # ------------------------------------------------------------------
    def finalize(self, t):
        if self.log is None:
            return
        self.log.event(t, "end", att_err={k: round(v, 2) for k, v in self.att_err.items()},
                       images=len(self.log.images), sweeps=self.seq.count)
        self.log.write_summary(f"Mine rover run {self.run_name}")
        self.log.close()
        self.log = None

    def run(self):
        dt = self.ts / 1000.0
        k = 0
        t = 0.0
        done_at = None
        while self.robot.step(self.ts) != -1:
            t = self.robot.getTime()
            a = self.accel.getValues()
            g = self.gyro.getValues()
            if k < 2:
                k += 1          # sensors deliver their first valid sample one period after enable
                continue
            if a[0] == a[0] and g[0] == g[0]:
                self.att.update(t, dt, self.imu_err.accel(a), self.imu_err.gyro(g))
                self.t_imu = t
            if k % self.loop_div == 0:
                self.cycle(t)
            k += 1
            # every step: dead-man + cut forward drive the moment a down IR loses the floor
            lost = any(d.getValue() > self.cfg["ir"]["floor_lost_m"] for d in self.ird.values())
            self.drive(*self.guard.get(t, floor_lost=lost))
            if self.finished and done_at is None:
                self.finalize(t)
                done_at = t
        if not self.finished:
            self.finalize(t)


if __name__ == "__main__":
    Rover().run()
