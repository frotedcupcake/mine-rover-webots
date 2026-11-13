"""World-side supervisor for the mine rover simulation.

* gas: evaluates the plume model at the rover and radios the ppm every 50 ms
* roof fall: on request (key D on the rover, or a scenario event) drops rocks ~0.4 m
  ahead of the rover and gives the chassis a short shove (accelerometer spike)
* metrics: path length, collisions, fell_in_pit, duration, hazard events -> metrics.json
* scenarios: timed events + expected outcomes from scenarios/<name>.yaml, pass/fail exit code
* hazard label in the 3D view, chase camera, screenshots, video

Environment: MINE_SCENARIO (yaml path), MINE_NOISE_SEED, MINE_SHIFT_SEED, MINE_RUN (report folder name),
MINE_VIDEO=1, MINE_SHOTS=1, MINE_CHASE=0/1.
"""
import json
import math
import os
import random
import sys
import time

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import yaml  # noqa: E402
from controller import Supervisor  # noqa: E402

from rover_core import config as rover_config  # noqa: E402
from rover_core.gas_model import GasField, GasSource  # noqa: E402

LABEL_COLOUR = {"SAFE": 0x35D060, "CAUTION": 0xFFB000, "DANGER": 0xFF3020}
ROCK = """Solid {{
  translation {x:.3f} {y:.3f} {z:.3f}
  rotation {ax:.3f} {ay:.3f} {az:.3f} {an:.3f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColorMap ImageTexture {{ url [ "../protos/textures/rubble.jpg" ] }}
        roughness 1
        metalness 0
      }}
      geometry Sphere {{ radius {r:.3f} subdivision 2 }}
    }}
  ]
  name "fallen rock {n}"
  model "fallen rock"
  boundingObject Sphere {{ radius {r:.3f} }}
  physics Physics {{
    density 1800
    damping Damping {{ linear 0.6 angular 0.9 }}
  }}
}}"""


def barrier_node(x, y, yaw, width, n):
    """A full-width fall of rock across the tunnel: a slab faced with rock columns (the columns
    give the sonar something to see at any angle, like the ribbed walls)."""
    shapes, bounds = [], []
    tex = ('appearance PBRAppearance { baseColorMap ImageTexture { url [ "../protos/textures/rubble.jpg" ] } '
           'roughness 1 metalness 0 }')
    shapes.append(f"Shape {{ {tex} geometry Box {{ size 0.16 {width:.3f} 0.6 }} }}")
    bounds.append(f"Box {{ size 0.16 {width:.3f} 0.6 }}")
    k = int(width / 0.11)
    for i in range(k + 1):
        py = -width / 2 + 0.03 + i * (width - 0.06) / k
        for px in (-0.08, 0.08):
            pose = f"translation {px:.3f} {py:.3f} 0"
            shapes.append(f"Pose {{ {pose} children [ Shape {{ {tex} geometry Cylinder {{ radius 0.05 height 0.6 subdivision 8 }} }} ] }}")
            bounds.append(f"Pose {{ {pose} children [ Cylinder {{ radius 0.05 height 0.6 }} ] }}")
    return (f'Solid {{ translation {x:.3f} {y:.3f} 0.3 rotation 0 0 1 {yaw:.4f} children [ {" ".join(shapes)} ] '
            f'name "barrier {n}" model "barrier" boundingObject Group {{ children [ {" ".join(bounds)} ] }} locked TRUE }}')


def env_flag(name, default=False):
    v = os.environ.get(name)
    return default if v is None or v == "" else v not in ("0", "false", "no")


class MineSupervisor:
    def __init__(self):
        self.sup = sup = Supervisor()
        self.basic = int(sup.getBasicTimeStep())
        self.ts = 2 * self.basic                       # 10 ms
        self.world = sup.getCustomData() or os.path.splitext(os.path.basename(sup.getWorldPath()))[0]
        # shift seed: start-pose shift (0 = none) and where dropped rocks land.
        # noise seed is only reported here (Webots randomSeed + the rover's own noise).
        self.seed = int(os.environ.get("MINE_SHIFT_SEED") or os.environ.get("MINE_SEED") or 0)
        self.noise_seed = int(os.environ.get("MINE_NOISE_SEED") or os.environ.get("MINE_SEED") or 1)
        self.rng = random.Random(self.seed)
        self.scenario = None
        spath = os.environ.get("MINE_SCENARIO")
        if spath:
            with open(spath) as fh:
                self.scenario = yaml.safe_load(fh)
        name = (self.scenario or {}).get("name", self.world)
        self.run_name = os.environ.get("MINE_RUN") or f"{name}_{time.strftime('%Y%m%d_%H%M%S')}"
        self.run_dir = os.path.join(ROOT, "reports", self.run_name)
        os.makedirs(self.run_dir, exist_ok=True)
        self.gas = GasField.from_yaml(os.path.join(ROOT, "config", "gas_sources", f"{self.world}.yaml"))
        # same config (and the same scenario overrides) as the rover
        self.cfg = rover_config.load(overrides=json.loads(os.environ.get("MINE_CFG", "{}") or "{}"))

        self.rover = sup.getFromDef("ROVER")
        self.view = sup.getFromDef("VIEW")
        self.emitter = sup.getDevice("emitter")
        self.receiver = sup.getDevice("receiver")
        self.receiver.enable(self.ts)
        self.rover.enableContactPointsTracking(self.ts, True)
        self.centre_z = self.rover.getField("translation").getSFVec3f()[2]

        self.rendering = env_flag("MINE_RENDER", True)
        self.chase = env_flag("MINE_CHASE", self.rendering)
        self.want_video = env_flag("MINE_VIDEO") or bool((self.scenario or {}).get("video"))
        self.want_shots = env_flag("MINE_SHOTS") or bool((self.scenario or {}).get("screenshots"))
        self.cam_state = None
        self.shot_queue = []
        self.shots = []
        self.video_path = None

        # metrics
        self.path_len = 0.0
        self.prev_pos = None
        self.start_pos = None
        self.collisions = []
        self.contact_last = -1e9
        self.fell = False
        self.fell_t = None
        self.min_z = 1e9
        self.timeline = []            # (t, level, reason) changes
        self.tel = {}                 # last telemetry from the rover
        self.tel_t = -1e9
        self.reached = {}
        self.rocks = 0
        self.kicks = []
        self.gas_mute_until = -1e9
        self.pending_cmd = []
        evs = (self.scenario or {}).get("events", []) or []
        # timed events have `t`; triggered ones have `when: return | home | hold | reach:<region>`
        # (not `on:` - YAML reads a bare `on` as the boolean True)
        self.events = sorted((e for e in evs if "when" not in e), key=lambda e: e["t"])
        self.triggered = [dict(e) for e in evs if "when" in e]
        self.ev_i = 0
        # dashboard link: the laptop sits at the entrance, where the rover is put down (the
        # nominal start position, before any start shift)
        tr = self.rover.getField("translation").getSFVec3f()
        self.entrance = list((self.scenario or {}).get("entrance") or tr[:2])
        self.link = True
        self.link_force = None            # scenario event: True / False / None (= by distance)
        self.link_flicker = None          # (until_t, period_s)
        self.link_changes = 0
        self.batt_charge = None           # pending test hook for the rover's simulated pack
        self.barriers = 0
        # trip tracking (from the rover's telemetry)
        self.t_return = self.return_pos = self.return_reason = None
        self.t_home = self.home_pos = None
        self.t_upload = None
        self.hold = None                  # open Danger hold episode
        self.holds = []                   # closed episodes: t0, s, gas_m, pos, reason
        self.end_since = None
        self.max_gas = 0.0
        self.min_gas_dist = float("inf")
        self.done = False
        self.label_prev = None

    # ------------------------------------------------------------------
    def pose(self):
        p = self.rover.getPosition()
        o = self.rover.getOrientation()
        yaw = math.atan2(o[3], o[0])
        return p, o, yaw

    def jitter_start(self):
        j = (self.scenario or {}).get("start_jitter")
        if not j or self.seed == 0:
            return
        tr = self.rover.getField("translation")
        rot = self.rover.getField("rotation")
        p = tr.getSFVec3f()
        r = rot.getSFRotation()
        yaw0 = r[3] * (1 if r[2] >= 0 else -1)
        dx = self.rng.uniform(-1, 1) * j.get("x", 0.0)
        dy = self.rng.uniform(-1, 1) * j.get("y", 0.0)
        dyaw = math.radians(self.rng.uniform(-1, 1) * j.get("yaw_deg", 0.0))
        tr.setSFVec3f([p[0] + dx, p[1] + dy, p[2]])
        rot.setSFRotation([0, 0, 1, yaw0 + dyaw])
        self.rover.resetPhysics()
        print(f"[sup] shift seed {self.seed}: start offset dx={dx:+.3f} dy={dy:+.3f} dyaw={math.degrees(dyaw):+.1f} deg", flush=True)

    # ------------------------------------------------------------------
    def roof_fall(self, t):
        p, o, yaw = self.pose()
        fx, fy = math.cos(yaw), math.sin(yaw)
        lx, ly = -fy, fx
        children = self.sup.getRoot().getField("children")
        for lat in (-0.17, 0.0, 0.17):
            self.rocks += 1
            ahead = 0.42 + self.rng.uniform(-0.02, 0.04)
            lat += self.rng.uniform(-0.02, 0.02)
            children.importMFNodeFromString(-1, ROCK.format(
                x=p[0] + fx * ahead + lx * lat, y=p[1] + fy * ahead + ly * lat, z=self.rng.uniform(0.28, 0.4),
                ax=self.rng.random(), ay=self.rng.random(), az=self.rng.random() + 0.1, an=self.rng.uniform(0, 3),
                r=self.rng.uniform(0.085, 0.105), n=self.rocks))
        # short shove on the chassis: three 10 ms pushes, backwards and a little sideways
        side = self.rng.choice((-1, 1)) * 12.0
        self.kicks = [(-45.0 * fx + side * lx, -45.0 * fy + side * ly)] * 3
        self.timeline.append((round(t, 2), "EVENT", "roof_fall"))
        print(f"[sup] t={t:.2f} roof fall: {3} rocks dropped ahead of the rover", flush=True)

    def do_event(self, t, ev):
        what = ev.get("do")
        if what == "roof_fall":
            self.roof_fall(t)
        elif what == "gas_dropout":
            self.gas_mute_until = t + float(ev.get("seconds", 2.0))
            print(f"[sup] t={t:.2f} gas radio dropout for {ev.get('seconds', 2.0)} s", flush=True)
        elif what == "cmd":
            self.pending_cmd.append(ev.get("cmd"))
        elif what == "screenshot":
            self.shot_queue.append((t, ev.get("name", "shot")))
        elif what == "barrier":
            # full-width rock fall across the tunnel at a fixed place (e.g. behind the rover)
            x, y = ev["at"]
            self.barriers += 1
            self.sup.getRoot().getField("children").importMFNodeFromString(-1, barrier_node(
                x, y, math.radians(ev.get("yaw_deg", 0.0)), float(ev.get("width", 1.2)), self.barriers))
            self.timeline.append((round(t, 2), "EVENT", "barrier"))
            print(f"[sup] t={t:.2f} barrier dropped across the tunnel at ({x}, {y})", flush=True)
        elif what == "gas_source":
            # a gas pocket that was not there on the way in
            x, y = ev["at"]
            src = GasSource(position=[x, y, 0.1], peak_ppm=float(ev.get("peak", 2500)), sigma_m=float(ev.get("sigma", 1.0)),
                            ramp_up_s=float(ev.get("ramp_up", 5.0)), trigger_distance_m=float(ev.get("trigger", 2.5)),
                            name=f"gas{len(self.gas.sources) + 1}")
            self.gas.sources.append(src)
            self.sup.getRoot().getField("children").importMFNodeFromString(
                -1, f'GasMarker {{ translation {x} {y} 0.35 name "gas marker event {len(self.gas.sources)}" radius {0.55 * src.sigma_m:.3f} }}')
            self.timeline.append((round(t, 2), "EVENT", "gas_source"))
            print(f"[sup] t={t:.2f} new gas pocket at ({x}, {y})", flush=True)
        elif what == "battery":
            self.batt_charge = float(ev["charge"])
            self.timeline.append((round(t, 2), "EVENT", f"battery charge -> {self.batt_charge}"))
            print(f"[sup] t={t:.2f} simulated battery charge set to {self.batt_charge}", flush=True)
        elif what == "link":
            st = ev.get("state", "auto")
            if st == "flicker":
                self.link_flicker = (t + float(ev.get("seconds", 5.0)), float(ev.get("period", 0.4)))
            else:
                self.link_force = {"up": True, "down": False}.get(st)
            print(f"[sup] t={t:.2f} link forced {st}", flush=True)

    def fire_triggered(self, t):
        """Events with `when:` fire once, `delay` seconds after their condition first holds."""
        for ev in self.triggered:
            if ev.get("_done"):
                continue
            if "_due" not in ev:
                on = str(ev["when"])
                hit = ((on == "return" and self.tel.get("dir") == "RETURN") or (on == "home" and self.tel.get("home"))
                       or (on == "hold" and self.tel.get("nav") == "HOLD")
                       or (on.startswith("reach:") and on[6:] in self.reached))
                if hit:
                    ev["_due"] = t + float(ev.get("delay", 0.0))
            if "_due" in ev and t >= ev["_due"]:
                ev["_done"] = True
                self.do_event(t, ev)

    def update_link(self, t, p):
        """Link up within link_range_m of the entrance, down link_hyst_m further out."""
        was = self.link
        if self.link_flicker and t < self.link_flicker[0]:
            self.link = int(t / self.link_flicker[1]) % 2 == 0
        elif self.link_force is not None:
            self.link = self.link_force
        else:
            d = math.dist(p[:2], self.entrance)
            lk = self.cfg["link"]
            if self.link and d > lk["link_range_m"] + lk["link_hyst_m"]:
                self.link = False
            elif not self.link and d <= lk["link_range_m"]:
                self.link = True
        if self.link != was:
            self.link_changes += 1
            self.timeline.append((round(t, 2), "LINK", "up" if self.link else "down"))
        return self.link

    # ------------------------------------------------------------------
    def update_metrics(self, t):
        p, o, yaw = self.pose()
        if self.prev_pos is not None:
            d = math.dist(p[:2], self.prev_pos[:2])
            if d < 0.2:                      # ignore teleports
                self.path_len += d
        else:
            self.start_pos = list(p)
        self.prev_pos = list(p)
        self.min_z = min(self.min_z, p[2])
        if p[2] < self.centre_z - 0.12 and not self.fell and self.in_pit_zone(p):
            self.fell, self.fell_t = True, round(t, 2)
            print(f"[sup] t={t:.2f} ROVER FELL (z={p[2]:.3f})", flush=True)
        # collisions: any contact point that is not a wheel bottom touching the ground
        if t > 0.3:
            hit = None
            for cp in self.rover.getContactPoints(True):
                c = cp.getPoint()
                d = [c[0] - p[0], c[1] - p[1], c[2] - p[2]]
                lz = o[2] * d[0] + o[5] * d[1] + o[8] * d[2]      # height in the rover frame
                if lz > -self.centre_z + 0.012:
                    lx = o[0] * d[0] + o[3] * d[1] + o[6] * d[2]
                    ly = o[1] * d[0] + o[4] * d[1] + o[7] * d[2]
                    hit = (round(lx, 3), round(ly, 3), round(lz, 3))
                    break
            if hit:
                if t - self.contact_last > 0.5:
                    self.collisions.append({"t": round(t, 2), "pos": [round(p[0], 2), round(p[1], 2)], "at": hit})
                    print(f"[sup] t={t:.2f} COLLISION at rover-frame {hit}", flush=True)
                self.contact_last = t
        self.min_gas_dist = min(self.min_gas_dist, self.gas.nearest_distance(p))
        for name, reg in ((self.scenario or {}).get("regions") or {}).items():
            if name not in self.reached and self.in_region(p, reg):
                self.reached[name] = round(t, 2)
                print(f"[sup] t={t:.2f} reached region '{name}'", flush=True)
        return p, o, yaw

    def in_pit_zone(self, p):
        return True

    @staticmethod
    def in_region(p, reg):
        return (reg.get("x_min", -1e9) <= p[0] <= reg.get("x_max", 1e9)
                and reg.get("y_min", -1e9) <= p[1] <= reg.get("y_max", 1e9))

    def read_rover(self, t):
        while self.receiver.getQueueLength() > 0:
            try:
                msg = json.loads(self.receiver.getString())
            except Exception:
                msg = None
            self.receiver.nextPacket()
            if not msg:
                continue
            prev = (self.tel.get("level"), self.tel.get("reason"))
            self.tel, self.tel_t = msg, t
            if (msg.get("level"), msg.get("reason")) != prev:
                self.timeline.append((round(t, 2), msg.get("level"), msg.get("reason")))
                if self.want_shots and msg.get("level") in ("CAUTION", "DANGER"):
                    self.shot_queue.append((t + 1.2, f"{msg['level']}_{msg['reason']}"))
            for req in msg.get("req") or []:
                if req == "drop_rock":
                    self.roof_fall(t)
                elif req == "toggle_cam":
                    self.chase = not self.chase
            self.track_trip(msg)

    def track_trip(self, msg):
        """Direction, HOME, upload and Danger holds, timed on the rover's own clock."""
        tr = msg.get("t", 0.0)
        pos = [round(v, 2) for v in (self.prev_pos or [0, 0, 0])[:2]]
        if msg.get("dir") == "RETURN" and self.t_return is None:
            self.t_return, self.return_pos, self.return_reason = tr, pos, msg.get("ret_reason")
            self.timeline.append((round(tr, 2), "RETURN", msg.get("ret_reason")))
            print(f"[sup] t={tr:.2f} rover turned back at {pos}: {msg.get('ret_reason')}", flush=True)
        if msg.get("dir") == "IN" and self.t_return is not None and self.t_home is not None:
            self.t_return = self.t_home = self.t_upload = None          # a new trip in (R at HOME)
        if msg.get("home") and self.t_home is None:
            self.t_home, self.home_pos = tr, pos
            self.timeline.append((round(tr, 2), "HOME", msg.get("note")))
            print(f"[sup] t={tr:.2f} rover HOME at {pos}, {math.dist(pos, self.entrance):.2f} m from the entrance", flush=True)
        if msg.get("uploaded") and self.t_upload is None:
            self.t_upload = tr
            self.timeline.append((round(tr, 2), "UPLOAD", "dashboard"))
        nav = msg.get("nav")
        if self.hold is None and nav == "HOLD":
            why = (msg.get("note") or "").replace("holding: ", "").split(" (")[0] or msg.get("reason")
            self.hold = {"t0": tr, "pos": pos, "reason": why, "dir": msg.get("dir"),
                         "gas_m": None if not self.gas.sources else round(self.gas.nearest_distance(self.prev_pos or [0, 0, 0]), 2)}
        elif self.hold is not None and nav not in ("HOLD", "SWEEP"):
            self.hold["s"] = round(tr - self.hold["t0"], 2)
            self.hold["then"] = nav
            self.holds.append(self.hold)
            self.hold = None

    def all_holds(self):
        """Closed hold episodes plus one still open at the end (marked)."""
        out = list(self.holds)
        if self.hold is not None:
            out.append(dict(self.hold, s=round(self.tel.get("t", 0.0) - self.hold["t0"], 2), then="(still holding)"))
        return out

    def update_label(self):
        level, reason = self.tel.get("level", "SAFE"), self.tel.get("reason", "")
        nav = f"{self.tel.get('mode', '')}/{self.tel.get('nav', '')}"
        text = f"{level}  {reason}   [{nav}]" if self.tel else "waiting for rover..."
        # second line = what the dashboard shows about the trip: direction, battery, link
        tel = self.tel
        lvl = tel.get("batt_level", "OK")
        v = tel.get("batt_avg") if tel.get("batt_avg") is not None else tel.get("batt_v")
        trip = tel.get("dir", "")
        if tel.get("home"):
            trip = "HOME" + (" - log uploaded" if tel.get("uploaded") else " - upload waiting for link")
        elif tel.get("ret_reason"):
            trip = f"RETURN ({tel['ret_reason']})"
        batt = "battery --" if v is None else f"battery {v:.1f} V" + ("" if lvl in ("OK", "RETURN") else f" {lvl}!")
        line2 = f"{trip}   {batt}{' (return level)' if lvl == 'RETURN' else ''}   link {'UP' if tel.get('link') else 'down'}" if tel else ""
        if (text, line2) != self.label_prev:
            self.sup.setLabel(0, text, 0.02, 0.9, 0.1, LABEL_COLOUR.get(level, 0xFFFFFF), 0.0, "Arial")
            colour = {"WARN": LABEL_COLOUR["CAUTION"], "CRITICAL": LABEL_COLOUR["DANGER"]}.get(lvl, 0xE8E2D8)
            self.sup.setLabel(1, line2, 0.02, 0.85, 0.07, colour, 0.0, "Arial")
            self.label_prev = (text, line2)

    def update_camera(self, p, yaw):
        if not self.chase or self.view is None:
            return
        if self.cam_state is None:
            self.cam_state = [p[0] - 1.0 * math.cos(yaw), p[1] - 1.0 * math.sin(yaw), yaw]
        cx, cy, cyaw = self.cam_state
        k = 0.03
        dyaw = (yaw - cyaw + math.pi) % (2 * math.pi) - math.pi
        cyaw += k * 0.7 * dyaw
        tx, ty = p[0] - 1.05 * math.cos(cyaw), p[1] - 1.05 * math.sin(cyaw)
        cx += k * 3 * (tx - cx)
        cy += k * 3 * (ty - cy)
        self.cam_state = [cx, cy, cyaw]
        look = math.atan2(p[1] - cy, p[0] - cx)
        pitch = 0.5
        # orientation = Rz(look) * Ry(pitch) as axis-angle
        cz, sz, cp_, sp = math.cos(look), math.sin(look), math.cos(pitch), math.sin(pitch)
        m = [[cz * cp_, -sz, cz * sp], [sz * cp_, cz, sz * sp], [-sp, 0, cp_]]
        ang = math.acos(max(-1, min(1, (m[0][0] + m[1][1] + m[2][2] - 1) / 2)))
        if ang < 1e-6:
            axis = [0, 0, 1]
        else:
            s = 2 * math.sin(ang)
            axis = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
        self.view.getField("position").setSFVec3f([cx, cy, 0.82])
        self.view.getField("orientation").setSFRotation(axis + [ang])

    def overview_shot(self, t):
        """Cut-away view from above (the roof is one-sided), if the scenario defines one."""
        ov = (self.scenario or {}).get("overview")
        if not ov or self.view is None:
            return
        # the mine is dark and foggy: light it and clear the fog just for this one picture
        children = self.sup.getRoot().getField("children")
        children.importMFNodeFromString(-1, "DEF OVERVIEW_LIGHT DirectionalLight "
                                            "{ direction 0.25 0.15 -1 intensity 1.6 castShadows FALSE }")
        fog = self.sup.getFromDef("FOG")
        fog_range = fog.getField("visibilityRange").getSFFloat() if fog else None
        if fog:
            fog.getField("visibilityRange").setSFFloat(500.0)
        self.view.getField("position").setSFVec3f(ov["position"])
        self.view.getField("orientation").setSFRotation(ov["orientation"])
        for _ in range(3):
            self.sup.step(self.ts)
        d = os.path.join(self.run_dir, "shots")
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, "00_overview.jpg")
        self.sup.exportImage(path, 92)
        self.shots.append(path)
        light = self.sup.getFromDef("OVERVIEW_LIGHT")
        if light:
            light.remove()
        if fog:
            fog.getField("visibilityRange").setSFFloat(fog_range)
        self.cam_state = None

    def take_shots(self, t):
        keep = []
        for due, name in self.shot_queue:
            # wait for the camera sweep to finish: its overlay would cover the 3D view
            if t >= due and (not self.tel.get("sweeping") or name == "end"):
                d = os.path.join(self.run_dir, "shots")
                os.makedirs(d, exist_ok=True)
                path = os.path.join(d, f"{len(self.shots) + 1:02d}_t{t:05.1f}_{name}.jpg")
                self.sup.exportImage(path, 92)
                self.shots.append(path)
            else:
                keep.append((due, name))
        self.shot_queue = keep

    # ------------------------------------------------------------------
    def check_end(self, t):
        sc = self.scenario
        if not sc:
            return False
        if t >= float(sc.get("duration", 120)):
            return True
        ew = sc.get("end_when")
        if not ew:
            return False
        ok = True
        if "nav" in ew:
            ok = ok and self.tel.get("nav") in ([ew["nav"]] if isinstance(ew["nav"], str) else ew["nav"])
            ok = ok and not self.tel.get("sweeping")
        if "reach" in ew:
            ok = ok and ew["reach"] in self.reached
        if "seen" in ew:
            ok = ok and all(s in (self.tel.get("seen") or []) for s in ew["seen"])
        if ew.get("uploaded"):
            ok = ok and bool(self.tel.get("uploaded"))
        if ew.get("home"):
            ok = ok and bool(self.tel.get("home"))
        if ew.get("or_nav"):              # e.g. give up early once the rover is STUCK
            ok = ok or self.tel.get("nav") in ew["or_nav"]
        if ew.get("fell"):
            ok = ok or self.fell
        if ok:
            if self.end_since is None:
                self.end_since = t
            return t - self.end_since >= float(ew.get("settle_s", 2.0))
        self.end_since = None
        return False

    def evaluate(self):
        ex = (self.scenario or {}).get("expect") or {}
        tel = self.tel
        seen = tel.get("seen") or []
        nav_seen = tel.get("nav_seen") or []
        checks = []

        def add(name, ok, detail=""):
            checks.append({"check": name, "ok": bool(ok), "detail": str(detail)})
        if "max_collisions" in ex:
            add(f"collisions <= {ex['max_collisions']}", len(self.collisions) <= ex["max_collisions"],
                f"{len(self.collisions)} collision(s) {self.collisions[:3]}")
        if ex.get("no_fall"):
            add("never falls in a pit", not self.fell, f"fell at t={self.fell_t}" if self.fell else f"min z {self.min_z:.3f}")
        for reg in ex.get("reach", []) or []:
            add(f"reaches '{reg}'", reg in self.reached, f"t={self.reached.get(reg)}")
        for lab in ex.get("seen", []) or []:
            add(f"reported {lab}", lab in seen, f"seen: {seen}")
        for lab in ex.get("not_seen", []) or []:
            add(f"never reported {lab}", lab not in seen, f"seen: {seen}")
        for st in ex.get("nav_seen", []) or []:
            add(f"navigator entered {st}", st in nav_seen, f"states: {nav_seen}")
        if "final_nav" in ex:
            want = ex["final_nav"] if isinstance(ex["final_nav"], list) else [ex["final_nav"]]
            add(f"ends in {'/'.join(want)}", tel.get("nav") in want, f"final nav {tel.get('nav')} ({tel.get('note')})")
        if "final_note_contains" in ex:
            add(f"final note contains '{ex['final_note_contains']}'", ex["final_note_contains"] in (tel.get("note") or ""),
                tel.get("note"))
        if "min_images" in ex:
            add(f"at least {ex['min_images']} sweep images", tel.get("images", 0) >= ex["min_images"], f"{tel.get('images', 0)} images")
        holds = self.all_holds()
        if "max_gas_distance_m" in ex:
            # where the rover stopped and held for the gas (it drives home afterwards)
            d = holds[0]["gas_m"] if holds and holds[0]["gas_m"] is not None else float("inf")
            add(f"stops within {ex['max_gas_distance_m']} m of the gas source", d <= ex["max_gas_distance_m"],
                f"held {d:.2f} m from it" if holds else "never held")
        if "holds" in ex:
            add(f"{ex['holds']} Danger hold(s)", len(holds) == ex["holds"],
                "; ".join(f"t={h['t0']} {h['reason']} {h['s']} s ({h['dir']})" for h in holds) or "none")
        if "hold_min_s" in ex:
            add(f"every Danger hold lasts >= {ex['hold_min_s']} s", bool(holds) and all(h["s"] >= ex["hold_min_s"] - 1e-3 for h in holds),
                ", ".join(f"{h['s']} s then {h['then']}" for h in holds) or "no hold")
        if "returns" in ex:
            want = bool(ex["returns"])
            add("turns back (RETURN)" if want else "does not turn back", (self.t_return is not None) == want,
                f"RETURN at t={self.t_return} ({self.return_reason}) at {self.return_pos}" if self.t_return is not None else "never returned")
        if "return_reason" in ex:
            add(f"return reason contains '{ex['return_reason']}'", ex["return_reason"] in (self.return_reason or ""), self.return_reason)
        if ex.get("home"):
            d = math.dist((self.prev_pos or [0, 0])[:2], self.entrance)
            add("reaches HOME", tel.get("home") is True, f"HOME at t={self.t_home}, {d:.2f} m from the entrance ({tel.get('note')})"
                if self.t_home is not None else f"final nav {tel.get('nav')} ({tel.get('note')})")
        if "home_max_entrance_m" in ex:
            d = math.dist((self.prev_pos or [0, 0])[:2], self.entrance)
            add(f"ends within {ex['home_max_entrance_m']} m of the entrance", d <= ex["home_max_entrance_m"], f"{d:.2f} m")
        if ex.get("uploaded"):
            f_ok = os.path.isfile(os.path.join(self.run_dir, "dashboard", "hazard_log.csv")) \
                and os.path.isfile(os.path.join(self.run_dir, "dashboard", "summary.html"))
            add("log uploaded to the dashboard", bool(tel.get("uploaded")) and f_ok,
                f"uploaded at t={self.t_upload}, dashboard/ copy {'present' if f_ok else 'MISSING'}")
        if "return_between_s" in ex:
            lo, hi = ex["return_between_s"]
            add(f"turns back between {lo} and {hi} s", self.t_return is not None and lo <= self.t_return <= hi, f"t={self.t_return}")
        if "flushed_on" in ex:            # the rover's own log must show a flush on entering that state
            evs = []
            try:
                with open(os.path.join(self.run_dir, "events.jsonl")) as fh:
                    evs = [json.loads(line) for line in fh if line.strip()]
            except (OSError, ValueError):
                pass
            fl = [e for e in evs if e.get("type") == "flush" and e.get("state") == ex["flushed_on"]]
            add(f"log flushed on entering {ex['flushed_on']}", bool(fl), f"flush at t={fl[0]['t']} ({fl[0].get('note')})" if fl else "no flush event")
        if ex.get("link_dropped"):
            downs = [x for x in self.timeline if x[1] == "LINK" and x[2] == "down"]
            add("link dropped on the way in", bool(downs), f"down at t={downs[0][0]}" if downs else "never dropped")
        if "battery_level" in ex:
            want = ex["battery_level"] if isinstance(ex["battery_level"], list) else [ex["battery_level"]]
            add(f"battery level ends {'/'.join(want)}", tel.get("batt_level") in want,
                f"{tel.get('batt_level')} (5 s average {tel.get('batt_avg')} V, lowest {tel.get('batt_min_avg')} V)")
        if "link_at_end" in ex:
            add(f"link {'up' if ex['link_at_end'] else 'down'} at the end", bool(tel.get("link")) == bool(ex["link_at_end"]), tel.get("link"))
        if "order" in ex:      # labels must first appear in this order
            idx = [seen.index(x) if x in seen else -1 for x in ex["order"]]
            add(f"order {' -> '.join(ex['order'])}", all(i >= 0 for i in idx) and idx == sorted(idx), f"seen: {seen}")
        return checks

    def metrics(self, t, final=False):
        tel = self.tel
        m = {
            "run": self.run_name, "world": self.world, "scenario": (self.scenario or {}).get("name"),
            "noise_seed": self.noise_seed, "shift_seed": self.seed, "duration_s": round(t, 2), "path_length_m": round(self.path_len, 3),
            "collisions": len(self.collisions), "collision_events": self.collisions[:20],
            "fell_in_pit": self.fell, "fell_at_s": self.fell_t,
            "final_position": [round(v, 3) for v in (self.prev_pos or [0, 0, 0])],
            "final_level": tel.get("level"), "final_reason": tel.get("reason"),
            "final_mode": tel.get("mode"), "final_nav": tel.get("nav"), "final_note": tel.get("note"),
            "hazards_seen": tel.get("seen") or [], "nav_states_seen": tel.get("nav_seen") or [],
            "hazard_timeline": [list(x) for x in self.timeline[:200]],
            "sweep_images": tel.get("images", 0), "sweeps": tel.get("sweeps", 0), "impacts": tel.get("impacts", 0),
            "regions_reached": self.reached, "rocks_dropped": self.rocks,
            "max_gas_ppm_at_rover": round(self.max_gas, 1),
            "min_distance_to_gas_m": None if self.min_gas_dist == float("inf") else round(self.min_gas_dist, 2),
            "attitude_filter_max_error_deg": tel.get("att_err"),
            "direction": tel.get("dir"), "return_reason": self.return_reason, "returned_at_s": self.t_return,
            "returned_from": self.return_pos, "home": bool(tel.get("home")), "home_at_s": self.t_home,
            "home_position": self.home_pos, "uploaded": bool(tel.get("uploaded")), "uploaded_at_s": self.t_upload,
            "entrance": [round(v, 3) for v in self.entrance],
            "entrance_distance_m": round(math.dist((self.prev_pos or [0, 0])[:2], self.entrance), 2),
            "driving_time_in_s": tel.get("out_s"), "driving_time_back_s": tel.get("ret_s"),
            "link": bool(tel.get("link")), "link_changes": self.link_changes,
            "battery": {"level": tel.get("batt_level"), "volts": tel.get("batt_v"), "avg_v": tel.get("batt_avg"),
                        "min_avg_v": tel.get("batt_min_avg")},
            "danger_holds": self.all_holds(), "barriers": self.barriers,
            "screenshots": [os.path.relpath(s, self.run_dir) for s in self.shots],
            "video": os.path.relpath(self.video_path, self.run_dir) if self.video_path else None,
        }
        if self.scenario and final:
            checks = self.evaluate()
            m["checks"] = checks
            m["passed"] = all(c["ok"] for c in checks)
        with open(os.path.join(self.run_dir, "metrics.json"), "w") as fh:
            json.dump(m, fh, indent=2)
        return m

    # ------------------------------------------------------------------
    def run(self):
        sup = self.sup
        self.jitter_start()
        print(f"[sup] world '{self.world}' run '{self.run_name}' noise seed {self.noise_seed} shift seed {self.seed} "
              f"scenario {(self.scenario or {}).get('name')} gas sources {len(self.gas.sources)}", flush=True)
        k = 0
        t = 0.0
        next_save = 5.0
        while sup.step(self.ts) != -1:
            t = sup.getTime()
            k += 1
            while self.ev_i < len(self.events) and t >= self.events[self.ev_i]["t"]:
                self.do_event(t, self.events[self.ev_i])
                self.ev_i += 1
            if self.triggered:
                self.fire_triggered(t)
            if self.kicks:
                fx, fy = self.kicks.pop(0)
                self.rover.addForce([fx, fy, 0.0], False)
            p, o, yaw = self.update_metrics(t)
            self.read_rover(t)
            if k % 5 == 0:                                   # 20 Hz radio
                ppm = self.gas.ppm_at(p, t)
                self.max_gas = max(self.max_gas, ppm)
                msg = {"t": round(t, 3), "run": self.run_name, "link": self.update_link(t, p)}
                if self.batt_charge is not None:
                    msg["batt_charge"], self.batt_charge = self.batt_charge, None
                if t >= self.gas_mute_until:
                    msg["gas"] = round(ppm, 2)
                    if self.pending_cmd:
                        msg["cmd"], self.pending_cmd = self.pending_cmd, []
                    self.emitter.send(json.dumps(msg))
                self.update_label()
            if self.rendering:
                self.update_camera(p, yaw)
                if self.want_video and self.video_path is None and t > 0.1:
                    self.video_path = os.path.join(self.run_dir, "run.mp4")
                    sup.movieStartRecording(self.video_path, 1280, 720, 0, 90, 1, False)
                    print(f"[sup] recording video to {self.video_path}", flush=True)
                if self.want_shots and k == 250:             # 2.5 s: scene fully loaded and lit
                    self.overview_shot(t)
                    self.shot_queue.append((t + 0.5, "start"))
                self.take_shots(t)
            if t >= next_save:
                next_save = t + 5.0
                self.metrics(t)
            if self.scenario and not self.done and self.check_end(t):
                self.done = True
                break
        self.finish(t)

    def finish(self, t):
        sup = self.sup
        m = self.metrics(t, final=True)
        if not self.scenario:
            return
        if self.rendering and self.want_shots:
            self.shot_queue.append((t, "end"))
            self.take_shots(t)
        # tell the rover to write its summary, give it a moment
        self.emitter.send(json.dumps({"t": t, "run": self.run_name, "cmd": ["finish"]}))
        for _ in range(30):
            if sup.step(self.ts) == -1:
                break
        if self.video_path:
            sup.movieStopRecording()
            sup.simulationSetMode(sup.SIMULATION_MODE_REAL_TIME)
            t_wall = time.time()                       # encoding runs in wall-clock time
            while not sup.movieIsReady() and not sup.movieFailed() and time.time() - t_wall < 300:
                if sup.step(self.ts) == -1:
                    break
            ok = sup.movieIsReady() and not sup.movieFailed() and os.path.exists(self.video_path)
            print(f"[sup] video {'saved' if ok else 'FAILED'}: {self.video_path}", flush=True)
            if not ok:
                self.video_path = None
        m = self.metrics(sup.getTime(), final=True)
        print(f"[sup] scenario '{self.scenario.get('name')}' noise {self.noise_seed} shift {self.seed}: "
              f"{'PASS' if m.get('passed') else 'FAIL'}", flush=True)
        for c in m.get("checks", []):
            print(f"[sup]   {'ok  ' if c['ok'] else 'FAIL'} {c['check']}  ({c['detail']})", flush=True)
        if env_flag("MINE_QUIT", True):
            sup.simulationQuit(0 if m.get("passed") else 1)
            sup.step(self.ts)


if __name__ == "__main__":
    MineSupervisor().run()
