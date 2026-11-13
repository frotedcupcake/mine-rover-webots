#!/usr/bin/env python3
"""Generate worlds/*.wbt and config/gas_sources/*.yaml for the mine rover project.

All worlds share one visual style and one set of building blocks (rock walls with
rib-like relief, dirt floor, one-sided roof, timber sets, dim lamps).  Edit the
layout functions at the bottom and re-run:

    .venv/bin/python tools/build_worlds.py
"""
import math
import os
import random

import numpy as np
import yaml

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
H = 1.0          # tunnel height (m)
WT = 0.2         # wall thickness (m)
HALF = 0.6       # tunnel half width (m)  -> 1.2 m wide
ROVER_Z = 0.066


def axis_angle(m):
    """Rotation matrix -> Webots axis-angle (x y z a)."""
    a = math.acos(max(-1.0, min(1.0, (np.trace(m) - 1) / 2)))
    if abs(a) < 1e-9:
        return (0, 0, 1, 0)
    ax = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]]) / (2 * math.sin(a))
    return (*ax, a)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def f(v):
    return f"{v:.4f}".rstrip("0").rstrip(".") if abs(v) > 1e-9 else "0"


def tex(name, sx, sy):
    return (f'PBRAppearance {{ baseColorMap ImageTexture {{ url [ "../protos/textures/{name}.jpg" ] }} '
            f'roughness 1 metalness 0 textureTransform TextureTransform {{ scale {f(sx)} {f(sy)} }} }}')


class World:
    def __init__(self, name, title):
        self.name, self.title = name, title
        self.nodes = []
        self.n = 0
        self.gas = []
        self.rng = random.Random(hash(name) % 1000 if False else sum(map(ord, name)))
        self.start = (0.6, 0.0, 0.0)
        self.lights = 0

    def uid(self, prefix):
        self.n += 1
        return f"{prefix} {self.n}"

    # ---------------- building blocks ----------------
    def box(self, cx, cy, cz, sx, sy, sz, texture, name, bounding=True, tscale=None):
        ts = tscale or (max(sx, sy), max(sz, min(sx, sy)))
        b = f"boundingObject Box {{ size {f(sx)} {f(sy)} {f(sz)} }}" if bounding else ""
        self.nodes.append(f"""Solid {{
  translation {f(cx)} {f(cy)} {f(cz)}
  children [
    Shape {{
      appearance {tex(texture, *ts)}
      geometry Box {{ size {f(sx)} {f(sy)} {f(sz)} }}
    }}
  ]
  name "{self.uid(name)}"
  {b}
  locked TRUE
}}""")

    def floor(self, x0, y0, x1, y1):
        self.box((x0 + x1) / 2, (y0 + y1) / 2, -0.05, x1 - x0, y1 - y0, 0.1, "floor", "floor",
                 tscale=(x1 - x0, y1 - y0))

    def floor_with_hole(self, x0, y0, x1, y1, hx0, hy0, hx1, hy1, depth=0.5):
        """Floor rectangle with a rectangular pit (hole + liner)."""
        if hx0 > x0:
            self.floor(x0, y0, hx0, y1)
        if hx1 < x1:
            self.floor(hx1, y0, x1, y1)
        if hy0 > y0:
            self.floor(hx0, y0, hx1, hy0)
        if hy1 < y1:
            self.floor(hx0, hy1, hx1, y1)
        t = 0.06
        zc, hz = -(depth + 0.1) / 2 - 0.05 + 0.05, depth - 0.1
        zc = -0.1 - hz / 2
        self.box((hx0 + hx1) / 2, (hy0 + hy1) / 2, -depth - t / 2, hx1 - hx0 + 2 * t, hy1 - hy0 + 2 * t, t,
                 "rock", "pit bottom")
        self.box(hx0 - t / 2, (hy0 + hy1) / 2, zc, t, hy1 - hy0, hz, "rock", "pit side")
        self.box(hx1 + t / 2, (hy0 + hy1) / 2, zc, t, hy1 - hy0, hz, "rock", "pit side")
        self.box((hx0 + hx1) / 2, hy0 - t / 2, zc, hx1 - hx0, t, hz, "rock", "pit side")
        self.box((hx0 + hx1) / 2, hy1 + t / 2, zc, hx1 - hx0, t, hz, "rock", "pit side")

    def roof(self, x0, y0, x1, y1):
        """One-sided roof (visible from inside, see-through from above for overview shots)."""
        self.nodes.append(f"""Pose {{
  translation {f((x0 + x1) / 2)} {f((y0 + y1) / 2)} {f(H)}
  rotation 1 0 0 3.14159265
  children [
    Shape {{
      appearance {tex("rock", x1 - x0, y1 - y0)}
      geometry Plane {{ size {f(x1 - x0)} {f(y1 - y0)} }}
      castShadows FALSE
    }}
  ]
}}""")

    def wall(self, x0, y0, x1, y1, nx, ny, ribs=True):
        """Axis-aligned rock wall from (x0,y0) to (x1,y1); (nx,ny) points into the tunnel.

        The rib-like relief (vertical rock columns) is real collision geometry: it makes
        the wall scatter sonar at oblique angles, like rough rock does."""
        along_x = abs(y1 - y0) < 1e-9
        length = abs(x1 - x0) if along_x else abs(y1 - y0)
        cx, cy = (x0 + x1) / 2 - nx * WT / 2, (y0 + y1) / 2 - ny * WT / 2
        sx, sy = (length, WT) if along_x else (WT, length)
        shapes = [f"""    Shape {{
      appearance {tex("rock", length, H)}
      geometry Box {{ size {f(sx)} {f(sy)} {f(H)} }}
    }}"""]
        bounds = [f"      Box {{ size {f(sx)} {f(sy)} {f(H)} }}"]
        if ribs:
            s = 0.08
            while s < length - 0.06:
                r = self.rng.uniform(0.038, 0.05)
                px = (min(x0, x1) + s if along_x else x0) - cx
                py = (y0 if along_x else min(y0, y1) + s) - cy
                pose = f"translation {f(px)} {f(py)} 0"
                shapes.append(f"""    Pose {{
      {pose}
      children [
        Shape {{
          appearance {tex("rock", 1, 3)}
          geometry Cylinder {{ radius {f(r)} height {f(H)} subdivision 8 top FALSE bottom FALSE }}
        }}
      ]
    }}""")
                bounds.append(f"      Pose {{ {pose} children [ Cylinder {{ radius {f(r)} height {f(H)} }} ] }}")
                s += self.rng.uniform(0.10, 0.12)
        self.nodes.append(f"""Solid {{
  translation {f(cx)} {f(cy)} {f(H / 2)}
  children [
{chr(10).join(shapes)}
  ]
  name "{self.uid('wall')}"
  boundingObject Group {{
    children [
{chr(10).join(bounds)}
    ]
  }}
  locked TRUE
}}""")

    def timber_x(self, x, y0=-HALF, y1=HALF):
        """Timber set across a tunnel running along X (posts on both walls + cap)."""
        b = 0.08
        self.box(x, y0 + b / 2, H / 2, b, b, H, "wood", "post", tscale=(1, 4))
        self.box(x, y1 - b / 2, H / 2, b, b, H, "wood", "post", tscale=(1, 4))
        self.box(x, (y0 + y1) / 2, H - b / 2, b, y1 - y0, b, "wood", "cap", bounding=False, tscale=(4, 1))

    def timber_y(self, y, x0, x1):
        b = 0.08
        self.box(x0 + b / 2, y, H / 2, b, b, H, "wood", "post", tscale=(1, 4))
        self.box(x1 - b / 2, y, H / 2, b, b, H, "wood", "post", tscale=(1, 4))
        self.box((x0 + x1) / 2, y, H - b / 2, x1 - x0, b, b, "wood", "cap", bounding=False, tscale=(4, 1))

    def cable(self, x0, y0, x1, y1, nx, ny):
        """Old pipe + cable run along a wall (visual only)."""
        along_x = abs(y1 - y0) < 1e-9
        length = abs(x1 - x0) if along_x else abs(y1 - y0)
        for z, r, col in ((0.78, 0.022, "0.25 0.2 0.16"), (0.66, 0.008, "0.04 0.04 0.04")):
            off = 0.075 + r
            rot = "0 1 0 1.5707963" if along_x else "1 0 0 1.5707963"
            self.nodes.append(f"""Pose {{
  translation {f((x0 + x1) / 2 + nx * off)} {f((y0 + y1) / 2 + ny * off)} {f(z)}
  rotation {rot}
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {col} roughness 0.8 metalness 0.4 }}
      geometry Cylinder {{ radius {f(r)} height {f(length)} subdivision 8 }}
      castShadows FALSE
    }}
  ]
}}""")

    def lamp(self, x, y, intensity=0.5, color="1 0.7 0.4", z=0.88):
        self.lights += 1
        self.nodes.append(f"""PointLight {{
  attenuation 0 0 1
  color {color}
  intensity {f(intensity)}
  location {f(x)} {f(y)} {f(z)}
  radius 7
  castShadows FALSE
}}
Pose {{
  translation {f(x)} {f(y)} {f(z + 0.04)}
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {color} emissiveColor {color} emissiveIntensity 2 roughness 1 metalness 0 }}
      geometry Sphere {{ radius 0.035 subdivision 2 }}
      castShadows FALSE
    }}
  ]
}}""")

    def entrance(self, x=0.0):
        """Blocked-up adit mouth at the start: timber frame with a pale 'daylight' panel."""
        self.wall(x, -HALF, x, HALF, 1, 0, ribs=False)
        self.nodes.append(f"""Pose {{
  translation {f(x + 0.003)} 0 {f(H / 2)}
  rotation 0 1 0 1.5707963
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor 0 0 0 emissiveColor 0.55 0.65 0.8 emissiveIntensity 0.6 roughness 1 metalness 0 }}
      geometry Plane {{ size {f(H - 0.25)} {f(2 * HALF - 0.4)} }}
      castShadows FALSE
    }}
  ]
}}""")
        self.timber_x(x + 0.06)
        self.lamp(x + 0.35, 0, intensity=0.25, color="0.6 0.72 1", z=0.6)

    def rubble(self, x, y, size=0.4, rot=0.0):
        self.nodes.append(f"""Rubble {{
  translation {f(x)} {f(y)} 0
  rotation 0 0 1 {f(rot)}
  name "{self.uid('rubble')}"
  size {f(size)}
}}""")

    def gas_source(self, x, y, peak=2500, sigma=1.0, ramp_up=5.0, trigger=3.5):
        self.gas.append(dict(name=f"gas{len(self.gas) + 1}", position=[x, y, 0.1], peak_ppm=peak,
                             sigma_m=sigma, ramp_up_s=ramp_up, trigger_distance_m=trigger))
        self.nodes.append(f"""GasMarker {{
  translation {f(x)} {f(y)} 0.35
  name "{self.uid('gas marker')}"
  radius {f(0.55 * sigma)}
}}""")

    def ramp(self, x, y, heading, length=1.6, width=1.1, angle_deg=25.0):
        """Ramp whose lower edge is centred at (x,y) on the floor and rises along `heading`."""
        a = math.radians(angle_deg)
        t = 0.06
        m = rz(heading) @ ry(-a)
        c = m @ np.array([length / 2, 0, -t / 2]) + np.array([x, y, 0])
        ax = axis_angle(m)
        self.nodes.append(f"""Solid {{
  translation {f(c[0])} {f(c[1])} {f(c[2])}
  rotation {' '.join(f(v) for v in ax)}
  children [
    Shape {{
      appearance {tex("floor", length, width)}
      geometry Box {{ size {f(length)} {f(width)} {f(t)} }}
    }}
  ]
  name "{self.uid('ramp')}"
  boundingObject Box {{ size {f(length)} {f(width)} {f(t)} }}
  locked TRUE
}}""")

    # ---------------- composite: straight tunnel along +X ----------------
    def tunnel_x(self, x0, x1, pit=None, end_wall=True, lamps=True, timbers=True, left_gap=None):
        if pit:
            self.floor_with_hole(x0, -HALF, x1, HALF, *pit)
        else:
            self.floor(x0, -HALF, x1, HALF)
        self.roof(x0, -HALF, x1, HALF)
        self.wall(x0, -HALF, x1, -HALF, 0, 1)
        if left_gap:
            self.wall(x0, HALF, left_gap[0], HALF, 0, -1)
            self.wall(left_gap[1], HALF, x1, HALF, 0, -1)
        else:
            self.wall(x0, HALF, x1, HALF, 0, -1)
        self.cable(x0, -HALF, x1, -HALF, 0, 1)
        self.entrance(x0)
        if end_wall:
            self.wall(x1, -HALF, x1, HALF, -1, 0)
        if timbers:
            x = x0 + 1.7
            while x < x1 - 0.8:
                if not pit or not (pit[0] - 0.5 < x < pit[2] + 0.5):
                    self.timber_x(x)
                x += 2.6
        if lamps:
            x = x0 + 2.4
            while x < x1 - 0.5:
                self.lamp(x, -HALF + 0.12)
                x += 3.6

    # ---------------- output ----------------
    def write(self, follow=True):
        x, y, yaw = self.start
        header = f"""#VRML_SIM R2025a utf8
# Generated by tools/build_worlds.py - edit that script rather than this file.

EXTERNPROTO "../protos/MineRover.proto"
EXTERNPROTO "../protos/Rubble.proto"
EXTERNPROTO "../protos/GasMarker.proto"

WorldInfo {{
  info [
    "ECE357 Autonomous Mine Rover - {self.title}"
  ]
  title "{self.title}"
  basicTimeStep 5
  FPS 30
  optimalThreadCount 1
  randomSeed 1
  contactProperties [
    ContactProperties {{
      material1 "wheel"
      coulombFriction [ 1.0 ]
      bounce 0
      bumpSound ""
      rollSound ""
      slideSound ""
    }}
    ContactProperties {{
      material1 "chassis"
      coulombFriction [ 0.3 ]
      bounce 0.1
      bumpSound ""
      rollSound ""
      slideSound ""
    }}
    ContactProperties {{
      bounce 0.1
      bumpSound ""
      rollSound ""
      slideSound ""
    }}
  ]
}}
DEF VIEW Viewpoint {{
  orientation 0 1 0 0.3
  position {f(x - 1.1 * math.cos(yaw))} {f(y - 1.1 * math.sin(yaw))} 0.62
  near 0.02
  ambientOcclusionRadius 0
  bloomThreshold -1
}}
Background {{
  skyColor [ 0.012 0.011 0.01 ]
  luminosity 1
}}
DEF FOG Fog {{
  color 0.02 0.018 0.015
  fogType "EXPONENTIAL"
  visibilityRange 16
}}
DEF ROVER MineRover {{
  translation {f(x)} {f(y)} {ROVER_Z}
  rotation 0 0 1 {f(yaw)}
}}
Robot {{
  name "mine_supervisor"
  controller "mine_supervisor"
  customData "{self.name}"
  supervisor TRUE
  children [
    Emitter {{ channel 1 }}
    Receiver {{ channel 2 }}
  ]
}}
"""
        path = os.path.join(ROOT, "worlds", f"{self.name}.wbt")
        with open(path, "w") as fh:
            fh.write(header + "\n".join(self.nodes) + "\n")
        gpath = os.path.join(ROOT, "config", "gas_sources", f"{self.name}.yaml")
        with open(gpath, "w") as fh:
            fh.write(f"# Gas sources for worlds/{self.name}.wbt (generated by tools/build_worlds.py).\n"
                     "# ppm at the rover = peak(t) * exp(-d^2 / (2 sigma^2)); peak(t) ramps 0 -> peak_ppm over\n"
                     "# ramp_up_s once the rover first comes within trigger_distance_m.\n")
            yaml.safe_dump({"sources": self.gas}, fh, sort_keys=False)
        print(f"wrote {path}  ({len(self.nodes)} nodes, {self.lights} lights, {len(self.gas)} gas)")


# =====================================================================================
# Layouts
# =====================================================================================
def corridor():
    w = World("corridor", "Test corridor")
    w.tunnel_x(0, 6)
    w.write()


def obstacles():
    w = World("obstacles", "Rubble obstacles")
    w.tunnel_x(0, 8)
    w.rubble(2.3, -0.30, 0.40, 0.4)
    w.rubble(4.1, 0.30, 0.40, 1.9)
    w.rubble(5.9, -0.27, 0.36, 2.7)
    w.write()


def pit_ahead():
    w = World("pit_ahead", "Full-width pit ahead")
    w.tunnel_x(0, 5, pit=(2.4, -HALF, 3.2, HALF))
    w.write()


def side_pit():
    w = World("side_pit", "Half-width pit")
    w.tunnel_x(0, 7, pit=(2.6, 0.0, 3.4, HALF))
    w.write()


def gas_ahead():
    w = World("gas_ahead", "Gas build-up ahead")
    w.tunnel_x(0, 7)
    w.gas_source(5.0, 0.0, peak=2500, sigma=1.0, ramp_up=5.0, trigger=3.5)
    w.write()


def ramp():
    w = World("ramp", "25 degree ramp")
    w.tunnel_x(0, 5.5)
    w.ramp(2.4, 0.0, 0.0, length=2.0, width=1.19)
    w.write()


def ramp_gentle():
    """A 10 degree hump (up, level, down): under the 20 degree tilt limit, so the rover
    drives over it and carries on."""
    w = World("ramp_gentle", "10 degree hump")
    w.tunnel_x(0, 8)
    a, length, top = 10.0, 1.0, 0.8
    run = length * math.cos(math.radians(a))
    rise = length * math.sin(math.radians(a))
    x0 = 2.4
    w.ramp(x0, 0.0, 0.0, length=length, width=1.19, angle_deg=a)
    w.box(x0 + run + top / 2, 0, rise - 0.03, top, 1.19, 0.06, "floor", "hump top", tscale=(top, 1.19))
    w.ramp(x0 + 2 * run + top, 0.0, math.pi, length=length, width=1.19, angle_deg=a)
    w.write()


def gas_return():
    """12 m tunnel with the gas source near the far end: far enough in for the link to drop,
    and room for a second gas pocket (added by the supervisor) on the way back."""
    w = World("gas_return", "Gas ahead, long tunnel")
    w.tunnel_x(0, 12)
    w.gas_source(10.0, 0.0, peak=2500, sigma=1.0, ramp_up=5.0, trigger=3.5)
    w.write()


def long_corridor():
    w = World("long_corridor", "Long test corridor")
    w.tunnel_x(0, 12)
    w.write()


def mine():
    """Main demo: 8.6 m tunnel A (+X), 90 deg left bend, 7 m tunnel B (+Y)."""
    w = World("mine", "Underground mine")
    xb0, xb1 = 8.0, 9.2          # tunnel B spans x in [xb0, xb1]
    yb1 = 7.0                    # far end of tunnel B
    bx0, bx1, by0 = 1.0, 2.0, -2.4   # side bay (south side of tunnel A) with the ramp
    pit = (5.9, 0.0, 6.7, HALF)  # left half of tunnel A

    # floors
    w.floor_with_hole(0, -HALF, xb1, HALF, *pit)
    w.floor(xb0, HALF, xb1, yb1)
    w.floor(bx0, by0, bx1, -HALF)
    # roofs
    w.roof(0, -HALF, xb1, HALF)
    w.roof(xb0, HALF, xb1, yb1)
    w.roof(bx0, by0, bx1, -HALF)
    # tunnel A walls
    w.wall(0, HALF, xb0, HALF, 0, -1)                 # north wall up to the bend
    w.wall(0, -HALF, bx0, -HALF, 0, 1)                # south wall, before bay
    w.wall(bx1, -HALF, xb1, -HALF, 0, 1)              # south wall, after bay
    w.entrance(0)
    # bend + tunnel B walls
    w.wall(xb1, -HALF, xb1, yb1, -1, 0)               # east wall (also end wall of A)
    w.wall(xb0, HALF, xb0, yb1, 1, 0)                 # west wall of B
    w.wall(xb0, yb1, xb1, yb1, 0, -1)                 # end of B
    # side bay walls + ramp rising towards -Y
    w.wall(bx0, -HALF, bx0, by0, 1, 0)
    w.wall(bx1, -HALF, bx1, by0, -1, 0)
    w.wall(bx0, by0, bx1, by0, 0, 1, ribs=False)
    w.ramp((bx0 + bx1) / 2, -HALF - 0.35, -math.pi / 2, length=1.5, width=0.98)
    # dressing
    w.cable(2.2, HALF, xb0, HALF, 0, -1)
    w.cable(xb1, 0.8, xb1, yb1, -1, 0)
    for x in (2.6, 4.9, 7.5):
        w.timber_x(x)
    for y in (1.6, 4.0, 6.2):
        w.timber_y(y, xb0, xb1)
    w.lamp(2.9, -HALF + 0.12)
    w.lamp(7.3, -HALF + 0.12)
    w.lamp(xb1 - 0.12, 2.6)
    w.lamp(1.5, -1.9, intensity=0.3)
    # hazards
    w.rubble(3.1, -0.30, 0.40, 0.4)
    w.rubble(4.6, 0.30, 0.40, 1.9)
    w.rubble(8.33, 2.2, 0.36, 2.7)
    w.gas_source(8.6, 5.4, peak=2500, sigma=1.0, ramp_up=6.0, trigger=4.0)
    w.write()


if __name__ == "__main__":
    for fn in (corridor, obstacles, pit_ahead, side_pit, gas_ahead, ramp, ramp_gentle, gas_return, long_corridor, mine):
        fn()
