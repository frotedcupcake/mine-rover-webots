"""TEMPORARY sensor probe (replaced by the real controller)."""
import os
import sys
from controller import Robot

r = Robot()
ts = int(r.getBasicTimeStep())
names = ["us_left", "us_right", "ir_front_left", "ir_front_right", "ir_down_left", "ir_down_right"]
ds = {n: r.getDevice(n) for n in names}
for d in ds.values():
    d.enable(ts * 2)
acc = r.getDevice("accelerometer"); acc.enable(ts)
gyro = r.getDevice("gyro"); gyro.enable(ts)
imu = r.getDevice("inertial_unit"); imu.enable(ts)
wheels = [r.getDevice("wheel_" + n) for n in ("fl", "rl", "fr", "rr")]
for w in wheels:
    w.setPosition(float("inf")); w.setVelocity(0)
print("PY", sys.version.split()[0], "probe mode", os.environ.get("PROBE", "static"), flush=True)
mode = os.environ.get("PROBE", "static")
k = 0
amax = 0
while r.step(ts) != -1:
    k += 1
    t = r.getTime()
    a = acc.getValues()
    mag = (a[0] ** 2 + a[1] ** 2 + a[2] ** 2) ** 0.5 / 9.81
    amax = max(amax, abs(mag - 1))
    if mode == "drive":
        # fwd 3 s, stop 1 s, spin 3 s, stop, arc 3 s
        ph = t % 12
        if ph < 3: l, rr = 0.7, 0.7
        elif ph < 4: l, rr = 0, 0
        elif ph < 7: l, rr = -0.55, 0.55
        elif ph < 8: l, rr = 0, 0
        elif ph < 11: l, rr = 0.4, 1.0
        else: l, rr = 0, 0
        mw = wheels[0].getMaxVelocity()
        wheels[0].setVelocity(l * mw); wheels[1].setVelocity(l * mw)
        wheels[2].setVelocity(rr * mw); wheels[3].setVelocity(rr * mw)
    if k % 50 == 0:
        v = {n: round(d.getValue(), 3) for n, d in ds.items()}
        rpy = [round(x * 57.3, 1) for x in imu.getRollPitchYaw()]
        if mode == "static" and k % 100 == 0:
            print(f"U {v['us_left']} {v['us_right']} {v['ir_front_left']} {v['ir_front_right']}", flush=True)
        print(f"R t={t:.2f} us=({v['us_left']},{v['us_right']}) irf=({v['ir_front_left']},{v['ir_front_right']}) "
              f"ird=({v['ir_down_left']},{v['ir_down_right']}) rpy={rpy} acc={[round(x,2) for x in a]} "
              f"gyro={[round(x,2) for x in gyro.getValues()]} amax={amax:.2f}", flush=True)
        amax = 0
