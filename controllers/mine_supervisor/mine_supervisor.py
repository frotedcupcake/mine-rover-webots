"""TEMPORARY probe supervisor (replaced by the real one)."""
import math
import os
from controller import Supervisor

s = Supervisor()
ts = int(s.getBasicTimeStep())
rover = s.getFromDef("ROVER")
tr = rover.getField("translation")
rot = rover.getField("rotation")
mode = os.environ.get("PROBE", "static")
poses = [(1.5, 0, 0), (1.5, 0, 10), (1.5, 0, 20), (1.5, 0, 30), (1.5, 0, 45), (1.5, 0, 60), (1.5, 0, 90),
         (1.5, 0.2, 0), (1.5, 0.3, 0), (1.5, 0.4, 0), (1.5, -0.3, 0), (5.2, 0, 0), (5.5, 0, 0), (5.5, 0, 15),
         (5.5, 0.2, 0), (5.6, 0, 0)]
if os.environ.get("POSES"):
    poses = eval(os.environ["POSES"])
i = -1
tend = float(os.environ.get("TEND", "30"))
while s.step(ts) != -1:
    t = s.getTime()
    if mode == "static":
        j = int(t // 0.5)
        if j != i:
            i = j
            if i >= len(poses):
                s.simulationQuit(0)
                break
            x, y, yaw = poses[i]
            tr.setSFVec3f([x, y, 0.068])
            rot.setSFRotation([0, 0, 1, math.radians(yaw)])
            rover.resetPhysics()
            print(f"S t={t:.2f} pose x={x} y={y} yaw={yaw}", flush=True)
    else:
        if int(t * 2) != i:
            i = int(t * 2)
            p = rover.getPosition()
            v = rover.getVelocity()
            print(f"S t={t:.2f} pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) v=({v[0]:.2f},{v[1]:.2f}) wz={v[5]:.2f}", flush=True)
        if t > tend:
            s.simulationQuit(0)
            break
