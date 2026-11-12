"""Attitude estimate and impact detection from a 6-axis IMU (MPU6050 stand-in).

Body axes: X forward, Y left, Z up.  The accelerometer measures specific force, so it
reads (0, 0, +g) at rest on level ground.

Sign conventions (checked by tests/test_attitude.py):
    nose up      = +pitch     (pitch rate = -gyro_y)
    left side up = +roll      (roll rate  = +gyro_x)
    left turn    = +yaw       (yaw rate   = +gyro_z)
"""
import math
import random

G = 9.81


class AttitudeFilter:
    def __init__(self, alpha=0.98, bias_time=2.0, impact_g=0.9, refractory=0.5, accel_trust_g=0.15):
        self.alpha = alpha
        self.bias_time = bias_time
        self.impact_g = impact_g
        self.refractory = refractory
        self.accel_trust_g = accel_trust_g
        self.pitch = 0.0          # rad
        self.roll = 0.0
        self.yaw = 0.0            # rad, continuous
        self.bias = [0.0, 0.0, 0.0]
        self.calibrated = False
        self._t0 = None
        self._sum = [0.0, 0.0, 0.0]
        self._n = 0
        self._last_impact = -1e9
        self._impact_pending = False
        self.last_impact_g = 0.0

    # ---- outputs ----
    @property
    def pitch_deg(self):
        return math.degrees(self.pitch)

    @property
    def roll_deg(self):
        return math.degrees(self.roll)

    @property
    def yaw_deg(self):
        return math.degrees(self.yaw)

    def pop_impact(self) -> bool:
        """True once for each detected impact."""
        hit, self._impact_pending = self._impact_pending, False
        return hit

    # ---- update ----
    def update(self, t, dt, accel, gyro):
        ax, ay, az = accel
        if self._t0 is None:
            self._t0 = t
        amag = math.sqrt(ax * ax + ay * ay + az * az)
        pitch_acc = math.atan2(ax, math.sqrt(ay * ay + az * az))
        roll_acc = math.atan2(ay, az)

        if not self.calibrated:
            # sit still: learn the gyro bias, take tilt straight from gravity
            for i in range(3):
                self._sum[i] += gyro[i]
            self._n += 1
            self.pitch, self.roll = pitch_acc, roll_acc
            if t - self._t0 >= self.bias_time:
                self.bias = [s / self._n for s in self._sum]
                self.calibrated = True
            return

        gx, gy, gz = (gyro[i] - self.bias[i] for i in range(3))
        pitch_g = self.pitch - gy * dt
        roll_g = self.roll + gx * dt
        self.yaw += gz * dt

        dev_g = abs(amag / G - 1.0)
        if dev_g < self.accel_trust_g:
            a = self.alpha
            self.pitch = a * pitch_g + (1 - a) * pitch_acc
            self.roll = a * roll_g + (1 - a) * roll_acc
        else:
            # accelerating hard: gravity direction is not trustworthy, coast on the gyro
            self.pitch, self.roll = pitch_g, roll_g

        if dev_g > self.impact_g and t - self._last_impact >= self.refractory:
            self._last_impact = t
            self._impact_pending = True
            self.last_impact_g = dev_g


class ImuErrorModel:
    """Adds white noise and a constant gyro bias to ideal simulator IMU samples."""

    def __init__(self, accel_noise=0.05, gyro_noise=0.003, gyro_bias=0.01, seed=0):
        self.rng = random.Random(seed)
        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise
        self.bias = [self.rng.uniform(-gyro_bias, gyro_bias) for _ in range(3)]

    def accel(self, a):
        return [v + self.rng.gauss(0, self.accel_noise) for v in a]

    def gyro(self, g):
        return [v + b + self.rng.gauss(0, self.gyro_noise) for v, b in zip(g, self.bias)]
