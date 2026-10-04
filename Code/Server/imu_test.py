# -*- coding: utf-8 -*-
# Hardware-free checks on imu.py's attitude integration. Run from this
# directory:
#     python imu_test.py
#
# The sensor is stubbed, the Kalman filters are bypassed and the accelerometer
# correction is switched off, which leaves update_imu_state as a pure
# integrator of the gyro rate. A constant body rate from the identity then has
# an exact answer -- a rotation about a fixed axis -- to compare against.

import math
import sys
import types


class _Sensor:
    ACCEL_RANGE_2G = 0x00
    GYRO_RANGE_250DEG = 0x00

    # Zero while IMU() averages its offsets, so nothing is subtracted later
    gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def __init__(self, address=0x68, bus=1):
        pass

    def set_accel_range(self, value):
        pass

    def set_gyro_range(self, value):
        pass

    def get_accel_data(self):
        return {'x': 0.0, 'y': 0.0, 'z': 9.8}

    def get_gyro_data(self):
        return dict(_Sensor.gyro)


_stub = types.ModuleType("mpu6050")
_stub.mpu6050 = _Sensor
sys.modules["mpu6050"] = _stub

from imu import IMU  # noqa: E402


class _Passthrough:
    def kalman(self, value):
        return value


def integrate(rate, steps):
    # Drive a fresh IMU at a constant body rate for a number of updates
    imu = IMU()
    imu.proportional_gain = 0
    imu.integral_gain = 0
    for name in ("AX", "AY", "AZ", "GX", "GY", "GZ"):
        setattr(imu, "kalman_filter_" + name, _Passthrough())
    _Sensor.gyro = {'x': rate[0], 'y': rate[1], 'z': rate[2]}
    try:
        for _ in range(steps):
            angles = imu.update_imu_state()
    finally:
        _Sensor.gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    quaternion = (imu.quaternion_w, imu.quaternion_x, imu.quaternion_y, imu.quaternion_z)
    return quaternion, angles, imu.half_time_step


def exact_rotation(rate, elapsed):
    # Constant body rate from the identity: q = exp(rate * elapsed / 2)
    speed = math.sqrt(sum(r * r for r in rate))
    half = speed * elapsed / 2
    return (math.cos(half),) + tuple(math.sin(half) * r / speed for r in rate)


def test_integrates_a_rotation_about_every_axis():
    # All three rates nonzero, so every cross term in the update is exercised
    rate = (0.4, -0.3, 0.5)
    steps = 2000
    quaternion, _, half_step = integrate(rate, steps)
    expected = exact_rotation(rate, steps * 2 * half_step)
    error = max(abs(a - b) for a, b in zip(quaternion, expected))
    assert error < 1e-3, "quaternion off by %.4f: got %s, expected %s" % (
        error, quaternion, expected)


def test_returns_roll_then_pitch():
    # A pure rotation about x is roll; it must come back first
    steps = 500
    _, angles, half_step = integrate((0.5, 0.0, 0.0), steps)
    expected = math.degrees(0.5 * steps * 2 * half_step)
    roll, pitch, _ = angles
    assert abs(roll - expected) < 0.5, "roll %.2f, expected %.2f" % (roll, expected)
    assert abs(pitch) < 0.5, "pitch %.2f, expected 0" % pitch


if __name__ == '__main__':
    failures = 0
    for name, test in sorted(globals().items()):
        if name.startswith("test_"):
            try:
                test()
                print("PASS " + name)
            except AssertionError as error:
                failures += 1
                print("FAIL %s: %s" % (name, error))
    sys.exit(1 if failures else 0)
