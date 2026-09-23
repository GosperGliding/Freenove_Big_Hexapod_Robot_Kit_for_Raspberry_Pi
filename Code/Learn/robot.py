"""Hardware access for the learning experiments, with the safety around it.

Deliberately thin. Everything here is either talking to a device or stopping
the robot hurting itself; the task and the search live elsewhere.

Attitude comes from the raw accelerometer rather than imu.py's fused estimate.
For a *static* pose that is not a compromise, it is the better instrument:
gravity alone fixes orientation when nothing is moving, so there is no
integration, no drift, and no filter to be wrong. It also sidesteps two
defects in the vendor's version -- imu.py returns (pitch, roll, yaw) while
control.py unpacks it as (roll, pitch, yaw), and the quaternion z update
multiplies by quaternion_x where the pattern requires gyro_x.
"""

import math
import sys
import time

# The 18 leg joints, in leg order, as flat 0..31 servo channels. Taken from
# Control.set_leg_angles -- note leg 3's tibia is on the other PCA9685.
JOINT_CHANNELS = [
    15, 14, 13,   # leg 1: coxa, femur, tibia
    12, 11, 10,   # leg 2
    9,  8,  31,   # leg 3
    22, 23, 27,   # leg 4
    19, 20, 21,   # leg 5
    16, 17, 18,   # leg 6
]

JOINT_COUNT = len(JOINT_CHANNELS)

# Servo travel we allow the search to use. The full range is 0..180, but the
# last few degrees at each end fold a leg into the chassis, where it can jam
# against the body or a neighbour and stall. Stalled servos are how this
# hardware dies, so the search never sees those angles.
JOINT_MIN = 15.0
JOINT_MAX = 165.0

# Beyond this the robot is on its side, not standing. Trials abort rather than
# leaving it straining against the floor.
TIP_ABORT_DEGREES = 40.0

SERVO_POWER_PIN = 4  # active-high DISABLE: drive low to energise


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


class FakeBackend:
    """Runs the loop with no hardware, for developing off the robot.

    It does not simulate physics and cannot tell you whether a pose stands.
    Its only job is to let the harness, the logging and the search be
    exercised somewhere other than on a live robot.
    """

    def __init__(self):
        self.angles = [90.0] * JOINT_COUNT

    def set_angles(self, angles):
        self.angles = list(angles)

    def read_accel(self):
        return 0.0, 0.0, 9.8

    def read_gyro(self):
        return 0.0, 0.0, 0.0

    def relax(self):
        pass

    def close(self):
        pass


class HardwareBackend:
    """The real thing: PCA9685 servos, BCM 4 power rail, MPU6050."""

    def __init__(self):
        from gpiozero import OutputDevice
        from mpu6050 import mpu6050
        sys.path.insert(0, _server_dir())
        from servo import Servo

        self.power = OutputDevice(SERVO_POWER_PIN)
        self.power.off()          # low energises the rail
        self.servo = Servo()
        self.sensor = mpu6050(address=0x68, bus=1)
        self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G)
        self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)

    def set_angles(self, angles):
        for channel, angle in zip(JOINT_CHANNELS, angles):
            self.servo.set_servo_angle(channel, int(round(angle)))

    def read_accel(self):
        data = self.sensor.get_accel_data()
        return data["x"], data["y"], data["z"]

    def read_gyro(self):
        data = self.sensor.get_gyro_data()
        return data["x"], data["y"], data["z"]

    def relax(self):
        self.servo.relax()

    def close(self):
        self.relax()
        self.power.on()           # high disables the rail


def _server_dir():
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "Server"))


def reference_angle(channel):
    """The angle servo.py holds a channel at while the legs are fitted.

    This is the mechanical zero the whole robot is referenced to. Used as the
    pose to return to between trials, not as a starting point for the search --
    nothing about it is a standing pose.
    """
    if channel in (10, 13, 31):
        return 10.0
    if channel in (18, 21, 27):
        return 170.0
    return 90.0


REFERENCE_POSE = [reference_angle(c) for c in JOINT_CHANNELS]


class Robot:
    """Context manager around one backend, with the safety rules applied."""

    def __init__(self, fake=False):
        self.backend = FakeBackend() if fake else HardwareBackend()
        self.fake = fake

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def set_pose(self, angles):
        """Command 18 joint angles, clamped into the safe travel range."""
        if len(angles) != JOINT_COUNT:
            raise ValueError("expected %d angles, got %d" % (JOINT_COUNT, len(angles)))
        safe = [clamp(float(a), JOINT_MIN, JOINT_MAX) for a in angles]
        self.backend.set_angles(safe)
        return safe

    def attitude(self):
        """Roll and pitch in degrees, from gravity alone.

        Only meaningful when the robot is still -- any real acceleration adds
        to gravity and tilts the apparent vertical. That is why a trial settles
        before measuring, and why stillness is part of the score.
        """
        ax, ay, az = self.backend.read_accel()
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        return roll, pitch

    def motion(self):
        """Magnitude of the angular rate, deg/s. Zero when settled."""
        gx, gy, gz = self.backend.read_gyro()
        return math.sqrt(gx * gx + gy * gy + gz * gz)

    def has_tipped(self):
        roll, pitch = self.attitude()
        return abs(roll) > TIP_ABORT_DEGREES or abs(pitch) > TIP_ABORT_DEGREES

    def go_reference(self, settle=1.0):
        """Return to the assembly reference pose and let it settle."""
        self.set_pose(REFERENCE_POSE)
        time.sleep(settle)

    def relax(self):
        self.backend.relax()

    def close(self):
        self.backend.close()
