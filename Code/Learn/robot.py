"""Hardware access for the learning experiments, with the safety around it.

Everything here either talks to a device or stops the robot hurting itself;
the task and the search live elsewhere.

Poses are leg-frame foot positions, driven through Code/Server's Control. That
reuses the IK, the point.txt calibration offsets, the mirrored servo mapping
and the channel map rather than copying any of them. What this module adds is
the checking Control does not do -- a real reach limit and a joint-angle limit,
applied to every frame before anything is written -- and the ramp.

Attitude comes from the raw accelerometer rather than imu.py's fused estimate.
For a static pose that is the better instrument: gravity alone fixes
orientation when nothing is moving, so there is no integration, no drift, and
no filter to be wrong.
"""

import math
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
SERVER = os.path.normpath(os.path.join(HERE, "..", "Server"))

# Reach window for a foot, measured from the coxa pivot in the leg frame. The
# lower bound is Control.check_point_validity's. Its upper bound of 248 mm is
# looser than the 233 mm the links can span, and the IK clamps silently past
# that, so the real limit here is the same 225 mm the circle dance probes with.
REACH_MIN = 90.0
REACH_MAX = 225.0

# Servo travel the search may use. The last degrees at each end fold a leg into
# the chassis, where it jams and stalls; stalled servos are how this hardware
# dies. A pose needing more travel is rejected, never clamped -- clamping would
# quietly command a different pose from the one being scored.
JOINT_MIN = 15
JOINT_MAX = 165

# Beyond this the robot is on its side, not standing.
TIP_ABORT_DEGREES = 40.0

# Control's own start-up pose: every foot out at coxa height, body resting on
# the floor. Every trial ramps out of this and back into it, so no trial
# depends on what ran before it, and none starts from a pose that already
# stands.
REST = [[140.0, 0.0, 0.0] for _ in range(6)]

FRAME_SECONDS = 0.02


def _install_fake_hardware():
    """Stub the device modules so Control constructs without a Raspberry Pi.

    Does not simulate physics. It exists so the loop, the logging and the
    search can be exercised somewhere other than on a live robot.
    """
    smbus = types.ModuleType("smbus")

    class SMBus:
        def __init__(self, bus):
            pass

        def write_byte_data(self, addr, reg, value):
            pass

        def read_byte_data(self, addr, reg):
            return 0

    smbus.SMBus = SMBus
    sys.modules["smbus"] = smbus

    gpiozero = types.ModuleType("gpiozero")

    class OutputDevice:
        def __init__(self, pin, *args, **kwargs):
            pass

        def on(self):
            pass

        def off(self):
            pass

    gpiozero.OutputDevice = OutputDevice
    sys.modules["gpiozero"] = gpiozero

    mpu = types.ModuleType("mpu6050")

    class mpu6050:
        ACCEL_RANGE_2G = 0x00
        GYRO_RANGE_250DEG = 0x00

        def __init__(self, address=0x68, bus=1):
            pass

        def set_accel_range(self, value):
            pass

        def set_gyro_range(self, value):
            pass

        def get_accel_data(self):
            return {"x": 0.0, "y": 0.0, "z": 9.8}

        def get_gyro_data(self):
            return {"x": 0.0, "y": 0.0, "z": 0.0}

    mpu.mpu6050 = mpu6050
    sys.modules["mpu6050"] = mpu


def _load_control(fake):
    if fake:
        _install_fake_hardware()
    sys.path.insert(0, SERVER)
    # Control reads point.txt relative to the working directory
    previous = os.getcwd()
    os.chdir(SERVER)
    try:
        from control import Control
        return Control()
    finally:
        os.chdir(previous)


def reach(position):
    return math.sqrt(sum(c * c for c in position))


class Robot:
    """Context manager around Control, with the safety rules applied.

    Constructing it is a full-authority move: Control calibrates and drives
    all 18 joints to the rest pose straight away. Support the body.
    """

    def __init__(self, fake=False):
        self.fake = fake
        self.control = _load_control(fake)
        self.positions = [list(p) for p in REST]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def wait(self, seconds):
        if not self.fake:
            time.sleep(seconds)

    def servo_angles(self, positions):
        """The 18 servo angles Control.set_leg_angles would write, in leg order.

        Mirrors its arithmetic, calibration offsets included, so a pose can be
        checked without anything being written.
        """
        control = self.control
        angles = []
        for leg, (x, y, z) in enumerate(positions):
            a, b, c = control.coordinate_to_angle(-z, x, y)
            offset = control.calibration_angles[leg]
            if leg < 3:
                angles += [a + offset[0], 90 - (b + offset[1]), c + offset[2]]
            else:
                angles += [a + offset[0], 90 + b + offset[1], 180 - (c + offset[2])]
        return angles

    def violation(self, positions):
        """Why a pose may not be commanded, or None if it may."""
        for leg, position in enumerate(positions):
            distance = reach(position)
            if not REACH_MIN <= distance <= REACH_MAX:
                return "leg %d reach %.0f mm" % (leg + 1, distance)
        for joint, angle in enumerate(self.servo_angles(positions)):
            if not JOINT_MIN <= angle <= JOINT_MAX:
                return "leg %d joint %d at %d degrees" % (joint // 3 + 1, joint % 3, angle)
        return None

    def path(self, target, frames, start=None):
        """Linear interpolation from start (default: the current pose) to target."""
        start = self.positions if start is None else start
        return [[[s + (t - s) * step / frames for s, t in zip(start[leg], target[leg])]
                 for leg in range(6)]
                for step in range(1, frames + 1)]

    def path_violation(self, target, frames, start=None):
        # The reach window is an annulus, so both ends being reachable does not
        # make the straight line between them reachable
        for frame in self.path(target, frames, start):
            problem = self.violation(frame)
            if problem:
                return problem
        return None

    def ramp(self, target, frames=30):
        """Move to target a little at a time, so every joint arrives together.

        Servos slew at roughly 0.1-0.2 s per 60 degrees, so a jump makes a joint
        with far to go arrive long after one with little, dragging the robot
        through poses nobody commanded. A degree or two per frame keeps every
        servo inside its slew rate. Every frame is checked before the first is
        sent.
        """
        problem = self.path_violation(target, frames)
        if problem:
            raise ValueError("refusing to move: " + problem)
        for frame in self.path(target, frames):
            self.control.leg_positions = frame
            self.control.set_leg_angles()
            self.wait(FRAME_SECONDS)
        self.positions = [list(p) for p in target]

    def gravity(self):
        data = self.control.imu.sensor.get_accel_data()
        return data["x"], data["y"], data["z"]

    def rotation_rate(self):
        """Magnitude of the angular rate, deg/s. Zero when settled."""
        data = self.control.imu.sensor.get_gyro_data()
        return math.sqrt(data["x"] ** 2 + data["y"] ** 2 + data["z"] ** 2)

    def sample(self, seconds, count):
        """Mean gravity vector and mean rotation rate over a short window.

        The gravity reading is only an orientation while the robot is still --
        any real acceleration adds to it. The rotation rate is what says
        whether that precondition held.
        """
        vectors = []
        rates = []
        for _ in range(count):
            vectors.append(self.gravity())
            rates.append(self.rotation_rate())
            self.wait(seconds / count)
        mean = tuple(sum(v[i] for v in vectors) / count for i in range(3))
        return mean, sum(rates) / count

    def relax(self):
        self.control.servo.relax()

    def close(self):
        self.relax()
        self.control.servo_power_disable.on()   # high disables the rail


def tilt(vector):
    """Angle between a gravity reading and the body's vertical, degrees."""
    x, y, z = vector
    return math.degrees(math.atan2(math.sqrt(x * x + y * y), z))


def angle_between(a, b):
    """Angle between two gravity readings, degrees."""
    dot = sum(p * q for p, q in zip(a, b))
    norms = math.sqrt(sum(p * p for p in a)) * math.sqrt(sum(q * q for q in b))
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / norms))))
