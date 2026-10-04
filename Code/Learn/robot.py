"""Hardware access for the learning experiments, with the safety around it.

Everything here either talks to a device or stops the robot hurting itself;
the task and the search live elsewhere.

Poses are joint angles, written straight to the servos. There is no IK and no
foot coordinate anywhere on the way out: a pose is three angles per leg,
measured from the legs-straight pose servo.py holds while the horns are
fitted. That pose is the mechanical reference of the whole robot, so every
trial starts there and ends there.

The one place geometry appears is leg_geometry below, forward kinematics from
the link lengths. It turns angles into a foot position for the safety check
and the score, and nothing is ever commanded through it.

Attitude comes from the raw accelerometer rather than imu.py's fused estimate.
For a static pose that is the better instrument: gravity alone fixes
orientation when nothing is moving, so there is no integration, no drift, and
no filter to be wrong.
"""

import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SERVER = os.path.normpath(os.path.join(HERE, "..", "Server"))
MPU6050_SOURCE = os.path.normpath(os.path.join(HERE, "..", "Libs", "mpu6050"))

# Servo channels per leg, (hip, knee, ankle), from Control.set_leg_angles.
# Leg 3's ankle is on the other PCA9685.
CHANNELS = [(15, 14, 13), (12, 11, 10), (9, 8, 31),
            (22, 23, 27), (19, 20, 21), (16, 17, 18)]

# Joint angles of the legs-straight pose, in the IK's convention: hip 90, knee
# 0, ankle 10. Through the mapping below this is servo.py's 90 / 90 / 10 on
# the right and 90 / 90 / 170 on the left.
STRAIGHT = (90.0, 0.0, 10.0)

# Absolute servo travel. Anything outside is refused, never clamped -- clamping
# would quietly command a different pose from the one being scored.
SERVO_MIN = 5.0
SERVO_MAX = 175.0

# A foot closer than this to its hip pivot has folded under the chassis.
# Control.check_point_validity's lower bound.
REACH_MIN = 90.0

# Beyond this the robot is on its side, not standing.
TIP_ABORT_DEGREES = 40.0

# ADS7830 channels for the two battery packs, and the voltages below which
# server.py raises its low-battery alarm -- used here to stop a run before a
# sagging pack browns out the Pi mid-move.
BATTERY_CHANNELS = (0, 4)
BATTERY_MIN_VOLTS = (5.5, 6.0)

FRAME_SECONDS = 0.02

# How often a watched ramp checks the body against the path it should follow
WATCH_EVERY = 5

COXA, FEMUR, TIBIA = 33.0, 90.0, 110.0


class Impeded(Exception):
    """A watched ramp found the body off the path its command should give."""

# Each leg's mounting angle and hip offset, as Control.transform_coordinates
# applies them between the body frame and the leg frame
LEG_MOUNTS = [(54, 94), (0, 85), (-54, 94), (-126, 94), (180, 85), (126, 94)]


def servo_angles(leg, joints):
    """Servo angles for one leg from its (hip, knee, ankle) joint angles.

    Control.set_leg_angles' mapping, without calibration offsets: the straight
    pose these angles are measured from bypasses point.txt too. Legs 4-6 are
    mounted mirrored, so their knee and ankle servos run the other way.
    """
    hip, knee, ankle = joints
    if leg < 3:
        return hip, 90.0 - knee, ankle
    return hip, 90.0 + knee, 180.0 - ankle


def leg_geometry(joints):
    """Foot position in the leg frame, mm: x out, y along, z up.

    Control.angle_to_coordinate without its rounding, reordered from the IK's
    axes into the leg frame's (the IK is called as coordinate_to_angle(-z, x, y)).
    """
    hip, knee, ankle = (math.radians(j) for j in joints)
    down = TIBIA * math.sin(knee + ankle) + FEMUR * math.sin(knee)
    out = TIBIA * math.cos(knee + ankle) + FEMUR * math.cos(knee) + COXA
    return [out * math.sin(hip), out * math.cos(hip), -down]


def body_position(leg, foot):
    """A leg-frame foot position in the body frame: the inverse of
    Control.transform_coordinates for one leg."""
    angle, offset = LEG_MOUNTS[leg]
    c = math.cos(math.radians(angle))
    s = math.sin(math.radians(angle))
    x, y, z = foot
    return [(x + offset) * c - y * s, (x + offset) * s + y * c, z + 14]


def floor_normal(pose):
    """Unit normal of the plane through the six feet, in the body frame.

    With rigid legs and every foot on flat ground, this is the direction
    gravity reads in. Used to predict how far a commanded change should tilt
    the body, and as the fake backend's accelerometer.
    """
    import numpy as np
    feet = np.array([body_position(leg, leg_geometry(j)) for leg, j in enumerate(pose)])
    design = np.column_stack([feet[:, 0], feet[:, 1], np.ones(6)])
    a, b, _ = np.linalg.lstsq(design, feet[:, 2], rcond=None)[0]
    normal = np.array([-a, -b, 1.0])
    return tuple(normal / np.linalg.norm(normal))


class FakeBackend:
    """Runs the loop with no hardware, for developing off the robot.

    Its accelerometer reads the floor plane under the commanded feet, so a
    commanded tilt is seen to be followed; there is no other physics. It
    cannot tell you whether a pose stands.
    """

    def __init__(self):
        self.pose = [STRAIGHT] * 6

    def write(self, pose):
        self.pose = [tuple(j) for j in pose]

    def read_accel(self):
        return tuple(9.8 * c for c in floor_normal(self.pose))

    def read_gyro(self):
        return 0.0, 0.0, 0.0

    def read_battery(self):
        return 7.4, 7.4

    def energise(self):
        pass

    def relax(self):
        pass

    def close(self):
        pass


class HardwareBackend:
    """PCA9685 servos, the BCM 4 servo rail, and the MPU6050."""

    def __init__(self):
        from gpiozero import OutputDevice
        # The IMU driver is loaded from the copy bundled in Code/Libs, not the
        # system install. setup.py install under setuptools 80+ leaves a hollow
        # egg on the Pi -- both the package's re-export and the class itself
        # are missing -- while the bundled copy arrives intact with git pull.
        sys.path.insert(0, MPU6050_SOURCE)
        from mpu6050 import mpu6050
        sys.path.insert(0, SERVER)
        from servo import Servo

        # The rail is held off until everything else is open, so a device
        # that fails to start does so with no power on the servos. BCM 4 is an
        # active-high disable, and gpiozero drives a new output low unless told
        # otherwise -- which would energise the rail on this very line.
        self.power = OutputDevice(4, initial_value=True)
        self.sensor = mpu6050(address=0x68, bus=1)
        self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G)
        self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)
        self.servo = Servo()
        from adc import ADC
        self.adc = ADC()

    def read_battery(self):
        # Both packs, as server.py reads them. Which one feeds the servos is
        # not documented, so both are kept and the caller decides.
        return tuple(self.adc.read_channel_voltage(c) for c in BATTERY_CHANNELS)

    def energise(self):
        self.power.off()          # low enables the rail

    def write(self, pose):
        for leg, joints in enumerate(pose):
            for channel, angle in zip(CHANNELS[leg], servo_angles(leg, joints)):
                self.servo.set_servo_angle(channel, angle)

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


class Robot:
    """Context manager around one backend, with the safety rules applied.

    Constructing it energises the rail and drives all 18 joints to the
    straight pose at once, from wherever they are. Support the body.
    """

    def __init__(self, fake=False):
        self.fake = fake
        self.backend = FakeBackend() if fake else HardwareBackend()
        self.pose = [STRAIGHT] * 6
        # Pulses first, then power, so the servos wake up already commanded
        # to straight rather than to whatever the PCA9685s last held
        self.backend.write(self.pose)
        self.backend.energise()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.backend.close()

    def wait(self, seconds):
        if not self.fake:
            time.sleep(seconds)

    def violation(self, pose):
        """Why a pose may not be commanded, or None if it may."""
        for leg, joints in enumerate(pose):
            for angle in servo_angles(leg, joints):
                if not SERVO_MIN <= angle <= SERVO_MAX:
                    return "leg %d servo at %.0f degrees" % (leg + 1, angle)
            foot = leg_geometry(joints)
            distance = math.sqrt(sum(c * c for c in foot))
            if distance < REACH_MIN:
                return "leg %d folded, foot %.0f mm from the hip" % (leg + 1, distance)
        return None

    def path(self, target, frames, start=None):
        """Linear interpolation in joint space from start (default: now) to target."""
        start = self.pose if start is None else start
        return [[tuple(s + (t - s) * step / frames for s, t in zip(start[leg], target[leg]))
                 for leg in range(6)]
                for step in range(1, frames + 1)]

    def path_violation(self, target, frames, start=None):
        # Foot reach is not convex in joint space, so both ends passing does
        # not make every frame between them pass
        for frame in self.path(target, frames, start):
            problem = self.violation(frame)
            if problem:
                return problem
        return None

    def ramp(self, target, frames=30, watch=None):
        """Move to target a little at a time, so every joint arrives together.

        Servos slew at roughly 0.1-0.2 s per 60 degrees, so a jump makes a joint
        with far to go arrive long after one with little, dragging the robot
        through poses nobody commanded. A few degrees per frame keeps every
        servo inside its slew rate. Every frame is checked before the first is
        sent.

        watch, if given, is called every WATCH_EVERY frames with the frame just
        sent and returns a reason to stop or None. On a reason the ramp halts
        where it is -- no further push against whatever is in the way -- and
        raises Impeded; self.pose is left at the last frame sent.
        """
        problem = self.path_violation(target, frames)
        if problem:
            raise ValueError("refusing to move: " + problem)
        for step, frame in enumerate(self.path(target, frames), 1):
            self.backend.write(frame)
            self.pose = frame
            self.wait(FRAME_SECONDS)
            if watch is not None and (step % WATCH_EVERY == 0 or step == frames):
                reason = watch(frame, step)
                if reason:
                    raise Impeded(reason)
        self.pose = [tuple(j) for j in target]

    def rotation_rate(self):
        """Magnitude of the angular rate, deg/s. Zero when settled."""
        gx, gy, gz = self.backend.read_gyro()
        return math.sqrt(gx * gx + gy * gy + gz * gz)

    def sample(self, seconds, count):
        """Mean gravity vector and mean rotation rate over a short window.

        The gravity reading is only an orientation while the robot is still --
        any real acceleration adds to it. The rotation rate is what says
        whether that precondition held.
        """
        vectors = []
        rates = []
        for _ in range(count):
            vectors.append(self.backend.read_accel())
            rates.append(self.rotation_rate())
            self.wait(seconds / count)
        mean = tuple(sum(v[i] for v in vectors) / count for i in range(3))
        return mean, sum(rates) / count

    def battery(self, count=10):
        """Mean voltage of each pack over a few readings.

        The ADS7830 is 8-bit through a 3:1 divider, so one step is about
        59 mV; averaging recovers a little of what a single reading rounds off.
        """
        readings = [self.backend.read_battery() for _ in range(count)]
        return tuple(sum(r[i] for r in readings) / count for i in range(len(readings[0])))

    def battery_low(self):
        volts = self.battery()
        return any(v < limit for v, limit in zip(volts, BATTERY_MIN_VOLTS)), volts

    def relax(self):
        self.backend.relax()

    def reenergise(self):
        self.backend.write(self.pose)


def tilt(vector):
    """Angle between a gravity reading and the body's vertical, degrees."""
    x, y, z = vector
    return math.degrees(math.atan2(math.sqrt(x * x + y * y), z))


def angle_between(a, b):
    """Angle between two directions, degrees."""
    dot = sum(p * q for p, q in zip(a, b))
    norms = math.sqrt(sum(p * p for p in a)) * math.sqrt(sum(q * q for q in b))
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / norms))))


def rotate_like(v, a, b):
    """Rotate v by the rotation that takes direction a onto direction b.

    Rodrigues' formula about the axis a x b. Used to carry a gravity reading
    through the turn the model predicts, to get the reading to expect.
    """
    na = math.sqrt(sum(c * c for c in a))
    nb = math.sqrt(sum(c * c for c in b))
    a = [c / na for c in a]
    b = [c / nb for c in b]
    axis = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
    s = math.sqrt(sum(c * c for c in axis))
    cos = sum(p * q for p, q in zip(a, b))
    if s < 1e-12:
        return list(v)
    k = [c / s for c in axis]
    dot = sum(p * q for p, q in zip(k, v))
    cross = [k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]]
    return [v[i] * cos + cross[i] * s + k[i] * dot * (1 - cos) for i in range(3)]
