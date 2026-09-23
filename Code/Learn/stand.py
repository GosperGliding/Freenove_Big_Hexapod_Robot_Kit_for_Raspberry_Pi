"""The standing task: a set of 18 joint angles in, one number out.

What "standing" means numerically
---------------------------------
Three things at once, and the interesting part is that no one of them is
enough on its own:

  height    how far the body sits above its feet. Computed from the commanded
            angles by forward kinematics -- it is what we are *asking* for.
  level     measured from gravity. It is what tells us the robot actually
            achieved the pose rather than folding under it.
  still     measured from the gyro. A pose caught mid-collapse can read level
            for an instant; one that is level and motionless is holding.

Maximise height alone and it commands a tall pose it cannot hold. Maximise
level and still alone and it lies flat on the floor, which is beautifully
stable and not standing. The product of the three is what has a standing pose
at its optimum, and nothing in it describes what a standing pose looks like.

Nothing here tells the robot that legs come in pairs, that six of them should
cooperate, or which joint bends which way. All of that has to be discovered.
"""

import math
import time

from robot import JOINT_COUNT, JOINT_MIN, JOINT_MAX

# Link lengths, mm. Same as the IK.
COXA = 33.0
FEMUR = 90.0
TIBIA = 110.0

# Reward weights, in mm per unit so every term is in millimetres and they can
# simply be added. A 10 degree lean costs 30 mm of height; 20 deg/s of wobble
# costs 10 mm. Both were chosen so that a pose has to be clearly better on one
# axis to be worth giving up ground on another.
TILT_COST_PER_DEGREE = 3.0
MOTION_COST_PER_DEG_PER_SEC = 0.5

# A trial that ends on its side scores worse than any upright pose, however
# bad, so the search is never tempted to explore in that direction.
TIPPED_REWARD = -200.0

SETTLE_SECONDS = 1.2
MEASURE_SECONDS = 0.8
MEASURE_SAMPLES = 12


def servo_to_ik_angles(leg_index, coxa, femur, tibia):
    """Undo the mapping Control.set_leg_angles applies on the way out.

    Legs 0-2 and 3-5 are mirrored, and not by a clean sign flip: the femur
    picks up 90 - b on one side and 90 + b on the other, and the tibia is
    inverted through 180 on the far side only. Assumes zero calibration
    offsets, which is the case while point.txt holds its nominal values.
    """
    if leg_index < 3:
        return coxa, 90.0 - femur, tibia
    return coxa, femur - 90.0, 180.0 - tibia


def foot_drop(leg_index, coxa, femur, tibia):
    """How far below the coxa pivot this leg puts its foot, in mm.

    The forward kinematics of Control.angle_to_coordinate. Its first component
    is the one we want: the IK frame is shuffled relative to the leg frame
    (the IK is called as coordinate_to_angle(-z, x, y)), so the IK's x axis is
    the leg frame's negative z -- which is to say, downward.
    """
    a, b, c = servo_to_ik_angles(leg_index, coxa, femur, tibia)
    a = math.radians(a)
    b = math.radians(b)
    c = math.radians(c)
    return TIBIA * math.sin(b + c) + FEMUR * math.sin(b)


def pose_height(angles):
    """The height a level body could hold on these legs.

    The *minimum* drop across the six legs, not the mean or the maximum. A leg
    that reaches further down than the others does not lift the body, it just
    becomes the only one touching -- so the height the robot can actually be
    supported at is set by its shortest reach. Taking the minimum makes uneven
    poses score badly without needing a separate evenness term.
    """
    drops = []
    for leg in range(6):
        coxa, femur, tibia = angles[leg * 3:leg * 3 + 3]
        drops.append(foot_drop(leg, coxa, femur, tibia))
    return min(drops)


def random_pose(rng):
    return [rng.uniform(JOINT_MIN, JOINT_MAX) for _ in range(JOINT_COUNT)]


def evaluate(robot, angles, verbose=False):
    """Run one trial. Returns (reward, detail dict)."""
    applied = robot.set_pose(angles)
    time.sleep(SETTLE_SECONDS)

    rolls = []
    pitches = []
    motions = []
    interval = MEASURE_SECONDS / MEASURE_SAMPLES
    for _ in range(MEASURE_SAMPLES):
        roll, pitch = robot.attitude()
        rolls.append(roll)
        pitches.append(pitch)
        motions.append(robot.motion())
        time.sleep(interval)

    mean_roll = sum(rolls) / len(rolls)
    mean_pitch = sum(pitches) / len(pitches)
    tilt = math.sqrt(mean_roll ** 2 + mean_pitch ** 2)
    motion = sum(motions) / len(motions)
    height = pose_height(applied)

    tipped = tilt > 40.0
    if tipped:
        reward = TIPPED_REWARD
    else:
        reward = (height
                  - TILT_COST_PER_DEGREE * tilt
                  - MOTION_COST_PER_DEG_PER_SEC * motion)

    detail = {
        "height_mm": round(height, 1),
        "tilt_deg": round(tilt, 2),
        "roll_deg": round(mean_roll, 2),
        "pitch_deg": round(mean_pitch, 2),
        "motion_dps": round(motion, 2),
        "tipped": tipped,
    }
    if verbose:
        print("    height %6.1f  tilt %5.2f  motion %6.2f  -> %7.1f"
              % (height, tilt, motion, reward))
    return reward, detail
