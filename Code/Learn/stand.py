"""The standing task: three joint angles in, one number out.

The symmetric first stage
-------------------------
A candidate is three angles, each an offset from the legs-straight pose:

  hip     swings the leg along the body
  knee    raises (negative) or lowers (positive) the thigh
  ankle   bends the shin down from straight

Every leg gets the same three. The hip is mirrored on the left side, because
the legs there are a reflection of the right: the same offset on both sides
would swing all six legs the same way round, which twists the body. Knee and
ankle need no flip here; Control's servo mapping already inverts them for the
mirrored legs.

The hand-written stance at `hexapod_walk --height 40` is (0, -15.5, +84.9) in
this space, so the score it has to beat sits inside the search.

What standing means numerically
-------------------------------
  height   how far below the hips the feet are, from forward kinematics of
           the commanded angles. Pushes the pose upward.
  level    tilt of the settled pose against the robot lying straight-legged.
  still    rotation rate while settled -- whether the tilt reading is valid.

Height is commanded, not measured, and on its own that is a hole: a pose the
servos cannot hold sags until the body rests on the floor, and because every
leg does the same thing it sags evenly, so it still reads level and still
while height reports the full commanded value.

So height only counts once the legs are shown to be carrying the body. In the
settled pose the front pair of knees is lowered and the rear pair raised by a
few degrees, which should pitch the body; forward kinematics says by how much
if the legs are rigid and the feet stay planted. The accelerometer measures
how far it actually pitched. A body on its legs follows; one resting on its
belly barely moves, because the floor is holding it, not the legs.
"""

from robot import (STRAIGHT, TIP_ABORT_DEGREES, angle_between, floor_normal,
                   leg_geometry, tilt)

# Search box, degrees from straight
BOUNDS = [
    (-20.0, 20.0),     # hip
    (-60.0, 45.0),     # knee
    (0.0, 140.0),      # ankle
]
PARAMETER_NAMES = ["hip", "knee", "ankle"]

# The hand-written stance at hexapod_walk --height 40
HAND_STANCE = [0.0, -15.5, 84.9]

# The support check: knees of the front pair (legs 1, 6) down and the rear
# pair (legs 3, 4) up by this much, and at least this share of the predicted
# pitch must appear. Both are judgements; support_ratio is logged so real
# trials can set them.
SUPPORT_KNEE_DEGREES = 6.0
SUPPORT_MIN_RATIO = 0.5
SUPPORT_MIN_PREDICTED_DEGREES = 1.5
FRONT = (0, 5)
REAR = (2, 3)

# Reward weights, in mm per unit of each measurement, so each weighted term
# is in mm and they add. A 10 degree lean costs 30 mm of height; 20 deg/s of
# wobble costs 10 mm.
LEVEL_COST_PER_DEGREE = 3.0
MOTION_COST_PER_DEG_PER_SEC = 0.5

# Below anything that stood: infeasible (nothing moved) < tipped < unsupported
INFEASIBLE_REWARD = -300.0
TIPPED_REWARD = -200.0
UNSUPPORTED_REWARD = -150.0

RAMP_FRAMES = 40
TILT_FRAMES = 10
SETTLE_SECONDS = 1.0
TILT_SETTLE_SECONDS = 0.4
MEASURE_SECONDS = 0.6
MEASURE_SAMPLES = 10


def joints(candidate):
    """Six legs' (hip, knee, ankle) from one shared set of offsets."""
    hip, knee, ankle = candidate
    right = (STRAIGHT[0] + hip, STRAIGHT[1] + knee, STRAIGHT[2] + ankle)
    left = (STRAIGHT[0] - hip, STRAIGHT[1] + knee, STRAIGHT[2] + ankle)
    return [right] * 3 + [left] * 3


def pitched(pose):
    tilted = [list(j) for j in pose]
    for leg in FRONT:
        tilted[leg][1] += SUPPORT_KNEE_DEGREES
    for leg in REAR:
        tilted[leg][1] -= SUPPORT_KNEE_DEGREES
    return [tuple(j) for j in tilted]


def height(pose):
    """The body height the lowest-reaching foot allows, mm."""
    return min(-leg_geometry(j)[2] for j in pose)


def infeasible(robot, pose):
    """Why this pose cannot be tried, checked before anything moves."""
    straight = [STRAIGHT] * 6
    problem = robot.path_violation(pose, RAMP_FRAMES, start=straight)
    if problem:
        return problem
    problem = robot.path_violation(pitched(pose), TILT_FRAMES, start=pose)
    if problem:
        return "support check: " + problem
    if predicted_pitch(pose) < SUPPORT_MIN_PREDICTED_DEGREES:
        # The knee change barely moves the feet vertically here, so whether
        # the body follows cannot be told from sensor noise
        return "support check would not show, %.2f degrees" % predicted_pitch(pose)
    return None


def predicted_pitch(pose):
    return angle_between(floor_normal(pose), floor_normal(pitched(pose)))


def evaluate(robot, candidate, level_reference):
    """Run one trial. Returns (reward, detail dict).

    level_reference is the gravity vector measured with the robot lying
    straight-legged, so an IMU mounted slightly off square does not read as a
    lean.
    """
    pose = joints(candidate)
    problem = infeasible(robot, pose)
    if problem:
        return INFEASIBLE_REWARD, {"infeasible": problem}

    predicted = predicted_pitch(pose)
    detail = {"height_mm": round(height(pose), 1)}
    try:
        robot.ramp(pose, RAMP_FRAMES)
        robot.wait(SETTLE_SECONDS)
        base, motion = robot.sample(MEASURE_SECONDS, MEASURE_SAMPLES)
        level = angle_between(base, level_reference)
        detail.update(level_deg=round(level, 2), motion_dps=round(motion, 2))
        if tilt(base) > TIP_ABORT_DEGREES:
            detail["tipped"] = "settling"
            return TIPPED_REWARD, detail

        robot.ramp(pitched(pose), TILT_FRAMES)
        robot.wait(TILT_SETTLE_SECONDS)
        vector, _ = robot.sample(MEASURE_SECONDS / 2, MEASURE_SAMPLES // 2)
        robot.ramp(pose, TILT_FRAMES)
        followed = angle_between(vector, base) / predicted
        detail.update(support_predicted_deg=round(predicted, 2),
                      support_ratio=round(followed, 2))
    finally:
        robot.ramp([STRAIGHT] * 6, RAMP_FRAMES)

    if followed < SUPPORT_MIN_RATIO:
        detail["unsupported"] = True
        return UNSUPPORTED_REWARD, detail

    reward = (detail["height_mm"]
              - LEVEL_COST_PER_DEGREE * level
              - MOTION_COST_PER_DEG_PER_SEC * motion)
    return reward, detail
