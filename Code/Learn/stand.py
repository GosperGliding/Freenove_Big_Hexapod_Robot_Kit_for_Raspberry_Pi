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

Two modes: measured only, or with the model
-------------------------------------------
By default a trial is scored from measurements alone. Nothing about the robot
is assumed beyond the safety cage -- the servo travel, the fold check, the
search box and the tip abort, which stop it hurting itself and say nothing
about where standing is. Standing means:

  supported  the knees of the front pair go down and the rear pair up, and
             the body measurably pitches. One resting on its belly barely
             moves, because the floor is holding it, not the legs.
  level      tilt of the settled pose against the robot lying straight-legged.
  still      rotation rate while settled.

Nothing on the robot measures height, so in this mode taller is not rewarded:
every supported pose earns the same base score, less its lean and wobble.

With use_model, the robot's geometry joins in -- forward kinematics from the
link lengths, which is what the simulator runs too:

  height     how far below the hips the feet are commanded to be, added to
             the reward, so taller wins.
  support    measured pitch compared against the pitch forward kinematics
             predicts for rigid legs, rather than against a fixed threshold;
             poses where the prediction is too small to see are skipped.

The hand-written stance at `hexapod_walk --height 40`, (0, -15.5, +84.9) in
this space, is only used as a baseline in that mode.
"""

from robot import (STRAIGHT, TIP_ABORT_DEGREES, angle_between, floor_normal,
                   leg_geometry, tilt)

# Search box, degrees from straight. Part of the safety cage: the hip range is
# what keeps the front and rear pairs from swinging into each other.
BOUNDS = [
    (-20.0, 20.0),     # hip
    (-60.0, 45.0),     # knee
    (0.0, 140.0),      # ankle
]
PARAMETER_NAMES = ["hip", "knee", "ankle"]

# The hand-written stance at hexapod_walk --height 40; model mode only
HAND_STANCE = [0.0, -15.5, 84.9]

# The support check: knees of the front pair (legs 1, 6) down and the rear
# pair (legs 3, 4) up by this much.
SUPPORT_KNEE_DEGREES = 6.0
FRONT = (0, 5)
REAR = (2, 3)
# Measured only: the body must pitch at least this far. A judgement, set well
# above the accelerometer's noise; support_measured_deg is logged to set it.
SUPPORT_MIN_MEASURED_DEGREES = 1.0
# With the model: this share of the predicted pitch must appear, and poses
# predicted to pitch less than the minimum are skipped as unreadable.
SUPPORT_MIN_RATIO = 0.5
SUPPORT_MIN_PREDICTED_DEGREES = 1.5

# Reward weights. With the model they are mm per unit, so each term is in mm
# alongside height; measured only, they are points against STOOD_REWARD.
LEVEL_COST_PER_DEGREE = 3.0
MOTION_COST_PER_DEG_PER_SEC = 0.5
STOOD_REWARD = 100.0

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
    """The body height the lowest-reaching foot allows, mm. Model mode only."""
    return min(-leg_geometry(j)[2] for j in pose)


def predicted_pitch(pose):
    return angle_between(floor_normal(pose), floor_normal(pitched(pose)))


def infeasible(robot, pose, use_model=False):
    """Why this pose cannot be tried, checked before anything moves."""
    straight = [STRAIGHT] * 6
    problem = robot.path_violation(pose, RAMP_FRAMES, start=straight)
    if problem:
        return problem
    problem = robot.path_violation(pitched(pose), TILT_FRAMES, start=pose)
    if problem:
        return "support check: " + problem
    if use_model and predicted_pitch(pose) < SUPPORT_MIN_PREDICTED_DEGREES:
        # The knee change barely moves the feet vertically here, so whether
        # the body follows cannot be told from sensor noise
        return "support check would not show, %.2f degrees" % predicted_pitch(pose)
    return None


def evaluate(robot, candidate, level_reference, use_model=False):
    """Run one trial. Returns (reward, detail dict).

    level_reference is the gravity vector measured with the robot lying
    straight-legged, so an IMU mounted slightly off square does not read as a
    lean.
    """
    pose = joints(candidate)
    problem = infeasible(robot, pose, use_model)
    if problem:
        return INFEASIBLE_REWARD, {"infeasible": problem}

    detail = {}
    if use_model:
        detail["height_mm"] = round(height(pose), 1)
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
        measured = angle_between(vector, base)
        detail["support_measured_deg"] = round(measured, 2)
    finally:
        robot.ramp([STRAIGHT] * 6, RAMP_FRAMES)

    if use_model:
        predicted = predicted_pitch(pose)
        ratio = measured / predicted
        detail.update(support_predicted_deg=round(predicted, 2),
                      support_ratio=round(ratio, 2))
        supported = ratio >= SUPPORT_MIN_RATIO
    else:
        supported = measured >= SUPPORT_MIN_MEASURED_DEGREES
    if not supported:
        detail["unsupported"] = True
        return UNSUPPORTED_REWARD, detail

    base_score = detail["height_mm"] if use_model else STOOD_REWARD
    reward = (base_score
              - LEVEL_COST_PER_DEGREE * level
              - MOTION_COST_PER_DEG_PER_SEC * motion)
    return reward, detail
