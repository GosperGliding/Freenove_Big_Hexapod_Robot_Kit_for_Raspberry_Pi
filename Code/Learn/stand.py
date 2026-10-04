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

Impeded motion
--------------
Nothing reads a joint back, so a leg dragging on a high-friction floor, a
foot that sticks, or a leg caught on something cannot be seen at the joint.
It shows in the body instead: a leg that cannot follow its command makes the
body move other than the command says it should. So every ramp is watched,
every few frames, against the orientation the body should have:

  measured only   a mirror-symmetric command should not turn the body at all,
                  which follows from the command, not from any model. The
                  support nudge is not watched: its size is unknown here.
  with the model  the turn forward kinematics predicts at that frame, which
                  also catches the body pitching less than it should during
                  the support nudge.

More than IMPEDED_DEGREES off and the ramp stops where it is, so nothing goes
on pushing, the robot ramps back to straight, and the trial scores as
impeded. search.py then skips poses close to it for the rest of the run: the
restricted region, grown from what the robot ran into rather than set in
advance, and so particular to the surface it is on.

The friction a pose would need if each leg pushed like a strut along the line
from foot to hip is logged beside it with the model, to see whether it
predicts which poses are impeded.
"""

import math

from robot import (STRAIGHT, TIP_ABORT_DEGREES, Impeded, angle_between,
                   floor_normal, leg_geometry, rotate_like, tilt)

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

# How far the body may stray from the orientation its command should give
# before a ramp counts as impeded. Above the accelerometer's noise and the
# apparent tilt a slow ramp's own acceleration adds; below the 40 degree tip.
IMPEDED_DEGREES = 8.0
WATCH_SECONDS = 0.03
WATCH_SAMPLES = 3

# Reward weights. With the model they are mm per unit, so each term is in mm
# alongside height; measured only, they are points against STOOD_REWARD.
LEVEL_COST_PER_DEGREE = 3.0
MOTION_COST_PER_DEG_PER_SEC = 0.5
STOOD_REWARD = 100.0

# Below anything that stood. Skipped or restricted (nothing moved) < impeded
# (stopped partway) < tipped < unsupported.
INFEASIBLE_REWARD = -300.0
IMPEDED_REWARD = -250.0
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


def friction_needed(pose):
    """Worst foot's horizontal-to-vertical ratio from hip to foot. Model mode only.

    The friction coefficient a foot would need if its leg pushed like a strut
    along that line. Servo-held legs are not struts, so this is a guess to be
    checked against which poses are actually impeded, not a fact.
    """
    worst = 0.0
    for joints_of_leg in pose:
        x, y, z = leg_geometry(joints_of_leg)
        if z >= 0:
            return None            # a foot at or above its hip pushes nothing
        worst = max(worst, math.hypot(x, y) / -z)
    return worst


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


def watcher(robot, name, use_model):
    """A ramp watch: the body's turn since the ramp began against the turn
    its command should give. Measured only, that is no turn at all."""
    start_gravity, _ = robot.sample(WATCH_SECONDS, WATCH_SAMPLES)
    start_pose = [tuple(j) for j in robot.pose]
    start_normal = floor_normal(start_pose) if use_model else None

    def watch(frame, step):
        gravity, _ = robot.sample(WATCH_SECONDS, WATCH_SAMPLES)
        expected = start_gravity
        if use_model:
            expected = rotate_like(start_gravity, start_normal, floor_normal(frame))
        off = angle_between(gravity, expected)
        if off > IMPEDED_DEGREES:
            return "%s, step %d: body %.1f degrees off its expected path" % (name, step, off)
        return None

    return watch


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
        needed = friction_needed(pose)
        detail.update(height_mm=round(height(pose), 1),
                      friction_needed=None if needed is None else round(needed, 2))
    try:
        robot.ramp(pose, RAMP_FRAMES, watcher(robot, "standing up", use_model))
        robot.wait(SETTLE_SECONDS)
        base, motion = robot.sample(MEASURE_SECONDS, MEASURE_SAMPLES)
        level = angle_between(base, level_reference)
        detail.update(level_deg=round(level, 2), motion_dps=round(motion, 2))
        if tilt(base) > TIP_ABORT_DEGREES:
            detail["tipped"] = "settling"
            return TIPPED_REWARD, detail

        # The nudge's size is only known with the model, so only then watched
        nudge_watch = watcher(robot, "support nudge", True) if use_model else None
        robot.ramp(pitched(pose), TILT_FRAMES, nudge_watch)
        robot.wait(TILT_SETTLE_SECONDS)
        vector, _ = robot.sample(MEASURE_SECONDS / 2, MEASURE_SAMPLES // 2)
        back_watch = watcher(robot, "nudge back", True) if use_model else None
        robot.ramp(pose, TILT_FRAMES, back_watch)
        measured = angle_between(vector, base)
        detail["support_measured_deg"] = round(measured, 2)
    except Impeded as stopped:
        detail["impeded"] = str(stopped)
        return IMPEDED_REWARD, detail
    finally:
        # From wherever it stopped. Not watched: if the way back is impeded
        # too, stopping halfway would leave the robot stranded mid-pose.
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
