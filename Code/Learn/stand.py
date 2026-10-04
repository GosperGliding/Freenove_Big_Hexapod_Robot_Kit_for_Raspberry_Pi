"""The standing task: three joint angles and a speed in, one number out.

The symmetric first stage
-------------------------
A candidate is three angles, each an offset from the legs-straight pose, and
how fast to move towards them:

  hip     swings the leg along the body
  knee    raises (negative) or lowers (positive) the thigh
  ankle   bends the shin down from straight
  speed   the fastest joint's speed, as a share of the cage's 250 deg/s

Every leg gets the same three angles. The hip is mirrored on the left side,
because the legs there are a reflection of the right: the same offset on both
sides would swing all six legs the same way round, which twists the body.
Knee and ankle need no flip here; Control's servo mapping already inverts
them for the mirrored legs.

A trial ends as soon as the robot stands
----------------------------------------
The robot moves from straight towards the candidate, and every 10 frames it
stops to check whether it is standing. The first check that passes ends the
trial there, short of the candidate if need be; if none does, the candidate
pose itself gets a last check. Standing means all three of:

  still      the gyro reads under 5 deg/s
  level      within 5 degrees of the robot lying straight-legged
  supported  the front knees go down and the rear knees up, and the body
             measurably pitches. One resting on its belly barely moves,
             because the floor is holding it, not the legs.

and, with use_model, the commanded height at least the target. Then

  reward = (base - 3*level - 0.5*still) * exp(-time / 2 s)

where base is the commanded height in mm of the frame that stood, with the
model, or 100 without it, since nothing then knows the height; and time runs
from the first frame to the start of the check that passed, counting the
pauses for any checks that failed before it. Lean and wobble both gate the
stand and still cost within it; time decays the whole. With the target in
force, the height earned is the target plus however far the last 10 frames
overshot it.

Two modes: measured only, or with the model
-------------------------------------------
By default nothing about the robot is assumed beyond the safety cage -- the
servo travel, the fold check, the search box, the joint-speed limit and the
tip abort, which stop it hurting itself and say nothing about where standing
is. Nothing on the robot measures height, so in this mode there is no target:
the trial ends at the first checkpoint that stands, however low.

With use_model, the robot's geometry joins in -- forward kinematics from the
link lengths, which is what the simulator runs too:

  height     the commanded height of each frame. Checks start only once it
             reaches the target, and candidates whose own pose falls short
             are skipped without moving.
  support    measured pitch compared against the pitch forward kinematics
             predicts for rigid legs, rather than against a fixed threshold.

The hand-written stance at `hexapod_walk --height 40`, (0, -15.5, +84.9) in
this space, is the noise baseline in that mode.

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
  with the model  the turn forward kinematics predicts at that frame.

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

from robot import (FRAME_SECONDS, MAX_JOINT_DEG_PER_SEC, STRAIGHT, TIP_ABORT_DEGREES,
                   Impeded, angle_between, floor_normal, leg_geometry, rotate_like, tilt)

# Search box. The angles are degrees from straight; the hip range is part of
# the safety cage, keeping the front and rear pairs from swinging into each
# other. Speed is the fastest joint as a share of MAX_JOINT_DEG_PER_SEC.
BOUNDS = [
    (-20.0, 20.0),     # hip
    (-60.0, 45.0),     # knee
    (0.0, 140.0),      # ankle
    (0.1, 1.0),        # speed
]
PARAMETER_NAMES = ["hip", "knee", "ankle", "speed"]

# The hand-written stance at hexapod_walk --height 40, at about the 0.8 s the
# old fixed ramp took; model mode only
HAND_STANCE = [0.0, -15.5, 84.9, 0.4]

# With the model, the commanded height a stand must reach, mm. About the
# tallest hand-written stance, hexapod_walk --height 80 at 124 mm; roughly a
# fifth of the poses the cage allows reach it. search.py --target-height
# overrides it.
TARGET_HEIGHT_MM = 120.0

# The support check: knees of the front pair (legs 1, 6) down and the rear
# pair (legs 3, 4) up by this much.
SUPPORT_KNEE_DEGREES = 6.0
FRONT = (0, 5)
REAR = (2, 3)
# Measured only: the body must pitch at least this far. A judgement, set well
# above the accelerometer's noise; support_measured_deg is logged to set it.
SUPPORT_MIN_MEASURED_DEGREES = 1.0
# With the model: this share of the predicted pitch must appear, and a check
# where the prediction is under the minimum cannot be read, so does not pass.
SUPPORT_MIN_RATIO = 0.5
SUPPORT_MIN_PREDICTED_DEGREES = 1.5

# The other two gates on standing
LEVEL_MAX_DEGREES = 5.0
STILL_MAX_DEG_PER_SEC = 5.0

# Frames of the stand-up between standing checks
CHECK_EVERY = 10

# How far the body may stray from the orientation its command should give
# before a ramp counts as impeded. Above the accelerometer's noise and the
# apparent tilt a slow ramp's own acceleration adds; below the 40 degree tip.
IMPEDED_DEGREES = 8.0
WATCH_SECONDS = 0.03
WATCH_SAMPLES = 3

# A successful stand scores (base - costs) * exp(-time / TIME_DECAY), where
# base is the height in mm with the model and STOOD_REWARD without it. At 2 s,
# a stand reached in 0.5 s keeps 78%, one taking 2 s keeps 37%.
STOOD_REWARD = 100.0
LEVEL_COST_PER_DEGREE = 3.0
MOTION_COST_PER_DEG_PER_SEC = 0.5
TIME_DECAY_SECONDS = 2.0

# Below anything that stood, and not decayed. Skipped or restricted (nothing
# moved) < impeded (stopped partway) < tipped < unsupported.
INFEASIBLE_REWARD = -300.0
IMPEDED_REWARD = -250.0
TIPPED_REWARD = -200.0
UNSUPPORTED_REWARD = -150.0

RAMP_FRAMES = 40
TILT_FRAMES = 10
SETTLE_SECONDS = 0.5
TILT_SETTLE_SECONDS = 0.4
MEASURE_SECONDS = 0.6
MEASURE_SAMPLES = 10


class Tipped(Exception):
    pass


def joints(candidate):
    """Six legs' (hip, knee, ankle) from one shared set of offsets."""
    hip, knee, ankle = candidate[:3]
    right = (STRAIGHT[0] + hip, STRAIGHT[1] + knee, STRAIGHT[2] + ankle)
    left = (STRAIGHT[0] - hip, STRAIGHT[1] + knee, STRAIGHT[2] + ankle)
    return [right] * 3 + [left] * 3


def stand_up_frames(pose, speed):
    """Frames for the move from straight, set by the joint with furthest to
    go: at speed 1.0 it moves at the cage's limit, every other joint slower."""
    travel = max(abs(p - s) for joint in pose for p, s in zip(joint, STRAIGHT))
    seconds = travel / (speed * MAX_JOINT_DEG_PER_SEC)
    return max(1, int(math.ceil(seconds / FRAME_SECONDS)))


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


def infeasible(robot, pose, use_model=False, target_mm=TARGET_HEIGHT_MM):
    """Why this pose cannot be tried, checked before anything moves."""
    straight = [STRAIGHT] * 6
    problem = robot.path_violation(pose, RAMP_FRAMES, start=straight)
    if problem:
        return problem
    problem = robot.path_violation(pitched(pose), TILT_FRAMES, start=pose)
    if problem:
        return "support check: " + problem
    if use_model and height(pose) < target_mm:
        return "never reaches the %.0f mm target, %.0f mm at most" % (target_mm, height(pose))
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


def standing_check(robot, pose, level_reference, use_model):
    """Hold the current pose and test it. Returns (standing, measurements).

    The measurements say why a check failed under "not_standing". The cheap
    gates go first, so a body still moving or leaning is not nudged at all.
    Raises Tipped past the tip angle, and Impeded from a watched nudge.
    """
    if robot.path_violation(pitched(pose), TILT_FRAMES, start=pose):
        return False, {"not_standing": "the support nudge cannot be made from here"}
    predicted = predicted_pitch(pose) if use_model else None
    if use_model and predicted < SUPPORT_MIN_PREDICTED_DEGREES:
        return False, {"not_standing": "support check would not show here, %.2f degrees"
                       % predicted}

    robot.wait(SETTLE_SECONDS)
    base, motion = robot.sample(MEASURE_SECONDS, MEASURE_SAMPLES)
    if tilt(base) > TIP_ABORT_DEGREES:
        raise Tipped()
    level = angle_between(base, level_reference)
    found = {"level_deg": round(level, 2), "motion_dps": round(motion, 2)}
    if motion > STILL_MAX_DEG_PER_SEC:
        found["not_standing"] = "still moving, %.1f deg/s" % motion
        return False, found
    if level > LEVEL_MAX_DEGREES:
        found["not_standing"] = "leaning %.1f degrees" % level
        return False, found

    # The nudge's size is only known with the model, so only then watched
    robot.ramp(pitched(pose), TILT_FRAMES,
               watcher(robot, "support nudge", True) if use_model else None)
    robot.wait(TILT_SETTLE_SECONDS)
    vector, _ = robot.sample(MEASURE_SECONDS / 2, MEASURE_SAMPLES // 2)
    robot.ramp(pose, TILT_FRAMES, watcher(robot, "nudge back", True) if use_model else None)
    measured = angle_between(vector, base)
    found["support_measured_deg"] = round(measured, 2)
    if use_model:
        ratio = measured / predicted
        found.update(support_predicted_deg=round(predicted, 2), support_ratio=round(ratio, 2))
        supported = ratio >= SUPPORT_MIN_RATIO
    else:
        supported = measured >= SUPPORT_MIN_MEASURED_DEGREES
    if not supported:
        found["not_standing"] = "unsupported, pitched %.2f degrees" % measured
    return supported, found


def evaluate(robot, candidate, level_reference, use_model=False, target_mm=TARGET_HEIGHT_MM):
    """Run one trial. Returns (reward, detail dict).

    level_reference is the gravity vector measured with the robot lying
    straight-legged, so an IMU mounted slightly off square does not read as a
    lean.
    """
    pose = joints(candidate)
    problem = infeasible(robot, pose, use_model, target_mm)
    if problem:
        return INFEASIBLE_REWARD, {"infeasible": problem}

    detail = {"checks": 0}
    if use_model:
        needed = friction_needed(pose)
        detail.update(target_mm=target_mm,
                      friction_needed=None if needed is None else round(needed, 2))
    started = robot.elapsed
    stood_at = []

    def check(frame):
        if use_model and height(frame) < target_mm:
            return False
        detail["checks"] += 1
        began = robot.elapsed
        standing, found = standing_check(robot, frame, level_reference, use_model)
        detail.update(found)
        if standing:
            stood_at.append(began - started)
            detail.pop("not_standing", None)
            if use_model:
                detail["height_mm"] = round(height(frame), 1)
        return standing

    try:
        frames = stand_up_frames(pose, candidate[3])
        robot.ramp(pose, frames, watcher(robot, "standing up", use_model),
                   done=check, check_every=CHECK_EVERY)
        if not stood_at:
            check(robot.pose)
        detail["reached_pose"] = robot.pose == pose
    except Impeded as stopped:
        detail["impeded"] = str(stopped)
        return IMPEDED_REWARD, detail
    except Tipped:
        detail["tipped"] = "checking the stand"
        return TIPPED_REWARD, detail
    finally:
        # From wherever it stopped. Not watched: if the way back is impeded
        # too, stopping halfway would leave the robot stranded mid-pose.
        robot.ramp([STRAIGHT] * 6, RAMP_FRAMES)

    if not stood_at:
        detail["unsupported"] = detail.pop("not_standing", "no check passed")
        return UNSUPPORTED_REWARD, detail

    seconds = stood_at[0]
    detail["stand_s"] = round(seconds, 2)
    base_score = detail["height_mm"] if use_model else STOOD_REWARD
    reward = ((base_score
               - LEVEL_COST_PER_DEGREE * detail["level_deg"]
               - MOTION_COST_PER_DEG_PER_SEC * detail["motion_dps"])
              * math.exp(-seconds / TIME_DECAY_SECONDS))
    return reward, detail
