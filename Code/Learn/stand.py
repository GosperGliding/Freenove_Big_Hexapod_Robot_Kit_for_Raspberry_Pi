"""The standing task: a candidate foot position in, one number out.

The symmetric first stage
-------------------------
Every leg is given the same foot position in its own leg frame, so a candidate
is three numbers: x (out from the coxa), y (swept along the body), z (up). The
left side gets y mirrored, because a reflection across the body's centreline
reverses the sense of the leg frame's y axis; without the flip the six feet
would all sweep the same way round, which is a twist, not a mirror image.

The hand-written stance at `hexapod_walk --height 40` is exactly (140, 0, -84)
in this space, so the score it has to beat sits inside the search, not beside
it.

What standing means numerically
-------------------------------
A stance you could walk from. After the pose settles, each foot in turn is
lifted 30 mm and the body's tilt is measured. On N feet the robot is stable
only while its centre of mass projects inside the support polygon; lifting a
foot shrinks the polygon, so the tilt it causes measures the stability margin
directly.

  height   the foot drop asked for, -z. Unmeasured: it pushes the pose upward.
  lift     worst tilt across the six lifts. Defines standing.
  level    tilt of the settled pose against the robot lying at rest.
  still    rotation rate while settled -- whether the tilt reading is valid.

Height alone is maximised by tucking the feet under the coxas, which is level,
still and tall and falls over the moment a foot lifts. The lift term is the
only one that pushes back, and under symmetry it carries even more of the
weight: a symmetric pose that sags under load sags evenly and stays level.

All six legs are lifted, not one per mirror pair, although symmetry says the
pairs should agree. They are only equivalent if the robot is mechanically
symmetric, and this one has never been calibrated. The difference between a
leg's lift and its mirror's is recorded with every trial, which turns the
symmetry from an assumption into something measured.
"""

from robot import REST, TIP_ABORT_DEGREES, angle_between, tilt

# Search box, leg frame, mm
BOUNDS = [
    (80.0, 200.0),     # x: out from the coxa
    (-50.0, 50.0),     # y: swept along the body
    (-150.0, -20.0),   # z: up; negative puts the foot below the coxa
]
PARAMETER_NAMES = ["x", "y", "z"]

# The hand-written stance at hexapod_walk --height 40
HAND_STANCE = [140.0, 0.0, -84.0]

LIFT_MM = 30.0

# Reward weights, in mm per unit, so every term is millimetres and they add.
# A degree of tilt when a foot lifts costs 4 mm of height, so a pose 20 mm
# taller must lift its feet with under 5 degrees more wobble to be worth it.
LIFT_COST_PER_DEGREE = 4.0
LEVEL_COST_PER_DEGREE = 3.0
MOTION_COST_PER_DEG_PER_SEC = 0.5

# A candidate that may not be commanded scores below anything that ran; one
# that ended on its side scores below anything that stayed up.
INFEASIBLE_REWARD = -300.0
TIPPED_REWARD = -200.0

RAMP_FRAMES = 30
LIFT_FRAMES = 8
SETTLE_SECONDS = 1.0
LIFT_SETTLE_SECONDS = 0.4
MEASURE_SECONDS = 0.6
MEASURE_SAMPLES = 10


def feet(candidate):
    """Six leg-frame foot positions from one shared (x, y, z)."""
    x, y, z = candidate
    return [[x, y, z]] * 3 + [[x, -y, z]] * 3


def lifted(positions, leg):
    raised = [list(p) for p in positions]
    raised[leg][2] += LIFT_MM
    return raised


def infeasible(robot, positions):
    """Why this pose cannot be tried, checked before anything moves."""
    problem = robot.path_violation(positions, RAMP_FRAMES, start=REST)
    if problem:
        return problem
    for leg in range(6):
        problem = robot.path_violation(lifted(positions, leg), LIFT_FRAMES, start=positions)
        if problem:
            return "lifting leg %d: %s" % (leg + 1, problem)
    return None


def evaluate(robot, candidate, level_reference):
    """Run one trial. Returns (reward, detail dict).

    level_reference is the gravity vector measured with the robot lying at
    rest, so an IMU mounted slightly off square does not read as a lean.
    """
    positions = feet(candidate)
    problem = infeasible(robot, positions)
    if problem:
        return INFEASIBLE_REWARD, {"infeasible": problem}

    detail = {"height_mm": round(-candidate[2], 1)}
    try:
        robot.ramp(positions, RAMP_FRAMES)
        robot.wait(SETTLE_SECONDS)
        base, motion = robot.sample(MEASURE_SECONDS, MEASURE_SAMPLES)
        level = angle_between(base, level_reference)
        detail.update(level_deg=round(level, 2), motion_dps=round(motion, 2))
        if tilt(base) > TIP_ABORT_DEGREES:
            detail["tipped"] = "settling"
            return TIPPED_REWARD, detail

        lifts = []
        for leg in range(6):
            robot.ramp(lifted(positions, leg), LIFT_FRAMES)
            robot.wait(LIFT_SETTLE_SECONDS)
            vector, _ = robot.sample(MEASURE_SECONDS / 2, MEASURE_SAMPLES // 2)
            robot.ramp(positions, LIFT_FRAMES)
            lifts.append(angle_between(vector, base))
            if tilt(vector) > TIP_ABORT_DEGREES:
                detail["tipped"] = "lifting leg %d" % (leg + 1)
                return TIPPED_REWARD, detail
    finally:
        robot.ramp(REST, RAMP_FRAMES)

    worst = max(lifts)
    detail.update(
        lift_deg=[round(t, 2) for t in lifts],
        worst_lift_deg=round(worst, 2),
        # Legs 1-3 mirror legs 6-4: 0<->5, 1<->4, 2<->3
        mirror_gap_deg=[round(abs(lifts[i] - lifts[5 - i]), 2) for i in range(3)],
    )
    reward = (-candidate[2]
              - LIFT_COST_PER_DEGREE * worst
              - LEVEL_COST_PER_DEGREE * level
              - MOTION_COST_PER_DEG_PER_SEC * motion)
    return reward, detail
