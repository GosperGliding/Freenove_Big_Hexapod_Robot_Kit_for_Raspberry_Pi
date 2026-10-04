# Learning to stand

The robot finds a standing pose by trying candidates and measuring them,
rather than being handed one. This is black-box search -- random search, then
CMA-ES -- not reinforcement learning: a candidate is a whole pose, scored once,
not a policy reacting to its sensors.

## Why standing before walking

Walking needs a displacement signal and this robot has no good one: the
accelerometer cannot be integrated over a trial without drifting, the
ultrasonic only sees forward, and the camera is dead. Standing is static, so
the accelerometer measures orientation exactly and a trial takes seconds.

## Stage 1: one foot position for every leg

Each leg gets the same `(x, y, z)` in its own leg frame, with `y` flipped on
the left side -- a reflection across the centreline reverses the leg frame's
`y`, which `Control.transform_coordinates` confirms. Three parameters. The
hand-written stance at `hexapod_walk --height 40` is `(140, 0, -84)`, inside
the search box, so the number to beat is measured on the same footing.

Later stages relax this: 9 parameters with only left/right mirroring, then 18
with none.

## What the score means

A stance you could walk from. After the pose settles, each foot is lifted
30 mm in turn and the body's tilt is measured. Lifting a foot shrinks the
support polygon, so the tilt it causes is a direct measure of how far inside
the polygon the centre of mass sits.

| term | from | role |
|---|---|---|
| **lift** | accelerometer, worst of six lifts | defines standing |
| height | the commanded `-z` | pushes the pose upward |
| level | accelerometer, against the robot lying at rest | catches a lean |
| still | gyroscope | says whether the tilt reading is valid |

Height alone is maximised by tucking the feet under the body: level, still,
tall, and it falls over the moment a foot lifts. Lift is the only term that
pushes back, and under symmetry it carries even more of the weight, because a
symmetric pose that sags under load sags evenly and stays level.

All six legs are lifted, though symmetry says mirror pairs should agree. They
only agree if the robot is mechanically symmetric, and it has never been
calibrated. Each trial logs `mirror_gap_deg`, the difference between a leg's
lift and its mirror's: large gaps mean the symmetric stage is fighting the
hardware, and are the cue to calibrate or move to the 9-parameter stage.

## How a trial moves

Every trial ramps out of Control's rest pose (feet at `140 0 0`, body on the
floor) over 30 frames and ramps back at the end. Ramping keeps every servo
inside its slew rate, so the robot is not dragged through poses nobody asked
for and scored on the transition instead of the pose. Starting from the same
pose each time makes trials independent of their order.

Every frame of every ramp and lift is checked before the first is sent: foot
reach must stay within 90-225 mm and every servo within 15-165 degrees,
calibration offsets included. A candidate that fails is logged as skipped and
nothing moves.

## Run it

From this directory, Python server stopped (it holds BCM 4):

```bash
python search.py --fake --trials 20     # no hardware; checks the loop only
python search.py --trials 100           # on the robot, random search
```

Trials take roughly 10 s, so 100 is under 20 minutes. Each starts with three
runs of the hand stance; if their spread exceeds 15 the harness warns, because
a search cannot climb noise. With three parameters random search covers the
box well; `--method cmaes` (`uv pip install cma`) matters more at 9 and 18.

`--fake` does not simulate physics. It exercises the loop, the feasibility
checks and the logging, nothing more.

## Safety

- Constructing `Control` drives all 18 joints to rest immediately. Support
  the body and keep the power switch in reach.
- A tilt beyond 40 degrees aborts the trial and returns to rest. If the robot
  is still over, servos relax and the harness waits for you.
- Every trial is flushed to `trials.jsonl` as it finishes.
