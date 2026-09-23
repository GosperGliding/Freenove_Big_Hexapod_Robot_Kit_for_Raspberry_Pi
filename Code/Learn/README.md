# Learning to stand

An attempt to have the robot discover a standing pose rather than be given
one. Nothing here describes what standing looks like: no leg pairing, no
symmetry, no notion of which joint bends which way. Just 18 servo angles, a
number saying how well that worked, and a search.

## Why standing before walking

Walking needs a displacement signal, and this robot has no good one — the
accelerometer cannot be integrated over a trial without drifting into
nonsense, and the ultrasonic only measures forward distance, which means
building a wall-and-reset rig before a single trial can run.

Standing needs none of that. It is static, so the accelerometer measures
orientation exactly; the robot does not move, so there is nothing to reset
between trials; and a trial takes three seconds instead of thirty. It
exercises the whole harness against an objective that can actually be checked.

## Run it

```bash
python search.py --fake --trials 20     # no hardware; checks the loop only
python search.py --trials 200           # on the robot, random search
python search.py --method cmaes --trials 400
```

`--fake` does not simulate physics. It cannot tell you whether a pose stands —
it exists so the loop, the logging and the search can be debugged somewhere
other than on a live robot.

## What the score means

Three terms, and the point is that no one of them is enough alone:

| term | where from | what it does |
|---|---|---|
| **height** | forward kinematics of the commanded angles | what we are asking for |
| **level** | accelerometer | says whether the robot *achieved* the pose or folded under it |
| **still** | gyroscope | a pose caught mid-collapse reads level for an instant; one that is level and motionless is holding |

Maximise height alone and it commands a tall pose it cannot hold. Maximise
level and still alone and it lies flat on the floor — beautifully stable, not
standing. Only the combination has a standing pose at its optimum.

Attitude comes from the **raw accelerometer**, not `imu.py`'s fused estimate.
For a static pose that is the better instrument, not a compromise: gravity
alone fixes orientation when nothing is moving, so there is no integration and
no drift. It also avoids two defects in the vendor's version — `imu.py:128`
returns `(pitch, roll, yaw)` while `control.py:322` unpacks it as
`(roll, pitch, yaw)`, and the quaternion `z` update multiplies by
`quaternion_x` where the pattern requires `gyro_x`.

## Scores to beat

The height term for poses that already exist, in millimetres:

| pose | height |
|---|---|
| reference pose, legs folded | 19.1 |
| `run_gait` default stance | 39.2 |
| `hexapod_walk --height 40` | **84.1** |
| `hexapod_walk --height 80` | 123.9 |
| straight-down legs, geometric ceiling | ~233 |

84.1 is the number to beat — that is the hand-written stance at its default
ride height. Note the ceiling is not really 233: a pose that tall has its feet
almost directly under the coxas, leaving a support polygon too small to hold.
The robot has to find that trade-off from the measurement, and nothing tells
it the trade-off exists.

## What to look for on the first run

Not good standing. Random search over 18 dimensions will mostly produce poses
with the feet *above* the body. What matters is:

1. **Does the reward vary at all**, or is it noise?
2. **Does the same pose score the same twice?** `search.py` measures this
   before starting and warns if the spread is large. If a pose scores 40 one
   time and 90 the next, the search is climbing noise and no optimiser can fix
   that — lengthen `stand.SETTLE_SECONDS` first.
3. **Does the robot survive a few hundred trials unattended?**

Only when all three are yes is it worth switching to CMA-ES.

## Safety

- Joint angles clamp to 15–165°. The last few degrees at each end fold a leg
  into the chassis where it jams and stalls, and stalled servos are how this
  hardware dies.
- A trial whose tilt exceeds 40° scores `-200` and aborts, rather than leaving
  the robot straining against the floor.
- After a tip the harness returns to the reference pose and retries. If it is
  still on its side it relaxes the servos and waits for a human — a hexapod on
  its back cannot right itself, and pretending otherwise just grinds gears.
- Every trial is written to `trials.jsonl` and flushed immediately, so a
  crash or a disconnected battery loses at most the trial in progress.

## Files

```
robot.py    hardware access and the safety rules; nothing task-specific
stand.py    the task: 18 angles in, one reward out
search.py   the trial loop, logging, and the search itself
```

The search algorithm is deliberately the least interesting part. Getting
several hundred trials to run unattended without damaging anything is the work
that decides whether any algorithm can succeed.
