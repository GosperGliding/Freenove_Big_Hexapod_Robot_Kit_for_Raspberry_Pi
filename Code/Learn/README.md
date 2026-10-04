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

## Stage 1: three joint angles, shared by every leg

A candidate is three angles, each an offset from the legs-straight pose that
`servo.py` holds while the horns are fitted:

| | range | |
|---|---|---|
| hip | -20..+20 | swings the leg along the body |
| knee | -60..+45 | raises (−) or lowers (+) the thigh |
| ankle | 0..140 | bends the shin down from straight |

Every leg gets the same three, with the hip mirrored on the left so the two
sides are reflections rather than a twist. Servos are written directly: no IK,
no foot coordinates, and no `point.txt` offsets, since the straight pose
bypasses those too. The hand-written `--height 40` stance is
`(0, −15.5, +84.9)`, inside the box.

Later stages relax the symmetry: per mirror pair, then per leg.

## How a trial moves

Every trial starts with the legs straight, ramps to the candidate over 40
frames in joint space, runs its measurements, and ramps back to straight.
Ramping keeps every servo inside its slew rate, so the robot is not dragged
through poses nobody asked for and scored on the transition. Starting from the
same pose each time makes trials independent of their order.

Every frame is checked before the first is sent: each servo within 5-175
degrees, and no foot folded closer than 90 mm to its hip. A candidate that
fails is logged as skipped and nothing moves.

## What the score means

```
if the body does not follow the support check:  -150
else:  reward = height − 3·level − 0.5·still
```

| term | measured as | weight | role |
|---|---|---|---|
| height | mm the feet sit below the hips, from the commanded angles | 1 | pushes the pose upward |
| level | degrees, accelerometer, against the robot lying straight-legged | 3 mm per degree | catches a lean |
| still | deg/s, gyroscope, while settled | 0.5 mm per deg/s | says whether the tilt reading is valid |

The weights convert each measurement into millimetres so the terms add. They
are exchange rates chosen by judgement; every trial logs the raw measurements,
so rewards can be recomputed with other weights afterwards.

Height comes from forward kinematics of the commanded angles -- the link
lengths, used only to score. Nothing is commanded through it.

**The support check.** Height is commanded, not measured. A pose the servos
cannot hold sags until the body rests on the floor, and with every leg doing
the same thing it sags evenly -- still level, still still, height reporting
the full commanded value. So in the settled pose the front knees go down and
the rear knees up by 6 degrees. Forward kinematics predicts how far that
should pitch the body if the legs are carrying it; the accelerometer measures
how far it did. A body on its legs follows; one resting on its belly barely
moves. Under half the prediction and the trial scores as unsupported. Poses
where the prediction is under 1.5 degrees are skipped, since there the check
could not be told from noise. The thresholds are judgements; `support_ratio`
is logged so real trials can set them.

Expect the search to find the tallest pose the servos can hold. With every
leg identical and six feet down, the body is centred over its feet by
construction, so this stage mostly measures where the servos run out of
strength -- and whether the harness can measure that reliably.

Not checked at this stage: whether the stance has margin to lift a foot. That
matters for walking, not standing, and comes back with the walking stage.

## Run it

From this directory, Python server stopped (it holds BCM 4):

```bash
python search.py --fake --trials 20     # no hardware; checks the loop only
python search.py --trials 100           # on the robot, random search
```

Trials take roughly 4 s, so 100 is under 10 minutes. Each run starts with three
trials of the hand stance; if their spread exceeds 15 the harness warns,
because a search cannot climb noise. With three parameters random search
covers the box well; `--method cmaes` (`uv pip install cma`) matters more once
the symmetry is relaxed.

`--fake` does not simulate physics. Its accelerometer reads the plane under
the commanded feet, enough to exercise the loop, the checks and the logging.

## Safety

- Starting the harness energises the rail and drives all 18 joints to
  straight at once, from wherever they are. Support the body and keep the
  power switch in reach.
- A tilt beyond 40 degrees aborts the trial and returns to straight. If the
  robot is still over, servos relax and the harness waits for you.
- Every trial is flushed to `trials.jsonl` as it finishes.
