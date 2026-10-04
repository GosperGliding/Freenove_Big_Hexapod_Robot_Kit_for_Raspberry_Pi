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
bypasses those too.

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

## Two modes: measured only, or with the model

By default the robot starts from nothing: no model of its own geometry, no
pose known to stand, and nothing carried over between runs. What stays built
in is the safety cage -- servo travel, the fold check, the search box and the
tip abort -- which stops it hurting itself and says nothing about where
standing is. `--use-model` adds the robot's geometry, the same forward
kinematics the simulator runs.

| | measured only (default) | `--use-model` |
|---|---|---|
| supported? | body pitches ≥ 1° when the knees are nudged | body pitches ≥ half what forward kinematics predicts |
| reward if supported | 100 − 3·level − 0.5·still | height − 3·level − 0.5·still |
| height | not known, not rewarded | from forward kinematics of the commanded angles |
| noise check | first random pose that can be tried, 3 times | the hand-written stance, 3 times |
| skipped as unreadable | — | poses predicted to pitch under 1.5° |

Not supported scores −150, tipped −200, impeded −250, skipped or restricted
−300 in both modes.

**The support check.** The front knees go down and the rear knees up by 6
degrees, and the accelerometer measures how far the body pitches. A body
carried by its legs follows; one resting on its belly barely moves, because
the floor is holding it. It matters most with the model: height is commanded,
not measured, so a pose the servos cannot hold sags evenly onto the floor,
still level and still, while height reports the full commanded value.

**What measured-only cannot do.** Nothing on the robot measures height, so
every supported pose earns the same 100, less lean and wobble: the search
learns which poses stand, not which stand tallest. It can also call a
genuinely standing pose unsupported where the knee nudge happens to barely
move the feet, which the model would have predicted and skipped. Both are the
price of assuming nothing. A measured height -- the head's ultrasonic pointed
at the floor, if it tilts that far -- would close the first gap.

The weights and thresholds are judgements; every trial logs the raw
measurements (`support_measured_deg`, and `support_ratio` with the model), so
they can be set from real runs and rewards recomputed afterwards.

Not checked at this stage: whether the stance has margin to lift a foot. That
matters for walking, not standing, and comes back with the walking stage.

## Impeded motion: the restricted region grows

Nothing reads a joint back, so a leg dragging on a high-friction floor, a
foot that sticks, or a leg caught on something is invisible at the joint. It
shows in the body: a leg that cannot follow its command makes the body move
other than the command says. So every ramp is watched every 5 frames against
the orientation the body should have by then:

| | expected orientation | ramps watched |
|---|---|---|
| measured only | unchanged: a mirror-symmetric command should not turn the body, which follows from the command, not a model | standing up |
| `--use-model` | the turn forward kinematics predicts at that frame | standing up, the support nudge and back |

More than 8° off and the ramp stops where it is, so nothing goes on pushing
against whatever is in the way; the robot ramps back to straight, and the
trial scores −250, impeded. For the rest of the run, poses within 5° on every
joint of an impeded one are skipped without moving, logged as restricted. So
the restricted region is the safety cage plus what the robot itself ran into
-- kept for one run only, so each run and each surface starts from nothing.

What it cannot see: an impediment that holds every leg back equally. A
symmetric stand-up that stalls stays level, and the accelerometer cannot tell
a body that failed to rise from one that rose. The support check catches the
worst of it, a body left on its belly. The support nudge is small, about 4°,
so a body that pitches less than it should there is caught by the support
check (unsupported) rather than the watch.

Energy is not in the reward. The battery packs are still read on the ADS7830,
but only to stop a run cleanly when either drops below the 5.5 / 6.0 V at
which `server.py` raises its low-battery alarm.

## Surfaces

Friction is not measurable here either, so robustness is measured directly:
run the same candidates on each surface and keep the worst result.

```bash
python search.py --trials 100 --seed 4242 --surface wood --out wood.jsonl
python search.py --trials 100 --seed 4242 --surface tile --out tile.jsonl
python combine.py wood.jsonl tile.jsonl
```

The same `--seed` draws the same candidates in either mode, so the runs line
up pose for pose. `combine.py` ranks poses by their worst reward and counts
those that stood on every surface; it refuses to mix `--use-model` and
measured-only logs, whose rewards are on different scales. This works for
random search only: CMA-ES picks candidates from the rewards it sees, so it
wanders differently on each surface.

With the model, `friction_needed` is logged too -- the friction a foot would
need if its leg pushed like a strut from hip to foot. Servo-held legs are not
struts, so it is a hypothesis: multi-surface runs will show whether poses
with a high value are the ones that slide on smooth floors.

## Run it

From this directory, Python server stopped (it holds BCM 4):

```bash
python search.py --fake --trials 20         # no hardware; checks the loop only
python search.py --trials 100               # on the robot, measured only
python search.py --trials 100 --use-model   # on the robot, with the model
```

Trials take roughly 4 s, so 100 is under 10 minutes. Each run starts with
three repeats of one pose; if their spread exceeds 15 the harness warns,
because a search cannot climb noise. Every run draws a fresh seed and prints
it; `--seed N` repeats a run's candidates exactly. With three parameters random search
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
