"""Trial loop for the standing task.

The search algorithm is deliberately the least interesting part. Getting a few
hundred trials to run unattended without damaging anything is the work that
decides whether any algorithm can succeed, so the harness comes first and
random search exists mainly to prove the harness is sound.

What to look for on the first run is NOT good standing. It is:

  - does the reward vary at all, or is it noise?
  - does a repeated pose give a repeatable reward?
  - does the robot survive a few hundred trials unattended?

If the second is no, no optimiser can help -- fix the measurement first.

    python search.py --fake --trials 20        # no hardware, checks the loop
    python search.py --trials 100              # on the robot, measured only
    python search.py --trials 100 --use-model  # with the robot's geometry
    python search.py --method cmaes --trials 100

Without --use-model the robot starts from nothing: no model of its geometry,
no pose known to stand, nothing carried over from earlier runs. Only the
safety cage is built in. See stand.py for what --use-model adds.
"""

import argparse
import json
import random
import sys
import time

from robot import BATTERY_MIN_VOLTS, TIP_ABORT_DEGREES, Robot, angle_between
import stand


class LowBattery(Exception):
    def __init__(self, volts):
        super().__init__(volts)
        self.volts = volts


def log_trial(handle, record):
    handle.write(json.dumps(record) + "\n")
    handle.flush()   # a crash or a flat battery must not lose the run


def needs_a_hand(robot, level_reference):
    """True if the robot is still on its side after returning to rest.

    A hexapod cannot right itself from its back, and pretending otherwise just
    grinds the servos.
    """
    vector, _ = robot.sample(0.4, 6)
    return angle_between(vector, level_reference) > TIP_ABORT_DEGREES


# Poses within this many degrees, on every joint, of one that was impeded are
# skipped for the rest of the run, whatever their speed: nearby poses share
# most of their path, and a path that drags or catches does so at any speed.
RESTRICT_DEGREES = 5.0


def near_impeded(candidate, impeded):
    for other in impeded:
        if all(abs(a - b) <= RESTRICT_DEGREES for a, b in zip(candidate[:3], other[:3])):
            return other
    return None


def to_box(unit):
    return [low + u * (high - low) for u, (low, high) in zip(unit, stand.BOUNDS)]


def summary(reward, detail):
    if "infeasible" in detail:
        return "%7.1f   skipped, %s" % (reward, detail["infeasible"])
    if "restricted" in detail:
        return "%7.1f   restricted, %s" % (reward, detail["restricted"])
    if "impeded" in detail:
        return "%7.1f   impeded, %s" % (reward, detail["impeded"])
    if "tipped" in detail:
        return "%7.1f   tipped while %s" % (reward, detail["tipped"])
    if "unsupported" in detail:
        return "%7.1f   did not stand after %d checks, last: %s" % (
            reward, detail["checks"], detail["unsupported"])
    line = "%7.1f   stood in %4.2f s, check %d" % (reward, detail["stand_s"], detail["checks"])
    if "height_mm" in detail:
        line += " at h %5.1f" % detail["height_mm"]
    if not detail["reached_pose"]:
        line += " (short of the pose)"
    line += "  level %4.2f  motion %4.2f  " % (detail["level_deg"], detail["motion_dps"])
    if "support_ratio" in detail:
        line += "support %4.2f" % detail["support_ratio"]
    else:
        line += "pitched %4.2f deg" % detail["support_measured_deg"]
    return line


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=50)
    parser.add_argument("--method", choices=["random", "cmaes"], default="random")
    parser.add_argument("--fake", action="store_true",
                        help="run the loop with no hardware; does not simulate physics")
    parser.add_argument("--seed", type=int, default=None,
                        help="draw every trial's seed from this one, so the run can be "
                             "repeated exactly (default: each trial's seed comes straight "
                             "from the operating system, and no run repeats)")
    parser.add_argument("--out", default="trials.jsonl")
    parser.add_argument("--repeats", type=int, default=3,
                        help="runs of one pose first, to measure noise (0 skips)")
    parser.add_argument("--use-model", action="store_true",
                        help="score with the robot's geometry: forward-kinematics height, "
                             "predicted support pitch, and the hand stance as the noise baseline")
    parser.add_argument("--surface", default="unlabelled",
                        help="what the robot is standing on, recorded with every trial; "
                             "run the same --seed on several surfaces, then combine.py")
    parser.add_argument("--target-height", type=float, default=stand.TARGET_HEIGHT_MM,
                        help="with --use-model, the commanded height in mm a stand must "
                             "reach before the trial can end (default %(default).0f)")
    options = parser.parse_args()
    use_model = options.use_model
    print("mode: %s" % ("with the model (forward kinematics, hand-stance baseline), "
                        "target height %.0f mm" % options.target_height
                        if use_model else "measured only, nothing assumed beyond the safety cage"))

    # Without --seed, every trial draws its seed from the operating system, so
    # no trial depends on another. The run still gets an id of its own, which
    # seeds only the noise-check pose and CMA-ES.
    repeatable = options.seed is not None
    system = random.SystemRandom()
    if not repeatable:
        options.seed = system.randrange(1, 1000000)
        print("run %d: every trial seeded independently by the operating system"
              % options.seed)
    else:
        print("seed %d: trial seeds drawn from it, so --seed %d repeats this run"
              % (options.seed, options.seed))
    rng = random.Random(options.seed) if repeatable else system
    if options.method == "cmaes":
        try:
            import cma
        except ImportError:
            print("cmaes needs the `cma` package: uv pip install cma")
            return 1

    best = None
    trial = 0
    started = time.time()

    if not options.fake:
        print("All 18 servos go straight to the legs-straight pose as soon as it starts.")
        print("Support the body; Ctrl-C to abort.")
        time.sleep(3.0)

    with Robot(fake=options.fake) as robot, open(options.out, "w") as log:
        robot.wait(1.5)
        level_reference, _ = robot.sample(1.0, 20)
        low, volts = robot.battery_low()
        print("surface: %s   battery: %s V" % (options.surface,
                                               " / ".join("%.2f" % v for v in volts)))
        if low:
            print("Battery below %s V before starting. Charge it first."
                  % " / ".join("%.1f" % v for v in BATTERY_MIN_VOLTS))
            return 1

        # Poses this run found impeded. Kept for this run only, so every run
        # starts from nothing and a new surface is not judged by an old one.
        impeded = []

        def run(candidate, label, target=options.target_height, trial_seed=None):
            nonlocal best, trial
            # A flat pack can brown out the Pi mid-move, so stop cleanly first
            low, volts = robot.battery_low()
            if low:
                raise LowBattery(volts)
            near = near_impeded(candidate, impeded)
            if near is not None:
                reward = stand.INFEASIBLE_REWARD
                detail = {"restricted": "within %.0f degrees of impeded %s"
                          % (RESTRICT_DEGREES, " ".join("%.1f" % c for c in near))}
            else:
                reward, detail = stand.evaluate(robot, candidate, level_reference,
                                                use_model, target)
                if "impeded" in detail:
                    impeded.append(list(candidate))
            record = {"trial": trial, "seed": options.seed, "repeatable": repeatable,
                      "use_model": use_model,
                      "surface": options.surface,
                      "label": label, "reward": round(reward, 2),
                      "trial_seed": trial_seed,
                      "candidate": [round(c, 1) for c in candidate]}
            record.update(detail)
            log_trial(log, record)
            marker = ""
            if (label == "search" and "infeasible" not in detail
                    and "restricted" not in detail
                    and (best is None or reward > best[0])):
                best = (reward, list(candidate))
                marker = "  <- best"
            print("%-6s %4d  %s  %s%s" % (label, trial,
                                          " ".join("%6.1f" % c for c in candidate),
                                          summary(reward, detail), marker))
            trial += 1
            if "tipped" in detail and needs_a_hand(robot, level_reference):
                robot.relax()
                input("  still on its side. Set it upright on its belly, then press Enter.")
                robot.reenergise()
            return reward, detail

        def random_candidate():
            # Every trial gets a seed of its own, logged, so one trial can be
            # regenerated from its seed alone. It comes from the operating
            # system, or from --seed when the run is to be repeatable.
            trial_seed = rng.randrange(1, 2 ** 31)
            draw = random.Random(trial_seed)
            return to_box([draw.random() for _ in stand.BOUNDS]), trial_seed

        try:
            # The same pose several times: if this spread is large, the search
            # is climbing noise and no optimiser will help. With the model the
            # pose is the hand stance; without it, the first random pose that
            # can be tried, so nothing known to stand is ever shown.
            if options.repeats:
                target = options.target_height
                if use_model:
                    pose, label = stand.HAND_STANCE, "hand"
                    # Measured at its own height, so a target above it does
                    # not skip the baseline
                    target = min(target, stand.height(stand.joints(pose)) - 0.5)
                else:
                    # Its own stream, so the search draws the same candidates
                    # for a given seed whatever the mode
                    picker = random.Random("repeat-%d" % options.seed)
                    draw = lambda: to_box([picker.random() for _ in stand.BOUNDS])
                    pose, label = draw(), "repeat"
                    while stand.infeasible(robot, stand.joints(pose)):
                        pose = draw()
                results = [run(pose, label, target) for _ in range(options.repeats)]
                rewards = [r for r, _ in results]
                pitches = [d["support_measured_deg"] for _, d in results
                           if "support_measured_deg" in d]
                spread = max(rewards) - min(rewards)
                print("repeated pose: reward mean %.1f, spread %.1f over %d runs"
                      % (sum(rewards) / len(rewards), spread, len(rewards)))
                if pitches:
                    print("  support pitch measured %.2f to %.2f deg"
                          % (min(pitches), max(pitches)))
                if spread > 15.0:
                    print("WARNING: that is a lot of noise to optimise through. Try a")
                    print("longer stand.SETTLE_SECONDS before trusting any result.")

            if options.method == "random":
                for _ in range(options.trials):
                    candidate, trial_seed = random_candidate()
                    run(candidate, "search", trial_seed=trial_seed)
            else:
                # Searched in the unit cube so one step size suits all three axes
                optimiser = cma.CMAEvolutionStrategy(
                    [0.5] * len(stand.BOUNDS), 0.3,
                    {"bounds": [0.0, 1.0], "seed": options.seed, "verbose": -9})
                done = 0
                while done < options.trials and not optimiser.stop():
                    batch = optimiser.ask()[:options.trials - done]
                    losses = [-run(to_box(unit), "search")[0] for unit in batch]
                    done += len(batch)
                    if len(batch) == optimiser.popsize:
                        optimiser.tell(batch, losses)
        except KeyboardInterrupt:
            print("\ninterrupted")
        except LowBattery as stopped:
            print("\nstopped: battery at %s V, below %s V. Charge it before the next run%s."
                  % (" / ".join("%.2f" % v for v in stopped.volts),
                     " / ".join("%.1f" % v for v in BATTERY_MIN_VOLTS),
                     "; --seed %d repeats this one" % options.seed if repeatable else ""))

    elapsed = time.time() - started
    print("\n%d trials in %.0f s (%.1f s each)" % (trial, elapsed, elapsed / max(trial, 1)))
    if best is not None:
        print("best reward %.1f at hip %+.1f  knee %+.1f  ankle %+.1f degrees from straight,"
              " speed %.2f" % ((best[0],) + tuple(best[1])))
    print("log: %s" % options.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
