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
    python search.py --trials 100              # on the robot
    python search.py --method cmaes --trials 100
"""

import argparse
import json
import random
import sys
import time

from robot import TIP_ABORT_DEGREES, Robot, angle_between
import stand


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


def to_box(unit):
    return [low + u * (high - low) for u, (low, high) in zip(unit, stand.BOUNDS)]


def summary(reward, detail):
    if "infeasible" in detail:
        return "%7.1f   skipped, %s" % (reward, detail["infeasible"])
    if "tipped" in detail:
        return "%7.1f   tipped while %s" % (reward, detail["tipped"])
    line = "%7.1f   h %5.1f  level %5.2f  motion %5.2f  support %4.2f" % (
        reward, detail["height_mm"], detail["level_deg"], detail["motion_dps"],
        detail["support_ratio"])
    if "unsupported" in detail:
        line += "  unsupported"
    return line


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=50)
    parser.add_argument("--method", choices=["random", "cmaes"], default="random")
    parser.add_argument("--fake", action="store_true",
                        help="run the loop with no hardware; does not simulate physics")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out", default="trials.jsonl")
    parser.add_argument("--repeats", type=int, default=3,
                        help="runs of the hand stance first, to measure noise (0 skips)")
    options = parser.parse_args()

    rng = random.Random(options.seed)
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

        def run(candidate, label):
            nonlocal best, trial
            reward, detail = stand.evaluate(robot, candidate, level_reference)
            record = {"trial": trial, "label": label, "reward": round(reward, 2),
                      "candidate": [round(c, 1) for c in candidate]}
            record.update(detail)
            log_trial(log, record)
            marker = ""
            if (label == "search" and "infeasible" not in detail
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
            return reward

        try:
            # The same pose several times: if this spread is large, the search
            # is climbing noise and no optimiser will help
            rewards = [run(stand.HAND_STANCE, "hand") for _ in range(options.repeats)]
            if rewards:
                spread = max(rewards) - min(rewards)
                print("hand stance: mean %.1f, spread %.1f over %d runs"
                      % (sum(rewards) / len(rewards), spread, len(rewards)))
                if spread > 15.0:
                    print("WARNING: that is a lot of noise to optimise through. Try a")
                    print("longer stand.SETTLE_SECONDS before trusting any result.")

            if options.method == "random":
                for _ in range(options.trials):
                    run(to_box([rng.random() for _ in stand.BOUNDS]), "search")
            else:
                # Searched in the unit cube so one step size suits all three axes
                optimiser = cma.CMAEvolutionStrategy(
                    [0.5] * len(stand.BOUNDS), 0.3,
                    {"bounds": [0.0, 1.0], "seed": options.seed, "verbose": -9})
                done = 0
                while done < options.trials and not optimiser.stop():
                    batch = optimiser.ask()[:options.trials - done]
                    losses = [-run(to_box(unit), "search") for unit in batch]
                    done += len(batch)
                    if len(batch) == optimiser.popsize:
                        optimiser.tell(batch, losses)
        except KeyboardInterrupt:
            print("\ninterrupted")

    elapsed = time.time() - started
    print("\n%d trials in %.0f s (%.1f s each)" % (trial, elapsed, elapsed / max(trial, 1)))
    if best is not None:
        print("best reward %.1f at hip %+.1f  knee %+.1f  ankle %+.1f degrees from straight"
              % ((best[0],) + tuple(best[1])))
    print("log: %s" % options.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
