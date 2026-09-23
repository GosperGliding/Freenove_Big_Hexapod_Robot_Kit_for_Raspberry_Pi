"""Trial loop for the standing task.

The search algorithm here is deliberately the least interesting part. Getting
several hundred trials to run unattended without damaging anything is the work
that decides whether any algorithm can succeed, so the harness comes first and
random search exists mainly to prove the harness is sound.

What to look for on the first run is NOT good standing. It is:

  - does the reward vary at all, or is it noise?
  - does a repeated pose give a repeatable reward?
  - does the robot survive a few hundred trials unattended?

If the answers are yes, yes and yes, swap in a real optimiser. If the second
is no, no optimiser can help -- fix the measurement first.

    python search.py --fake --trials 20        # no hardware, checks the loop
    python search.py --trials 200              # on the robot
    python search.py --method cmaes --trials 400
"""

import argparse
import json
import random
import statistics
import sys
import time

from robot import JOINT_COUNT, JOINT_MIN, JOINT_MAX, REFERENCE_POSE, Robot
import stand


def log_trial(handle, record):
    handle.write(json.dumps(record) + "\n")
    handle.flush()   # a crash or a yanked battery must not lose the run


def recover(robot, attempts=2):
    """Get back to something upright between trials.

    Returns False if the robot is still on its side, in which case it needs a
    hand -- there is no way for a hexapod to right itself from its back, and
    pretending otherwise just grinds the servos.
    """
    for _ in range(attempts):
        robot.go_reference(settle=1.0)
        if not robot.has_tipped():
            return True
    return False


def repeatability_check(robot, pose, repeats=3):
    """Measure the same pose several times and report the spread.

    This is the single most important number before any optimisation. If the
    same pose scores 40 one time and 90 the next, the search is climbing noise
    and no amount of cleverness will fix it.
    """
    rewards = []
    for _ in range(repeats):
        reward, _ = stand.evaluate(robot, pose)
        rewards.append(reward)
    spread = max(rewards) - min(rewards)
    print("repeatability over %d runs of one pose: %s  (spread %.1f)"
          % (repeats, " ".join("%.1f" % r for r in rewards), spread))
    return spread


def random_search(rng, trials):
    for _ in range(trials):
        yield [rng.uniform(JOINT_MIN, JOINT_MAX) for _ in range(JOINT_COUNT)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=50)
    parser.add_argument("--method", choices=["random", "cmaes"], default="random")
    parser.add_argument("--fake", action="store_true",
                        help="run the loop with no hardware; does not simulate physics")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out", default="trials.jsonl")
    parser.add_argument("--skip-check", action="store_true",
                        help="skip the repeatability check at the start")
    options = parser.parse_args()

    rng = random.Random(options.seed)

    if options.method == "cmaes":
        try:
            import cma  # noqa: F401
        except ImportError:
            print("cmaes needs the `cma` package: uv pip install cma")
            return 1

    best_reward = None
    best_pose = None
    started = time.time()

    with Robot(fake=options.fake) as robot, open(options.out, "w") as log:
        if not options.fake:
            print("about to move all 18 servos. support the body, Ctrl-C to abort.")
            time.sleep(3.0)

        robot.go_reference(settle=1.5)

        if not options.skip_check:
            spread = repeatability_check(robot, REFERENCE_POSE)
            if spread > 15.0:
                print("WARNING: the same pose varies by %.1f mm-equivalent between runs."
                      % spread)
                print("That is a lot of noise to optimise through. Consider a longer")
                print("settle time in stand.SETTLE_SECONDS before trusting any result.")

        if options.method == "random":
            candidates = random_search(rng, options.trials)
        else:
            import cma
            # Start from the middle of the joint range with a wide spread: the
            # reference pose is not a standing pose, so seeding there would
            # bias the search toward a fold rather than help it.
            centre = [(JOINT_MIN + JOINT_MAX) / 2.0] * JOINT_COUNT
            optimiser = cma.CMAEvolutionStrategy(
                centre, 30.0,
                {"bounds": [JOINT_MIN, JOINT_MAX], "seed": options.seed, "verbose": -9})
            candidates = None

        trial = 0
        try:
            while trial < options.trials:
                if options.method == "random":
                    batch = [next(candidates)]
                else:
                    batch = optimiser.ask()

                losses = []
                for pose in batch:
                    if trial >= options.trials:
                        break

                    reward, detail = stand.evaluate(robot, pose)
                    losses.append(-reward)   # cma minimises

                    record = {"trial": trial, "reward": round(reward, 2),
                              "pose": [round(a, 1) for a in pose]}
                    record.update(detail)
                    log_trial(log, record)

                    marker = ""
                    if best_reward is None or reward > best_reward:
                        best_reward, best_pose = reward, list(pose)
                        marker = "  <- best"
                    print("trial %4d  reward %7.1f   h %5.1f  tilt %5.2f  motion %6.2f%s"
                          % (trial, reward, detail["height_mm"], detail["tilt_deg"],
                             detail["motion_dps"], marker))

                    if detail["tipped"]:
                        print("  tipped -- recovering")
                        if not recover(robot):
                            print("  still on its side. Set it upright, then press Enter.")
                            robot.relax()
                            input()
                            robot.go_reference(settle=1.5)

                    trial += 1

                if options.method == "cmaes" and losses:
                    optimiser.tell(batch[:len(losses)], losses)

        except KeyboardInterrupt:
            print("\ninterrupted")

        robot.go_reference(settle=1.0)

    elapsed = time.time() - started
    print("\n%d trials in %.0f s (%.1f s each)" % (trial, elapsed, elapsed / max(trial, 1)))
    if best_pose is not None:
        print("best reward %.1f" % best_reward)
        print("best pose  %s" % " ".join("%.0f" % a for a in best_pose))
    print("log: %s" % options.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
