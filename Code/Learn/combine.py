"""Rank poses by how they did across several surfaces.

A pose that stands on wood may slide apart on tile. Friction is not
something this robot can measure, so robustness is measured the direct way:
run the same candidates on each surface and keep the worst result.

    python search.py --trials 100 --seed 4242 --surface wood --out wood.jsonl
    python search.py --trials 100 --seed 4242 --surface tile --out tile.jsonl
    python combine.py wood.jsonl tile.jsonl

The same --seed draws the same candidates, so the runs line up pose for pose.
That holds for random search only: CMA-ES chooses its next candidates from
the rewards it has seen, so on a different surface it wanders somewhere else.
"""

import argparse
import json
import sys


def load(path):
    with open(path) as handle:
        return [json.loads(line) for line in handle if line.strip()]


def kind(record):
    for key in ("infeasible", "restricted", "impeded", "tipped", "unsupported"):
        if key in record:
            return key
    return "stood"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", help="trials.jsonl files, one per surface")
    parser.add_argument("--top", type=int, default=10)
    options = parser.parse_args()

    by_surface = {}
    for path in options.logs:
        for record in load(path):
            if record.get("label") != "search":
                continue
            surface = record.get("surface", path)
            key = tuple(record["candidate"])
            by_surface.setdefault(surface, {})[key] = record

    modes = {r.get("use_model", False) for poses in by_surface.values() for r in poses.values()}
    if len(modes) > 1:
        # Height in mm against a fixed 100: the rewards are not on one scale
        print("these logs mix --use-model and measured-only runs; combine one mode at a time")
        return 1

    surfaces = sorted(by_surface)
    if len(surfaces) < 2:
        print("only one surface found (%s); label runs with --surface" % ", ".join(surfaces))
    common = set.intersection(*(set(poses) for poses in by_surface.values()))
    if not common:
        print("no pose appears on every surface -- were the runs made with the same --seed?")
        return 1

    rows = []
    for key in common:
        records = [by_surface[s][key] for s in surfaces]
        rows.append((min(r["reward"] for r in records), key, records))
    rows.sort(key=lambda row: row[0], reverse=True)

    everywhere = sum(1 for _, _, rs in rows if all(kind(r) == "stood" for r in rs))
    somewhere = sum(1 for _, _, rs in rows if any(kind(r) == "stood" for r in rs))
    print("%d poses tried on all of: %s" % (len(rows), ", ".join(surfaces)))
    print("stood on every surface: %d   on some but not all: %d"
          % (everywhere, somewhere - everywhere))
    print()
    print("%-22s %8s   %s" % ("hip knee ankle", "worst", "   ".join(surfaces)))
    for worst, key, records in rows[:options.top]:
        cells = ["%7.1f %-11s" % (r["reward"], kind(r)) for r in records]
        print("%-22s %8.1f   %s" % (" ".join("%.1f" % c for c in key), worst, " ".join(cells)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
