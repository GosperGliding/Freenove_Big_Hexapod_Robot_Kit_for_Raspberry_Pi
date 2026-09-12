#!/usr/bin/env python3
"""Compare the Python reference trace against the C++ one.

Exit status is 0 only when the two are identical. On a mismatch, report the
first few differences with the scenario and frame they fall in, because a raw
line number in an 11k-line trace says nothing about which gait branch broke.
"""

import sys
import argparse

MAX_REPORTED = 12

# Which servo channel drives which joint, so a mismatch can name the joint
# rather than a bare channel number.
JOINT_OF_CHANNEL = {}
for _leg, _coxa, _femur, _tibia in [
    (1, 15, 14, 13),
    (2, 12, 11, 10),
    (3, 9, 8, 31),
    (6, 16, 17, 18),
    (5, 19, 20, 21),
    (4, 22, 23, 27),
]:
    JOINT_OF_CHANNEL[_coxa] = "leg %d coxa" % _leg
    JOINT_OF_CHANNEL[_femur] = "leg %d femur" % _leg
    JOINT_OF_CHANNEL[_tibia] = "leg %d tibia" % _leg


def load(path):
    with open(path, "r") as handle:
        return [line.rstrip("\n") for line in handle]


def describe(lines, index):
    """Return (scenario, frame) context for a line index."""
    scenario = "<construction>"
    start = 0
    for i in range(index, -1, -1):
        if lines[i].startswith("s "):
            scenario = lines[i][2:]
            start = i + 1
            break
    # 18 servo writes per frame, so the write count since the scenario header
    # gives the frame the mismatch landed in.
    writes = sum(1 for i in range(start, index) if lines[i].startswith("w "))
    return scenario, writes // 18


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("reference")
    parser.add_argument("actual")
    options = parser.parse_args()

    reference = load(options.reference)
    actual = load(options.actual)

    mismatches = []
    for index in range(min(len(reference), len(actual))):
        if reference[index] != actual[index]:
            mismatches.append(index)
            if len(mismatches) >= MAX_REPORTED:
                break

    if not mismatches and len(reference) == len(actual):
        writes = sum(1 for line in reference if line.startswith("w "))
        skips = sum(1 for line in reference if line == "!range")
        print("PASS  %d lines identical (%d servo writes, %d out-of-range frames)"
              % (len(reference), writes, skips))
        return 0

    print("FAIL")
    if len(reference) != len(actual):
        print("  trace lengths differ: reference %d, C++ %d"
              % (len(reference), len(actual)))

    for index in mismatches:
        scenario, frame = describe(reference, index)
        ref_line = reference[index]
        act_line = actual[index]
        joint = ""
        if ref_line.startswith("w "):
            channel = int(ref_line.split()[1])
            joint = "  (%s)" % JOINT_OF_CHANNEL.get(channel, "channel %d" % channel)
        print("  line %d, scenario '%s', frame %d%s" % (index + 1, scenario, frame, joint))
        print("      reference: %s" % ref_line)
        print("      c++      : %s" % act_line)

    total = sum(1 for i in range(min(len(reference), len(actual)))
                if reference[i] != actual[i])
    if total > len(mismatches):
        print("  ... and %d more differing lines" % (total - len(mismatches)))
    return 1


if __name__ == "__main__":
    sys.exit(main())
