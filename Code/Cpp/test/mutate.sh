#!/usr/bin/env bash
# Mutation check for the differential harness.
#
# A green test that has never been red proves nothing. Each mutation below
# reintroduces a specific porting mistake that the port was written to avoid;
# the harness is only trustworthy if it catches them. Mutations that are NOT
# caught are just as informative -- they mark places where the fidelity work
# was precautionary rather than load-bearing.
#
# Run from Code/Cpp:  bash test/mutate.sh

set -u

# Clear build artifacts BEFORE snapshotting: object files live next to the
# sources, and restoring a stale .o with a fresh timestamp can leave make
# believing it is current. NTFS timestamp granularity is coarse enough that a
# sed plus a rebuild inside the same tick silently tests the unmutated binary,
# which is exactly the kind of flake that makes a mutation report worthless.
make clean >/dev/null 2>&1

BACKUP="$(mktemp -d)"
cp -r include src "$BACKUP/"

# Replace the trees outright. "cp -r $BACKUP/include ." would copy INTO the
# existing directory rather than over it, leaving mutations in place and every
# later result meaningless.
restore() {
    rm -rf include src
    cp -r "$BACKUP/include" "$BACKUP/src" .
}
trap 'restore; rm -rf "$BACKUP"' EXIT

caught=0
missed=0

check() {
    local name="$1"
    # Full rebuild, not an incremental one: see the timestamp note above.
    make clean >/dev/null 2>&1
    if ! make -s >/dev/null 2>&1; then
        printf '  BUILD FAIL  %s\n' "$name"
        restore
        return
    fi
    rm -f test/mutant.trace
    if ! ./hexapod_trace test/scenarios.txt ../Server/point.txt test/mutant.trace 2>/dev/null \
        || [ ! -f test/mutant.trace ]; then
        printf '  RUN FAIL    %s\n' "$name"
        restore
        return
    fi
    if python test/compare.py test/reference.trace test/mutant.trace >/dev/null 2>&1; then
        printf '  NOT CAUGHT  %s\n' "$name"
        missed=$((missed + 1))
    else
        printf '  caught      %s\n' "$name"
        caught=$((caught + 1))
    fi
    restore
}

# Reference trace must already exist.
if [ ! -f test/reference.trace ]; then
    python test/gen_reference.py -o test/reference.trace >/dev/null 2>&1
fi

echo "mutation checks:"

# 1. Tie-breaking. Python rounds half to even; std::round rounds half away
#    from zero. Every joint angle goes through this.
sed -i 's|return static_cast<int>(std::nearbyint(x));|return static_cast<int>(std::round(x));|' \
    include/hexapod/pyround.hpp
check "py_round uses round-half-away instead of half-to-even"

# 2. The quantisation before the inverse trig in the IK.
sed -i 's|std::asin(py_round2(w))|std::asin(w)|; s|std::acos(py_round2(v))|std::acos(v)|; s|std::acos(py_round2(u))|std::acos(u)|' \
    src/kinematics.cpp
check "IK drops the round-to-2-decimals before asin/acos"

# 3. Integer division in a gait phase boundary. The Python divides in float.
sed -i 's|} else if (j < 3 \* f / 8.0) {|} else if (j < 3 * frames / 8) {|' src/control.cpp
check "tripod phase boundary 3F/8 uses integer division"

# 4. Same, for the wave gait sub-frame count.
sed -i 's|static_cast<int>(f / 18.0)|static_cast<int>(f / 17.0)|' src/control.cpp
check "wave gait lift window off by one divisor"

# 5. Servo emission order. The pose is identical; only the sequence differs.
sed -i 's|{2, 9, 8, 31},|{2, 9, 31, 8},|' src/control.cpp
check "leg 3 femur/tibia channels swapped"

# 6. Degree conversion associativity. Differs at the ulp level about a quarter
#    of the time, but may never reach the rounded integer.
sed -i 's|py_round(a \* kRadToDeg)|py_round(a * 180.0 / kPi)|; s|py_round(b \* kRadToDeg)|py_round(b * 180.0 / kPi)|; s|py_round(c \* kRadToDeg)|py_round(c * 180.0 / kPi)|' \
    src/kinematics.cpp
check "math.degrees spelled as (x*180)/pi instead of x*(180/pi)"

# 7. Rotation-order associativity in the posture solver.
sed -i 's|const double bearing = table\[i\].bearing_deg / 180.0 \* kPi;|const double bearing = table[i].bearing_deg * (kPi / 180.0);|' \
    src/control.cpp
check "mount bearing spelled as deg*(pi/180) instead of (deg/180)*pi"

rm -f test/mutant.trace
echo
echo "caught $caught, not caught $missed"
echo
echo "Mutations that were NOT caught are not bugs in the port -- they mark"
echo "fidelity choices this scenario set does not exercise. Widen the scenarios"
echo "if you want them pinned down."
