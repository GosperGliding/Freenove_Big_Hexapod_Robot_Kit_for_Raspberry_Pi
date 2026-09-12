#pragma once

#include <cmath>

namespace hexapod {

// The Python original rounds in two places that are load-bearing for the
// output, so the port has to reproduce its rounding exactly rather than
// approximately.
//
// Do not build this translation unit with -ffast-math: both functions depend
// on the default IEEE-754 rounding mode staying FE_TONEAREST.

// Python's builtin round(x) with no ndigits: returns an integer, breaking
// ties to even. FE_TONEAREST is also round-half-to-even, so nearbyint is an
// exact match -- unlike std::round, which breaks ties away from zero.
inline int py_round(double x)
{
    return static_cast<int>(std::nearbyint(x));
}

// Python's round(x, 2). CPython rounds the true binary value to the nearest
// multiple of 0.01 via correctly-rounded decimal conversion. Scaling by 100
// and rounding is not formally identical -- the multiply can itself round
// across a tie boundary -- but it agrees over the domain used here, where the
// argument is always a clamped [-1, 1] trig ratio. The differential test is
// what certifies that, so if it ever disagrees the test is the place it shows.
inline double py_round2(double x)
{
    return std::nearbyint(x * 100.0) / 100.0;
}

}  // namespace hexapod
