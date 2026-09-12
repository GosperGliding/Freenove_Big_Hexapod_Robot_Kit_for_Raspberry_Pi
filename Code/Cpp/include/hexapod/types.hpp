#pragma once

#include <array>

namespace hexapod {

// math.pi as an IEEE-754 double, spelled out so the port does not depend on
// M_PI being visible (MinGW hides it without _USE_MATH_DEFINES).
inline constexpr double kPi = 3.14159265358979323846;

// CPython's math.degrees multiplies by a precomputed 180/pi rather than
// dividing by pi, and the two disagree in the last ulp about a quarter of the
// time. That never reached the rounded integer over 400k samples, but matching
// the constant costs nothing and removes the question.
inline constexpr double kRadToDeg = 180.0 / kPi;

inline constexpr int kLegCount = 6;

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

// Joint angles are integers, not an approximation of them: the Python IK
// returns round(degrees(...)), so the quantisation happens before anything
// downstream sees the value.
struct JointAngles {
    int coxa{};
    int femur{};
    int tibia{};
};

using FootPositions = std::array<Vec3, kLegCount>;
using LegAngles = std::array<JointAngles, kLegCount>;

}  // namespace hexapod
