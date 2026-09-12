#include "hexapod/kinematics.hpp"

#include "hexapod/pyround.hpp"

#include <cmath>

namespace hexapod::kinematics {

double restrict_value(double value, double min_value, double max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

int restrict_value(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

double map_value(double value, double from_low, double from_high,
                 double to_low, double to_high)
{
    return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low;
}

JointAngles coordinate_to_angle(double x, double y, double z,
                                double l1, double l2, double l3)
{
    // Coxa angle: rotate the leg plane to contain the target.
    const double a = kPi / 2.0 - std::atan2(z, y);

    // Project the coxa out along that plane, then solve the remaining two
    // links as a planar 2R arm by the law of cosines.
    const double x_3 = 0.0;
    const double x_4 = l1 * std::sin(a);
    const double x_5 = l1 * std::cos(a);
    const double l23 = std::sqrt((z - x_5) * (z - x_5) +
                                 (y - x_4) * (y - x_4) +
                                 (x - x_3) * (x - x_3));

    // Clamping here is what makes unreachable targets silent rather than fatal.
    const double w = restrict_value((x - x_3) / l23, -1.0, 1.0);
    const double v = restrict_value((l2 * l2 + l23 * l23 - l3 * l3) / (2.0 * l2 * l23), -1.0, 1.0);
    const double u = restrict_value((l2 * l2 + l3 * l3 - l23 * l23) / (2.0 * l3 * l2), -1.0, 1.0);

    // The round-to-2-decimals before the inverse trig is not cosmetic: it
    // quantises the result by up to a few hundredths of a degree, and dropping
    // it makes this port disagree with the robot in the field.
    const double b = std::asin(py_round2(w)) - std::acos(py_round2(v));
    const double c = kPi - std::acos(py_round2(u));

    return JointAngles{
        py_round(a * kRadToDeg),
        py_round(b * kRadToDeg),
        py_round(c * kRadToDeg),
    };
}

Vec3 angle_to_coordinate(double a, double b, double c,
                         double l1, double l2, double l3)
{
    a = kPi / 180.0 * a;
    b = kPi / 180.0 * b;
    c = kPi / 180.0 * c;

    return Vec3{
        static_cast<double>(py_round(l3 * std::sin(b + c) + l2 * std::sin(b))),
        static_cast<double>(py_round(l3 * std::sin(a) * std::cos(b + c) +
                                     l2 * std::sin(a) * std::cos(b) +
                                     l1 * std::sin(a))),
        static_cast<double>(py_round(l3 * std::cos(a) * std::cos(b + c) +
                                     l2 * std::cos(a) * std::cos(b) +
                                     l1 * std::cos(a))),
    };
}

}  // namespace hexapod::kinematics
