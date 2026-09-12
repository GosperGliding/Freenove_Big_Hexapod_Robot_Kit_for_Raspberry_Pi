#pragma once

#include "hexapod/types.hpp"

namespace hexapod::kinematics {

// Link lengths in mm, measured outward from the body.
inline constexpr double kCoxa = 33.0;   // l1 in the original
inline constexpr double kFemur = 90.0;  // l2
inline constexpr double kTibia = 110.0; // l3

// Reach envelope enforced by the original's check_point_validity. Note the
// upper bound is looser than the arm can actually reach: l1 + l2 + l3 = 233.
inline constexpr double kMinReach = 90.0;
inline constexpr double kMaxReach = 248.0;

// Inverse kinematics for one leg.
//
// WARNING -- axis convention. The caller in the original passes the leg-frame
// position shuffled: coordinate_to_angle(-leg.z, leg.x, leg.y). This function's
// (x, y, z) are therefore NOT the leg frame's. The shuffle is preserved in
// Control::solve_leg rather than being quietly normalised here, because
// "cleaning it up" changes which way the robot walks.
//
// Unreachable targets do not fail: the inverse-trig arguments are clamped, so
// the result is a plausible straight-leg pose. Guard with Control's reach check.
JointAngles coordinate_to_angle(double x, double y, double z,
                                double l1 = kCoxa,
                                double l2 = kFemur,
                                double l3 = kTibia);

// Forward kinematics. Present in the original but never called by it; kept
// because it makes IK round-trip testing possible. Components are whole
// numbers -- the original rounds before returning.
Vec3 angle_to_coordinate(double a, double b, double c,
                         double l1 = kCoxa,
                         double l2 = kFemur,
                         double l3 = kTibia);

double restrict_value(double value, double min_value, double max_value);
int restrict_value(int value, int min_value, int max_value);
double map_value(double value, double from_low, double from_high,
                 double to_low, double to_high);

}  // namespace hexapod::kinematics
