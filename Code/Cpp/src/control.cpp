#include "hexapod/control.hpp"

#include "hexapod/kinematics.hpp"
#include "hexapod/pyround.hpp"

#include <cmath>
#include <fstream>

namespace hexapod {

using kinematics::map_value;
using kinematics::restrict_value;

namespace {

// Nominal foot positions in the body frame, mm. Corner legs sit at bearing
// +/-54 and +/-126 at radius ~234; middle legs at 0 and 180 at radius 225.
constexpr double kStanceX[kLegCount] = {137.1, 225.0, 137.1, -137.1, -225.0, -137.1};
constexpr double kStanceY[kLegCount] = {189.4, 0.0, -189.4, -189.4, 0.0, 189.4};

// Vertical drop from the body plane to the coxa pivot.
constexpr double kCoxaZOffset = 14.0;

// The stance that the calibration routine measures against.
constexpr Vec3 kNominalFoot{140.0, 0.0, 0.0};

struct Mat3 {
    double m[3][3]{};
};

// Summation runs k = 0,1,2 left to right, matching how numpy accumulates a
// 3x3 product. Any other order is a different floating-point result.
Mat3 multiply(const Mat3& a, const Mat3& b)
{
    Mat3 out;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out.m[i][j] = a.m[i][0] * b.m[0][j] +
                          a.m[i][1] * b.m[1][j] +
                          a.m[i][2] * b.m[2][j];
        }
    }
    return out;
}

Vec3 multiply(const Mat3& a, const Vec3& v)
{
    return Vec3{
        a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
        a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
        a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z,
    };
}

}  // namespace

bool read_calibration_file(const std::string& path, FootPositions* out,
                           std::string* error)
{
    std::ifstream file(path);
    if (!file) {
        if (error != nullptr) {
            *error = "cannot open " + path;
        }
        return false;
    }
    for (int i = 0; i < kLegCount; ++i) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!(file >> x >> y >> z)) {
            if (error != nullptr) {
                *error = "short read in " + path + " at row " + std::to_string(i);
            }
            return false;
        }
        (*out)[i] = Vec3{x, y, z};
    }
    return true;
}

const std::array<LegMount, kLegCount>& Control::mounts()
{
    static const std::array<LegMount, kLegCount> table = {{
        {54.0, 94.0},
        {0.0, 85.0},
        {-54.0, 94.0},
        {-126.0, 94.0},
        {180.0, 85.0},
        {126.0, 94.0},
    }};
    return table;
}

const std::array<LegChannels, kLegCount>& Control::channel_map()
{
    static const std::array<LegChannels, kLegCount> table = {{
        {0, 15, 14, 13},
        {1, 12, 11, 10},
        {2, 9, 8, 31},
        {5, 16, 17, 18},
        {4, 19, 20, 21},
        {3, 22, 23, 27},
    }};
    return table;
}

JointAngles Control::solve_leg(const Vec3& leg_position)
{
    // The axis shuffle. See the warning in kinematics.hpp.
    return kinematics::coordinate_to_angle(-leg_position.z, leg_position.x, leg_position.y);
}

Control::Control(ServoBus& bus, const FootPositions& calibration_points)
    : bus_(bus), calibration_points_(calibration_points)
{
    for (int i = 0; i < kLegCount; ++i) {
        body_points_[i] = Vec3{kStanceX[i], kStanceY[i], body_height_};
        leg_positions_[i] = kNominalFoot;
    }
    calibrate();
    set_leg_angles();
}

void Control::calibrate()
{
    // Resetting the stance here is a side effect, but the original does it too
    // and later calls depend on it.
    for (int i = 0; i < kLegCount; ++i) {
        leg_positions_[i] = kNominalFoot;
    }

    LegAngles measured{};
    for (int i = 0; i < kLegCount; ++i) {
        measured[i] = solve_leg(calibration_points_[i]);
    }
    for (int i = 0; i < kLegCount; ++i) {
        current_angles_[i] = solve_leg(leg_positions_[i]);
    }

    // Calibrate in Cartesian, correct in joint space: store the per-joint
    // angular difference between where the foot actually is and where the
    // nominal stance says it should be.
    for (int i = 0; i < kLegCount; ++i) {
        calibration_angles_[i].coxa = measured[i].coxa - current_angles_[i].coxa;
        calibration_angles_[i].femur = measured[i].femur - current_angles_[i].femur;
        calibration_angles_[i].tibia = measured[i].tibia - current_angles_[i].tibia;
    }
}

bool Control::check_point_validity() const
{
    for (int i = 0; i < kLegCount; ++i) {
        const Vec3& p = leg_positions_[i];
        const double length = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (length > kinematics::kMaxReach || length < kinematics::kMinReach) {
            return false;
        }
    }
    return true;
}

void Control::set_leg_angles()
{
    if (!check_point_validity()) {
        bus_.on_unreachable();
        return;
    }

    for (int i = 0; i < kLegCount; ++i) {
        current_angles_[i] = solve_leg(leg_positions_[i]);
    }

    // Legs 0-2 and 3-5 are mirrored, and the mirror is not a clean sign flip:
    // the femur picks up 90 - x on one side and 90 + x on the other, and the
    // tibia is inverted through 180 on the far side only.
    for (int i = 0; i < 3; ++i) {
        JointAngles& near_leg = current_angles_[i];
        JointAngles& far_leg = current_angles_[i + 3];
        const JointAngles& near_offset = calibration_angles_[i];
        const JointAngles& far_offset = calibration_angles_[i + 3];

        near_leg.coxa = restrict_value(near_leg.coxa + near_offset.coxa, 0, 180);
        near_leg.femur = restrict_value(90 - (near_leg.femur + near_offset.femur), 0, 180);
        near_leg.tibia = restrict_value(near_leg.tibia + near_offset.tibia, 0, 180);

        far_leg.coxa = restrict_value(far_leg.coxa + far_offset.coxa, 0, 180);
        far_leg.femur = restrict_value(90 + far_leg.femur + far_offset.femur, 0, 180);
        far_leg.tibia = restrict_value(180 - (far_leg.tibia + far_offset.tibia), 0, 180);
    }

    for (const LegChannels& channels : channel_map()) {
        const JointAngles& angles = current_angles_[channels.leg];
        bus_.set_angle(channels.coxa, angles.coxa);
        bus_.set_angle(channels.femur, angles.femur);
        bus_.set_angle(channels.tibia, angles.tibia);
    }
    bus_.commit();
}

void Control::transform_coordinates(const FootPositions& points)
{
    const std::array<LegMount, kLegCount>& table = mounts();
    for (int i = 0; i < kLegCount; ++i) {
        // Spelled (deg / 180) * pi, left to right, exactly as the original has
        // it. The intermediate is a different double than deg * (pi / 180).
        const double bearing = table[i].bearing_deg / 180.0 * kPi;
        const double cos_b = std::cos(bearing);
        const double sin_b = std::sin(bearing);
        leg_positions_[i].x = points[i].x * cos_b + points[i].y * sin_b - table[i].x_offset;
        leg_positions_[i].y = -points[i].x * sin_b + points[i].y * cos_b;
        leg_positions_[i].z = points[i].z - kCoxaZOffset;
    }
}

void Control::move_position(int x, int y, int z)
{
    FootPositions points = body_points_;
    for (int i = 0; i < kLegCount; ++i) {
        points[i].x = body_points_[i].x - x;
        points[i].y = body_points_[i].y - y;
        points[i].z = static_cast<double>(-30 - z);
        body_height_ = points[i].z;
        body_points_[i].z = points[i].z;
    }
    transform_coordinates(points);
    set_leg_angles();
}

FootPositions Control::calculate_posture_balance(double roll, double pitch, double yaw) const
{
    // numpy evaluates (array * pi) / 180, which is not the same double as
    // (pi / 180) * array. Keep this ordering.
    const double roll_angle = (roll * kPi) / 180.0;
    const double pitch_angle = (pitch * kPi) / 180.0;
    const double yaw_angle = (yaw * kPi) / 180.0;

    // The axis naming in the original is crossed: rotation_x is driven by
    // pitch and rotation_y by roll, and rotation_y carries the sign convention
    // of a negative rotation. Reproduced rather than corrected.
    Mat3 rotation_x;
    rotation_x.m[0][0] = 1.0;
    rotation_x.m[1][1] = std::cos(pitch_angle);
    rotation_x.m[1][2] = -std::sin(pitch_angle);
    rotation_x.m[2][1] = std::sin(pitch_angle);
    rotation_x.m[2][2] = std::cos(pitch_angle);

    Mat3 rotation_y;
    rotation_y.m[0][0] = std::cos(roll_angle);
    rotation_y.m[0][2] = -std::sin(roll_angle);
    rotation_y.m[1][1] = 1.0;
    rotation_y.m[2][0] = std::sin(roll_angle);
    rotation_y.m[2][2] = std::cos(roll_angle);

    Mat3 rotation_z;
    rotation_z.m[0][0] = std::cos(yaw_angle);
    rotation_z.m[0][1] = -std::sin(yaw_angle);
    rotation_z.m[1][0] = std::sin(yaw_angle);
    rotation_z.m[1][1] = std::cos(yaw_angle);
    rotation_z.m[2][2] = 1.0;

    const Mat3 rotation = multiply(multiply(rotation_x, rotation_y), rotation_z);

    // The original also builds a body_structure matrix here and never uses it.
    // Omitted, because it cannot affect the result.
    FootPositions out{};
    for (int i = 0; i < kLegCount; ++i) {
        const Vec3 footpoint{kStanceX[i], kStanceY[i], 0.0};
        const Vec3 rotated = multiply(rotation, footpoint);
        out[i] = Vec3{rotated.x, rotated.y, body_height_ + rotated.z};
    }
    return out;
}

void Control::run_gait(const GaitCommand& command, double step_height)
{
    const int x = restrict_value(command.x, -35, 35);
    const int y = restrict_value(command.y, -35, 35);

    // Frame count, inversely proportional to the requested speed: the stride
    // is fixed, so fewer frames means a larger step per frame.
    const int frames = (command.gait == 1)
        ? py_round(map_value(command.speed, 2.0, 10.0, 126.0, 22.0))
        : py_round(map_value(command.speed, 2.0, 10.0, 171.0, 45.0));

    const double lift_step = step_height / frames;

    FootPositions points = body_points_;

    // Per-frame translation for each foot, derived from the UNMUTATED stance.
    const double yaw = command.angle / 180.0 * kPi;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    double xy[kLegCount][2]{};
    for (int i = 0; i < kLegCount; ++i) {
        xy[i][0] = ((points[i].x * cos_yaw + points[i].y * sin_yaw - points[i].x) + x) / frames;
        xy[i][1] = ((-points[i].x * sin_yaw + points[i].y * cos_yaw - points[i].y) + y) / frames;
    }

    if (x == 0 && y == 0 && command.angle == 0) {
        transform_coordinates(points);
        set_leg_angles();
        return;
    }

    const double f = static_cast<double>(frames);

    if (command.gait == 1) {
        // Tripod: even legs against odd legs, half a cycle out of phase.
        for (int j = 0; j < frames; ++j) {
            for (int i = 0; i < 3; ++i) {
                Vec3& a = points[2 * i];
                Vec3& b = points[2 * i + 1];
                const double* xy_a = xy[2 * i];
                const double* xy_b = xy[2 * i + 1];

                if (j < f / 8.0) {
                    a.x -= 4 * xy_a[0];
                    a.y -= 4 * xy_a[1];
                    b.x += 8 * xy_b[0];
                    b.y += 8 * xy_b[1];
                    // Assignment, not increment: the swing group snaps to full
                    // height on the first frame instead of ramping. This is the
                    // main source of jerk in the original gait.
                    b.z = step_height + body_height_;
                } else if (j < f / 4.0) {
                    a.x -= 4 * xy_a[0];
                    a.y -= 4 * xy_a[1];
                    b.z -= lift_step * 8;
                } else if (j < 3 * f / 8.0) {
                    a.z += lift_step * 8;
                    b.x -= 4 * xy_b[0];
                    b.y -= 4 * xy_b[1];
                } else if (j < 5 * f / 8.0) {
                    a.x += 8 * xy_a[0];
                    a.y += 8 * xy_a[1];
                    b.x -= 4 * xy_b[0];
                    b.y -= 4 * xy_b[1];
                } else if (j < 3 * f / 4.0) {
                    a.z -= lift_step * 8;
                    b.x -= 4 * xy_b[0];
                    b.y -= 4 * xy_b[1];
                } else if (j < 7 * f / 8.0) {
                    a.x -= 4 * xy_a[0];
                    a.y -= 4 * xy_a[1];
                    b.z += lift_step * 8;
                } else if (j < f) {
                    a.x -= 4 * xy_a[0];
                    a.y -= 4 * xy_a[1];
                    b.x += 8 * xy_b[0];
                    b.y += 8 * xy_b[1];
                }
            }
            transform_coordinates(points);
            set_leg_angles();
        }
    } else if (command.gait == 2) {
        // Wave: one leg swings at a time, in this order around the body.
        const int order[kLegCount] = {5, 2, 1, 0, 3, 4};
        const int sub_frames = static_cast<int>(f / 6.0);
        const int lift_until = static_cast<int>(f / 18.0);
        const int advance_until = static_cast<int>(f / 9.0);

        for (int i = 0; i < kLegCount; ++i) {
            for (int j = 0; j < sub_frames; ++j) {
                for (int k = 0; k < kLegCount; ++k) {
                    if (order[i] == k) {
                        if (j < lift_until) {
                            points[k].z += 18 * lift_step;
                        } else if (j < advance_until) {
                            points[k].x += 30 * xy[k][0];
                            points[k].y += 30 * xy[k][1];
                        } else if (j < sub_frames) {
                            points[k].z -= 18 * lift_step;
                        }
                    } else {
                        points[k].x -= 2 * xy[k][0];
                        points[k].y -= 2 * xy[k][1];
                    }
                }
                transform_coordinates(points);
                set_leg_angles();
            }
        }
    }
}

}  // namespace hexapod
