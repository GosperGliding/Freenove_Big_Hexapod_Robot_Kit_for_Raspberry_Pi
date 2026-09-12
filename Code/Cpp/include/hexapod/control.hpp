#pragma once

#include "hexapod/servo_bus.hpp"
#include "hexapod/types.hpp"

#include <array>
#include <string>

namespace hexapod {

// Read point.txt: six rows of whitespace-separated numbers, the measured foot
// positions used to derive the per-joint calibration offsets.
bool read_calibration_file(const std::string& path, FootPositions* out,
                           std::string* error);

struct GaitCommand {
    int gait{1};   // 1 = tripod (3+3), 2 = wave (one leg at a time)
    int x{0};      // body translation per cycle, mm, saturated to +/-35
    int y{0};
    int speed{10}; // 2..10; maps INVERSELY to the frame count
    int angle{0};  // yaw per cycle, degrees
};

// How each leg is mounted on the chassis: bearing from body +x, and the
// distance from body centre out to the coxa pivot.
struct LegMount {
    double bearing_deg;
    double x_offset;
};

// Where each joint lives in the flat 0..31 servo address space, in the exact
// order the original wrote them. The order is legs 1,2,3,6,5,4 -- not 1..6 --
// and leg 3's tibia sits on the other PCA9685 at channel 31. Preserved because
// a differential trace compares the write sequence, not just the final pose.
struct LegChannels {
    int leg;
    int coxa;
    int femur;
    int tibia;
};

class Control {
public:
    // calibration_points are the six measured foot positions from point.txt,
    // nominally {140, 0, 0}.
    Control(ServoBus& bus, const FootPositions& calibration_points);

    // Recompute per-joint calibration offsets and reset the stance to nominal.
    void calibrate();

    // Solve IK for the current leg_positions and push 18 angles to the bus.
    // Writes nothing and calls ServoBus::on_unreachable() if any foot is
    // outside the reach envelope.
    void set_leg_angles();

    bool check_point_validity() const;

    // Body-frame foot targets -> per-leg frames, into leg_positions.
    void transform_coordinates(const FootPositions& points);

    // Translate the body without moving the feet. Mutates body_height and the
    // z of every body_point, so it is order-dependent with respect to gaits.
    void move_position(int x, int y, int z);

    // Foot positions that hold the body at the given attitude.
    FootPositions calculate_posture_balance(double roll, double pitch, double yaw) const;

    // One full gait cycle. Unlike the original this does not sleep: the caller
    // owns the timing policy, which is what makes it testable at full speed and
    // schedulable against a real clock on the robot.
    void run_gait(const GaitCommand& command, double step_height = 40.0);

    const LegAngles& current_angles() const { return current_angles_; }
    const FootPositions& leg_positions() const { return leg_positions_; }
    double body_height() const { return body_height_; }

    static const std::array<LegMount, kLegCount>& mounts();
    static const std::array<LegChannels, kLegCount>& channel_map();

private:
    // The axis shuffle from the original, quarantined in one place:
    // coordinate_to_angle(-leg.z, leg.x, leg.y).
    static JointAngles solve_leg(const Vec3& leg_position);

    ServoBus& bus_;
    FootPositions calibration_points_{};
    FootPositions body_points_{};
    FootPositions leg_positions_{};
    LegAngles calibration_angles_{};
    LegAngles current_angles_{};
    double body_height_{-25.0};
};

}  // namespace hexapod
