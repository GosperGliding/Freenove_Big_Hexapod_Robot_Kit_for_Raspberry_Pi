#include "hexapod/dance.hpp"

#include <cmath>
#include <cstring>

namespace hexapod::dance {

namespace {

const Routine kRoutines[] = {
    {"sway", "roll side to side, feet planted"},
    {"twist", "rotate the body about its centre, feet planted"},
    {"circle", "roll and pitch in quadrature; the body describes a cone"},
    {"bob", "rise and dip on the spot"},
    {"pushup", "dip deep and press back up"},
    {"wave", "plant five legs and wave the sixth"},
    {"twerk", "rear-biased double-time bounce with a hip sway"},
};

constexpr int kRoutineCount = static_cast<int>(sizeof(kRoutines) / sizeof(kRoutines[0]));

// Amplitudes are held well inside the reach envelope; test/unit_test.cpp runs
// every routine and asserts no frame is ever rejected as unreachable.
constexpr double kSwayRollDeg = 12.0;
constexpr double kTwistYawDeg = 15.0;
constexpr double kCircleTiltDeg = 10.0;
constexpr double kBobRise = 15.0;     // mm either side of neutral
constexpr double kPushupDip = 30.0;   // mm, downward only
constexpr double kWaveLift = 70.0;    // mm the waving foot is raised
constexpr double kWaveSwing = 40.0;   // mm it sweeps either side
constexpr int kWaveLeg = 0;           // a corner leg: five stay planted

constexpr double kTwerkDip = 22.0;    // mm at the rear
constexpr double kTwerkSway = 10.0;   // mm of fore-aft hip shift

// How much of the dip each leg takes. y is the fore-aft axis: legs 0 and 5
// sit at +189 (front), 1 and 4 at 0 (middle), 2 and 3 at -189 (rear). Loading
// the rear and barely moving the front is what makes the body pitch about its
// front feet rather than see-saw about its centre.
constexpr double kTwerkBias[kLegCount] = {0.15, 0.6, 1.0, 1.0, 0.6, 0.15};

void apply_attitude(Control& control, double roll, double pitch, double yaw)
{
    const FootPositions points = control.calculate_posture_balance(roll, pitch, yaw);
    control.transform_coordinates(points);
    control.set_leg_angles();
}

void apply_points(Control& control, const FootPositions& points)
{
    control.transform_coordinates(points);
    control.set_leg_angles();
}

// Ease in and out so a routine does not start or stop with a jerk.
double eased(double turns)
{
    return std::sin(2.0 * kPi * turns);
}

}  // namespace

const Routine* routines()
{
    return kRoutines;
}

int routine_count()
{
    return kRoutineCount;
}

const Routine* find_routine(const char* name)
{
    for (int i = 0; i < kRoutineCount; ++i) {
        if (std::strcmp(kRoutines[i].name, name) == 0) {
            return &kRoutines[i];
        }
    }
    return nullptr;
}

bool perform(Control& control, const char* name, int frames_per_beat, int repeats)
{
    if (find_routine(name) == nullptr || frames_per_beat < 1) {
        return false;
    }

    const FootPositions neutral = control.body_points();
    const bool is_attitude = std::strcmp(name, "sway") == 0 ||
                             std::strcmp(name, "twist") == 0 ||
                             std::strcmp(name, "circle") == 0;

    for (int beat = 0; beat < repeats; ++beat) {
        for (int frame = 0; frame < frames_per_beat; ++frame) {
            const double turns = static_cast<double>(frame) / frames_per_beat;

            if (std::strcmp(name, "sway") == 0) {
                apply_attitude(control, kSwayRollDeg * eased(turns), 0.0, 0.0);

            } else if (std::strcmp(name, "twist") == 0) {
                apply_attitude(control, 0.0, 0.0, kTwistYawDeg * eased(turns));

            } else if (std::strcmp(name, "circle") == 0) {
                // Quadrature: roll leads pitch by a quarter turn, so the body
                // axis sweeps a cone rather than rocking in one plane.
                const double angle = 2.0 * kPi * turns;
                apply_attitude(control, kCircleTiltDeg * std::sin(angle),
                               kCircleTiltDeg * std::cos(angle), 0.0);

            } else if (std::strcmp(name, "bob") == 0) {
                // Neutral z is negative, so adding to it brings the foot
                // closer to the body and lowers the robot.
                FootPositions points = neutral;
                const double dip = kBobRise * eased(turns);
                for (int leg = 0; leg < kLegCount; ++leg) {
                    points[leg].z = neutral[leg].z + dip;
                }
                apply_points(control, points);

            } else if (std::strcmp(name, "pushup") == 0) {
                // Down and back up once per beat, never above neutral.
                FootPositions points = neutral;
                const double dip = kPushupDip * 0.5 * (1.0 - std::cos(2.0 * kPi * turns));
                for (int leg = 0; leg < kLegCount; ++leg) {
                    points[leg].z = neutral[leg].z + dip;
                }
                apply_points(control, points);

            } else if (std::strcmp(name, "twerk") == 0) {
                // Double time: two dips per beat, so it reads as a bounce
                // rather than the slow rise and fall of bob.
                FootPositions points = neutral;
                const double beat_angle = 4.0 * kPi * turns;
                const double drop = 0.5 * (1.0 - std::cos(beat_angle));  // 0..1
                const double sway = kTwerkSway * std::sin(beat_angle);
                for (int leg = 0; leg < kLegCount; ++leg) {
                    points[leg].z = neutral[leg].z + kTwerkDip * kTwerkBias[leg] * drop;
                    // Quadrature with the drop, so the hips travel through the
                    // bounce instead of bobbing straight up and down.
                    points[leg].y = neutral[leg].y + sway;
                }
                apply_points(control, points);

            } else {  // wave
                FootPositions points = neutral;

                // Raise the waving foot over the first eighth of the beat,
                // wave through the middle, set it down over the last eighth,
                // so it neither snaps up nor stamps down.
                double lift = 1.0;
                if (turns < 0.125) {
                    lift = turns / 0.125;
                } else if (turns > 0.875) {
                    lift = (1.0 - turns) / 0.125;
                }

                points[kWaveLeg].z = neutral[kWaveLeg].z + kWaveLift * lift;
                points[kWaveLeg].y = neutral[kWaveLeg].y +
                                     kWaveSwing * lift * std::sin(6.0 * kPi * turns);

                // Lean away from the lifted corner so the centre of mass stays
                // over the five feet still on the ground.
                const double lean = 12.0 * lift;
                for (int leg = 0; leg < kLegCount; ++leg) {
                    if (leg != kWaveLeg) {
                        points[leg].x += lean;
                        points[leg].y += lean;
                    }
                }
                apply_points(control, points);
            }
        }
    }

    // Finish square, whichever primitive the routine used.
    if (is_attitude) {
        apply_attitude(control, 0.0, 0.0, 0.0);
    } else {
        apply_points(control, neutral);
    }
    return true;
}

}  // namespace hexapod::dance
