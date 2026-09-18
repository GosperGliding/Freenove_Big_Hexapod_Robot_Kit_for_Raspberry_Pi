#include "hexapod/dance.hpp"

#include "hexapod/gait.hpp"

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
    {"bounce", "rear-biased double-time bounce with a hip sway"},
    {"moonwalk", "glide backwards while the feet skim forwards"},
    {"twerk", "four phases: rise circling, turn about, bounce, sink circling"},
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

// The moonwalk illusion is entirely about the feet barely leaving the floor:
// lift them properly and it reads as an ordinary backward walk.
constexpr double kMoonGlide = 26.0;    // mm travelled backwards per beat
constexpr double kMoonSkim = 7.0;      // mm of lift -- just clear of the floor
constexpr double kMoonLeanDeg = 8.0;   // forward lean, against the direction of travel

// The staged routine. A bigger front-to-back differential than `bounce`, and
// a slower circle over the top of it.
constexpr double kShowDip = 28.0;
constexpr double kShowBias[kLegCount] = {0.0, 0.5, 1.0, 1.0, 0.5, 0.0};
constexpr double kShowTiltDeg = 9.0;
constexpr int kShowRideHeight = 60;    // stands tall for the middle of the routine
constexpr int kShowFloorHeight = -20;  // and finishes down on the floor
// 20 degrees a cycle makes the half turn in 9, and the turn runs at half the
// frame count of the rest: at 18 degrees over full-length cycles the turn
// alone took 18 seconds and swamped the routine. Larger yaw per cycle would be
// quicker still, but the stride it implies eats the reach margin -- at 30
// degrees a foot swings 60 mm from neutral and comes within 8 mm of the limit.
constexpr double kShowYawPerCycle = 20.0;
constexpr int kShowMinTurnFrames = 12;

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

// move_position takes the inverse of body height: body_height = -30 - z.
int current_ride_height(const Control& control)
{
    return static_cast<int>(std::lround(-30.0 - control.body_height()));
}

// Change ride height a millimetre at a time with a circle laid over the top,
// so the robot rises or sinks in a spiral rather than straight up and down.
// Each step is two frames: move_position applies the new height, then the
// attitude is applied on top of it.
void spiral_to_height(Control& control, int target, double circles)
{
    const int start = current_ride_height(control);
    const int span = (target >= start) ? (target - start) : (start - target);
    if (span == 0) {
        return;
    }
    const int step = (target >= start) ? 1 : -1;

    for (int z = start; ; z += step) {
        control.move_position(0, 0, z);

        const double progress = static_cast<double>((z - start) * step) / span;
        const double angle = 2.0 * kPi * circles * progress;
        apply_attitude(control, kShowTiltDeg * std::sin(angle),
                       kShowTiltDeg * std::cos(angle), 0.0);

        if (z == target) {
            break;
        }
    }
}

// A backward glide with the feet barely clearing the floor.
//
// A hexapod cannot do the actual illusion -- that needs one foot sliding while
// the other takes weight, and six legs in a wave have no such moment. What
// does survive is the part the eye reads: feet skimming rather than stepping,
// and a forward lean that fights the direction of travel.
void moonwalk(Control& control, int frames_per_beat, int repeats)
{
    // Wave order: one leg off the floor at a time, five always planted.
    const double offsets[kLegCount] = {0.0, 2.0 / 6.0, 4.0 / 6.0,
                                       1.0 / 6.0, 3.0 / 6.0, 5.0 / 6.0};
    constexpr double duty = 5.0 / 6.0;

    for (int beat = 0; beat < repeats; ++beat) {
        for (int frame = 0; frame < frames_per_beat; ++frame) {
            const double phase = static_cast<double>(frame) / frames_per_beat;

            // Lean first, then add the stride to the leaned stance.
            FootPositions points =
                control.calculate_posture_balance(0.0, kMoonLeanDeg, 0.0);

            for (int leg = 0; leg < kLegCount; ++leg) {
                double p = phase + offsets[leg];
                p -= std::floor(p);

                double along = 0.0;
                double lift = 0.0;
                if (p < duty) {
                    along = 0.5 - p / duty;
                } else {
                    const double s = (p - duty) / (1.0 - duty);
                    along = -0.5 + s;
                    lift = kMoonSkim * std::sin(kPi * s);
                }

                // Negative travel: the body goes backwards.
                points[leg].y += -kMoonGlide * along;
                points[leg].z += lift;
            }
            apply_points(control, points);
        }
    }
    apply_attitude(control, 0.0, 0.0, 0.0);
}

// Four phases: rise in a spiral, turn about, bounce with a slow circle over
// it, then sink back to the floor in a spiral.
void twerk_show(Control& control, int frames_per_beat, int repeats)
{
    spiral_to_height(control, kShowRideHeight, 2.0);

    // Half turn on the spot: no translation, pure yaw, on the ripple pattern
    // so four feet stay down throughout.
    const gait::Pattern* ripple = gait::find_pattern("ripple");
    if (ripple != nullptr) {
        gait::Motion motion;
        motion.x = 0.0;
        motion.y = 0.0;
        motion.yaw_deg = kShowYawPerCycle;
        motion.step_height = 35.0;
        const int cycles =
            static_cast<int>(std::lround(180.0 / kShowYawPerCycle));
        const int turn_frames =
            (frames_per_beat / 2 > kShowMinTurnFrames) ? frames_per_beat / 2
                                                       : kShowMinTurnFrames;
        gait::walk(control, *ripple, motion, cycles, turn_frames);
    }

    const int beats = (repeats > 0) ? repeats : 1;
    for (int beat = 0; beat < beats; ++beat) {
        for (int frame = 0; frame < frames_per_beat; ++frame) {
            const double turns = static_cast<double>(frame) / frames_per_beat;

            // Bounce at double time, circle once across the whole phase, so
            // the two rates read as separate motions rather than one.
            const double drop = 0.5 * (1.0 - std::cos(4.0 * kPi * turns));
            const double slow = 2.0 * kPi * (beat + turns) / beats;

            FootPositions points = control.calculate_posture_balance(
                kShowTiltDeg * std::sin(slow), kShowTiltDeg * std::cos(slow), 0.0);
            for (int leg = 0; leg < kLegCount; ++leg) {
                points[leg].z += kShowDip * kShowBias[leg] * drop;
            }
            apply_points(control, points);
        }
    }

    spiral_to_height(control, kShowFloorHeight, 2.0);
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

    // Staged routines run their own phases rather than one repeating beat, so
    // they take over before the per-frame loop below.
    if (std::strcmp(name, "moonwalk") == 0) {
        moonwalk(control, frames_per_beat, repeats);
        return true;
    }
    if (std::strcmp(name, "twerk") == 0) {
        twerk_show(control, frames_per_beat, repeats);
        return true;
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

            } else if (std::strcmp(name, "bounce") == 0) {
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
