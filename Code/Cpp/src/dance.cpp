#include "hexapod/dance.hpp"

#include "hexapod/gait.hpp"
#include "hexapod/kinematics.hpp"

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
    {"moonwalk", "glide backwards while the feet skim forwards"},
    {"show", "four phases: rise circling, turn about, twerk, sink circling"},
};

constexpr int kRoutineCount = static_cast<int>(sizeof(kRoutines) / sizeof(kRoutines[0]));

// Fixed amplitudes, chosen to sit inside the reach envelope at every ride
// height the tool allows. test/unit_test.cpp runs every routine at heights 0,
// 40 and 80 and asserts no frame is ever rejected as unreachable.
//
// `circle` is the exception: it probes instead, because a single number safe
// at height 80 throws away most of the range available lower down.
constexpr double kSwayRollDeg = 12.0;
constexpr double kTwistYawDeg = 15.0;
// The ceiling `circle` probes down from.
//
// At ride heights 40 and 80 the legs bind first, at 36 and 24 degrees. Down
// at 0 this cap is what stops it -- but 60 degrees of body lean is already
// past what is sensible to ask of a machine whose feet are merely resting on
// the floor, so the cap stands as a judgement rather than a measurement.
constexpr double kCircleTiltCeiling = 60.0;

// 8 mm short of the 233 mm the legs can physically span, so a probe that just
// passes still has somewhere to go.
constexpr double kSafeReach = 225.0;

// How far in the feet may be drawn. Tucking buys tilt, but it also shrinks
// the polygon the centre of mass has to stay inside while the body leans --
// so this stops well short of what the joints would tolerate.
constexpr double kMinTuck = 0.86;
constexpr double kBobRise = 15.0;     // mm either side of neutral
constexpr double kPushupDip = 30.0;   // mm, downward only
constexpr double kWaveLift = 70.0;    // mm the waving foot is raised
constexpr double kWaveSwing = 40.0;   // mm it sweeps either side
constexpr int kWaveLeg = 0;           // a corner leg: five stay planted

constexpr double kTwerkDip = 28.0;    // mm at the rear
constexpr double kTwerkSway = 10.0;   // mm of fore-aft hip shift

// How much of the dip each leg takes. y is the fore-aft axis: legs 0 and 5
// sit at +189 (front), 1 and 4 at 0 (middle), 2 and 3 at -189 (rear).
//
// The front pair take none of it, so the body pitches about its front feet as
// a hinge instead of see-sawing about its centre. That differential is what
// makes the motion read as the rear moving rather than the whole robot
// rocking -- it matters more than the amplitude does.
constexpr double kTwerkBias[kLegCount] = {0.0, 0.5, 1.0, 1.0, 0.5, 0.0};

// The moonwalk illusion is entirely about the feet barely leaving the floor:
// lift them properly and it reads as an ordinary backward walk.
constexpr double kMoonGlide = 26.0;    // mm travelled backwards per beat
constexpr double kMoonSkim = 7.0;      // mm of lift -- just clear of the floor
constexpr double kMoonLeanDeg = 8.0;   // forward lean, against the direction of travel

// The staged routine reuses the twerk dip and bias, and lays a slow circle
// over the top of them.
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

// Draw the stance in toward the body centre. `tuck` of 1.0 is nominal.
//
// A rotation is linear, so scaling the footpoints before rotating gives the
// same answer as scaling the rotated result about the body centre. That means
// this can reuse calculate_posture_balance rather than duplicating its
// rotation maths, which is the part worth not copying.
FootPositions tucked(const FootPositions& points, double body_height, double tuck)
{
    FootPositions out = points;
    for (int i = 0; i < kLegCount; ++i) {
        out[i].x = points[i].x * tuck;
        out[i].y = points[i].y * tuck;
        out[i].z = body_height + (points[i].z - body_height) * tuck;
    }
    return out;
}

bool sweep_fits(Control& control, double tilt, double tuck)
{
    const double body_height = control.body_height();
    for (int step = 0; step < 360; step += 15) {
        const double angle = static_cast<double>(step) / 180.0 * kPi;
        control.transform_coordinates(tucked(
            control.calculate_posture_balance(tilt * std::sin(angle),
                                              tilt * std::cos(angle), 0.0),
            body_height, tuck));

        for (const Vec3& p : control.leg_positions()) {
            const double reach = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            // Both bounds matter. The upper one is obvious; the lower one is
            // what stops a deep tuck, by folding the leg in so tight the foot
            // comes closer to the coxa than the joints allow.
            if (reach > kSafeReach || reach < kinematics::kMinReach) {
                return false;
            }
        }
    }
    return true;
}

// The widest cone the legs will carry at the current ride height, searching
// over both how far the body leans and how far the feet are drawn in.
//
// Tucking is not a free win. Pulling the feet in unloads the outer reach
// limit, but past roughly 0.85 the inner limit takes over and the available
// tilt collapses -- at ride height 0 it goes 47 degrees at nominal, 60 at
// 0.9, then 11 at 0.8. Hence a search rather than a constant.
//
// Probing moves nothing: transform_coordinates only writes leg_positions, and
// the servos see nothing until set_leg_angles is called.
//
// Deliberately not using Control::check_point_validity -- its upper bound is
// 248 mm, looser than the 233 the legs can actually span, so it would wave
// through poses the IK then silently clamps.
CircleShape probe_circle(Control& control, double ceiling)
{
    CircleShape best{2.0, 1.0};

    for (double tuck = 1.0; tuck >= kMinTuck - 1e-9; tuck -= 0.02) {
        for (double tilt = ceiling; tilt > best.tilt; tilt -= 1.0) {
            if (sweep_fits(control, tilt, tuck)) {
                best.tilt = tilt;
                best.tuck = tuck;
                break;
            }
        }
    }
    return best;
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
                points[leg].z += kTwerkDip * kTwerkBias[leg] * drop;
            }
            apply_points(control, points);
        }
    }

    spiral_to_height(control, kShowFloorHeight, 2.0);
}

}  // namespace

CircleShape probed_circle_shape(Control& control)
{
    return probe_circle(control, kCircleTiltCeiling);
}

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
    if (std::strcmp(name, "show") == 0) {
        twerk_show(control, frames_per_beat, repeats);
        return true;
    }

    // Probed once, before any frame is emitted: the answer depends on ride
    // height, which does not change during a routine.
    const CircleShape circle = (std::strcmp(name, "circle") == 0)
                                   ? probe_circle(control, kCircleTiltCeiling)
                                   : CircleShape{0.0, 1.0};

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
                apply_points(control,
                             tucked(control.calculate_posture_balance(
                                        circle.tilt * std::sin(angle),
                                        circle.tilt * std::cos(angle), 0.0),
                                    control.body_height(), circle.tuck));

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
