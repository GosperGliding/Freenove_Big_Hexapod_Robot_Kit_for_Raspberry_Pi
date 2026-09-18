#include "hexapod/gait.hpp"

#include <cmath>
#include <cstring>

namespace hexapod::gait {

namespace {

// Swing order around the body, alternating sides so the robot is never
// unsupported down one flank. Legs 0-2 are one side, 3-5 the other.
constexpr double k1 = 1.0 / 6.0;
constexpr double k2 = 2.0 / 6.0;
constexpr double k3 = 3.0 / 6.0;
constexpr double k4 = 4.0 / 6.0;
constexpr double k5 = 5.0 / 6.0;

const Pattern kPatterns[] = {
    // Two groups of three, half a cycle apart: the fastest gait, and the one
    // with the least margin -- exactly three feet are down at any moment.
    {"tripod", "3+3 alternating; fastest, least stable",
     {0.0, k3, 0.0, k3, 0.0, k3}, 0.5},

    // Two legs swinging at once, offset by a sixth of a cycle each. A
    // travelling wave around the body: four feet down throughout.
    {"ripple", "travelling wave, two legs swinging; smooth compromise",
     {0.0, k2, k4, k1, k3, k5}, 2.0 / 3.0},

    // One leg at a time, five always planted. Slowest and steadiest.
    {"wave", "one leg at a time; slowest, most stable",
     {0.0, k2, k4, k1, k3, k5}, 5.0 / 6.0},
};

constexpr int kPatternCount = static_cast<int>(sizeof(kPatterns) / sizeof(kPatterns[0]));

}  // namespace

const Pattern* patterns()
{
    return kPatterns;
}

int pattern_count()
{
    return kPatternCount;
}

const Pattern* find_pattern(const char* name)
{
    for (int i = 0; i < kPatternCount; ++i) {
        if (std::strcmp(kPatterns[i].name, name) == 0) {
            return &kPatterns[i];
        }
    }
    return nullptr;
}

void compute_travel(const FootPositions& neutral, const Motion& motion, Vec3* travel_out)
{
    const double yaw = motion.yaw_deg / 180.0 * kPi;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    for (int i = 0; i < kLegCount; ++i) {
        // Rotation moves each foot tangentially, by an amount that depends on
        // where it sits; translation moves them all alike. The sum is how far
        // this foot must travel for the body to complete one cycle.
        const double rotated_x = neutral[i].x * cos_yaw + neutral[i].y * sin_yaw;
        const double rotated_y = -neutral[i].x * sin_yaw + neutral[i].y * cos_yaw;
        travel_out[i].x = (rotated_x - neutral[i].x) + motion.x;
        travel_out[i].y = (rotated_y - neutral[i].y) + motion.y;
        travel_out[i].z = 0.0;
    }
}

Vec3 foot_offset(const Pattern& pattern, int leg, double phase,
                 const Vec3& travel, double step_height)
{
    double p = phase + pattern.offsets[leg];
    p -= std::floor(p);  // wrap to [0, 1)

    double along = 0.0;
    double lift = 0.0;

    if (p < pattern.duty) {
        // Stance: on the ground, driving the body forward. The foot tracks
        // from half a stride ahead of neutral to half a stride behind, which
        // is what carries the body by one travel vector per cycle.
        const double s = p / pattern.duty;
        along = 0.5 - s;
    } else {
        // Swing: off the ground, returning. A half-sine lift leaves and meets
        // the ground with zero vertical velocity, instead of the original's
        // jump to full height on the first frame.
        const double s = (p - pattern.duty) / (1.0 - pattern.duty);
        along = -0.5 + s;
        lift = step_height * std::sin(kPi * s);
    }

    return Vec3{travel.x * along, travel.y * along, lift};
}

void walk(Control& control, const Pattern& pattern, const Motion& motion,
          int cycles, int frames_per_cycle)
{
    if (frames_per_cycle < 1) {
        return;
    }

    // Neutral is sampled once: the gait is relative to whatever stance and
    // ride height are current, and must not drift as it runs.
    const FootPositions neutral = control.body_points();

    Vec3 travel[kLegCount];
    compute_travel(neutral, motion, travel);

    for (int cycle = 0; cycle < cycles; ++cycle) {
        for (int frame = 0; frame < frames_per_cycle; ++frame) {
            const double phase = static_cast<double>(frame) / frames_per_cycle;

            FootPositions points = neutral;
            for (int leg = 0; leg < kLegCount; ++leg) {
                const Vec3 offset =
                    foot_offset(pattern, leg, phase, travel[leg], motion.step_height);
                points[leg].x = neutral[leg].x + offset.x;
                points[leg].y = neutral[leg].y + offset.y;
                // A positive lift raises the foot toward the body, which is
                // what takes it off the ground: neutral z is negative.
                points[leg].z = neutral[leg].z + offset.z;
            }

            control.transform_coordinates(points);
            control.set_leg_angles();
        }
    }
}

}  // namespace hexapod::gait
