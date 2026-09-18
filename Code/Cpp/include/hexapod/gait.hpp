#pragma once

#include "hexapod/control.hpp"
#include "hexapod/types.hpp"

namespace hexapod::gait {

// A second gait engine, written as a function of phase.
//
// Control::run_gait is a faithful port of the original and stays that way --
// the differential test depends on it. But it is built from incremental
// mutations inside a chain of frame-index branches, which means you cannot ask
// where a foot should be at a given moment, only replay from the start, and
// each gait is a separate hand-written branch.
//
// Here a gait is data: a phase offset per leg and a duty factor. Tripod,
// ripple and wave differ only in those numbers. Foot positions are absolute
// functions of phase, so nothing accumulates float error, and adding a gait
// means adding a row rather than writing another loop.

struct Pattern {
    const char* name;
    const char* description;
    double offsets[kLegCount];  // when each leg starts its cycle, 0..1
    double duty;                // fraction of the cycle spent on the ground
};

// Where the body should end up after one cycle.
struct Motion {
    double x{0.0};             // sideways travel per cycle, mm
    double y{35.0};            // forward travel per cycle, mm
    double yaw_deg{0.0};       // rotation per cycle
    double step_height{40.0};  // peak foot lift during swing, mm
};

const Pattern* patterns();
int pattern_count();
const Pattern* find_pattern(const char* name);

// Foot position for one leg at one moment, relative to its neutral stance.
// Exposed so it can be tested directly rather than only through a whole run.
Vec3 foot_offset(const Pattern& pattern, int leg, double phase,
                 const Vec3& travel, double step_height);

// Per-cycle travel vector for each foot, combining translation and rotation.
void compute_travel(const FootPositions& neutral, const Motion& motion,
                    Vec3* travel_out);

// Run whole cycles. Timing belongs to the caller, as with Control::run_gait:
// each frame ends in set_leg_angles, so a PacedBus paces it.
void walk(Control& control, const Pattern& pattern, const Motion& motion,
          int cycles, int frames_per_cycle);

}  // namespace hexapod::gait
