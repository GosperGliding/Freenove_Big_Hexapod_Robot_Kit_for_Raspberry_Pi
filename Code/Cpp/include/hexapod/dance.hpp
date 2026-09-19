#pragma once

#include "hexapod/control.hpp"

namespace hexapod::dance {

// Choreography built on the same two primitives the gaits use:
// calculate_posture_balance for body attitude, and direct foot positions for
// everything else.
//
// Most of these keep all six feet planted and move only the body. That is not
// just for looks -- with nothing lifted the support polygon never changes, so
// they stay stable at amplitudes where a gait would be marginal.

struct Routine {
    const char* name;
    const char* description;
};

const Routine* routines();
int routine_count();
const Routine* find_routine(const char* name);

// How much more the rear of the body moves than the front, during `twerk` and
// the twerk phase of `show`.
//
//   0.00  every leg dips equally -- indistinguishable from `bob`
//   0.85  the first version: front legs take 15% of the dip
//   0.90  the default
//   1.00  front legs completely still; the body hinges about them
//
// The differential does more for how the motion reads than the amplitude
// does, which is why it is the knob that got exposed.
inline constexpr double kDefaultTwerkBias = 0.90;

// frames_per_beat sets how long one repetition takes; the caller's ServoBus
// supplies the timing, as everywhere else. Returns false for an unknown name.
//
// Every routine returns the robot to its neutral stance before it ends, so
// they can be chained without leaving the body tilted.
bool perform(Control& control, const char* name, int frames_per_beat, int repeats,
             double twerk_bias = kDefaultTwerkBias);

// How far `circle` will lean, and how far in it draws the feet to afford it.
struct CircleShape {
    double tilt;  // degrees
    double tuck;  // 1.0 is the nominal stance
};

// What `circle` would use at the robot's current ride height. Exposed so the
// adaptive behaviour can be asserted directly, and so a caller can report it.
// Moves nothing.
CircleShape probed_circle_shape(Control& control);

}  // namespace hexapod::dance
