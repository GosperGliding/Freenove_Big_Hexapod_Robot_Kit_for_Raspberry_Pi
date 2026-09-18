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

// frames_per_beat sets how long one repetition takes; the caller's ServoBus
// supplies the timing, as everywhere else. Returns false for an unknown name.
//
// Every routine returns the robot to its neutral stance before it ends, so
// they can be chained without leaving the body tilted.
bool perform(Control& control, const char* name, int frames_per_beat, int repeats);

}  // namespace hexapod::dance
