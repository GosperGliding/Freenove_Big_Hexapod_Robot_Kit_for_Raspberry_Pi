#pragma once

#include "hexapod/servo_bus.hpp"

#include <time.h>

namespace hexapod {

// Holds a fixed frame period, wrapping another ServoBus.
//
// Control deliberately owns no timing policy, so pacing goes here rather than
// inside run_gait. The deadline is absolute and advances by exactly one period
// each frame, so a slow frame does not push every later frame late -- which is
// what a naive sleep(period) at the bottom of a loop does.
//
// Skipped frames are paced too. The original sleeps once per frame regardless
// of whether the pose was reachable, so an unreachable frame still consumes
// its slot; without that, an out-of-range stretch would silently speed the
// gait up.
//
// Linux only: uses clock_nanosleep with TIMER_ABSTIME.
class PacedBus final : public ServoBus {
public:
    PacedBus(ServoBus& inner, long period_us);

    void set_angle(int channel, int angle) override;
    void commit() override;
    void on_unreachable() override;

    // Start the schedule fresh; call before a run so that setup time is not
    // charged against the first frame.
    void reset();

    long period_us() const { return period_us_; }
    long frames() const { return frames_; }
    long overruns() const { return overruns_; }
    long worst_overrun_us() const { return worst_overrun_us_; }

private:
    void wait_for_next_frame();

    ServoBus& inner_;
    long period_us_;
    timespec deadline_{};
    bool started_{false};
    long frames_{0};
    long overruns_{0};
    long worst_overrun_us_{0};
};

}  // namespace hexapod
