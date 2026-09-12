#include "hexapod/paced_bus.hpp"

namespace hexapod {

namespace {

constexpr long kNanosPerSecond = 1000000000L;

void add_micros(timespec& t, long micros)
{
    t.tv_nsec += micros * 1000L;
    while (t.tv_nsec >= kNanosPerSecond) {
        t.tv_nsec -= kNanosPerSecond;
        t.tv_sec += 1;
    }
}

long micros_between(const timespec& from, const timespec& to)
{
    const long seconds = static_cast<long>(to.tv_sec - from.tv_sec);
    return seconds * 1000000L + (to.tv_nsec - from.tv_nsec) / 1000L;
}

bool is_after(const timespec& a, const timespec& b)
{
    if (a.tv_sec != b.tv_sec) {
        return a.tv_sec > b.tv_sec;
    }
    return a.tv_nsec > b.tv_nsec;
}

}  // namespace

PacedBus::PacedBus(ServoBus& inner, long period_us)
    : inner_(inner), period_us_(period_us)
{
}

void PacedBus::set_angle(int channel, int angle)
{
    inner_.set_angle(channel, angle);
}

void PacedBus::commit()
{
    inner_.commit();
    wait_for_next_frame();
}

void PacedBus::on_unreachable()
{
    inner_.on_unreachable();
    wait_for_next_frame();
}

void PacedBus::reset()
{
    started_ = false;
}

void PacedBus::wait_for_next_frame()
{
    ++frames_;

    if (!started_) {
        clock_gettime(CLOCK_MONOTONIC, &deadline_);
        started_ = true;
    }

    add_micros(deadline_, period_us_);

    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if (is_after(now, deadline_)) {
        // The frame took longer than its slot. Count it and restart the
        // schedule from now, rather than trying to catch up -- chasing a
        // missed deadline just runs the next frames back to back.
        const long late_us = micros_between(deadline_, now);
        ++overruns_;
        if (late_us > worst_overrun_us_) {
            worst_overrun_us_ = late_us;
        }
        deadline_ = now;
        return;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline_, nullptr);
}

}  // namespace hexapod
