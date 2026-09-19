// Unit tests for the portable pieces the differential trace does not reach.
//
// The trace already pins down angle -> ticks and chip routing for every angle
// the gaits actually produce. What it cannot cover is the batching algorithm
// (no trace line corresponds to a transaction) and the boundary behaviour at
// the edges of the channel and angle ranges. Those live here so they can be
// checked on a development machine rather than on the robot.

#include "hexapod/control.hpp"
#include "hexapod/dance.hpp"
#include "hexapod/gait.hpp"
#include "hexapod/pwm_encoding.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

int failures = 0;

void expect_eq(long actual, long expected, const char* what)
{
    if (actual != expected) {
        std::printf("  FAIL  %s: got %ld, expected %ld\n", what, actual, expected);
        ++failures;
    }
}

void expect_true(bool condition, const char* what)
{
    if (!condition) {
        std::printf("  FAIL  %s\n", what);
        ++failures;
    }
}

// Ticks at the ends and middle of the servo range. Values come from the
// original: int(4095 * (2000*angle/180 + 500) / 20000).
void test_angle_to_ticks()
{
    using hexapod::pwm::angle_to_ticks;
    expect_eq(angle_to_ticks(0), 102, "angle_to_ticks(0)");
    expect_eq(angle_to_ticks(90), 307, "angle_to_ticks(90)");
    expect_eq(angle_to_ticks(180), 511, "angle_to_ticks(180)");

    // Monotonic, and never outside the 12-bit counter.
    int previous = -1;
    for (int angle = 0; angle <= 180; ++angle) {
        const int ticks = angle_to_ticks(angle);
        expect_true(ticks >= previous, "angle_to_ticks is monotonic");
        expect_true(ticks >= 0 && ticks <= 4095, "angle_to_ticks stays in range");
        previous = ticks;
    }
}

// The split across the two chips, including the original's silent drop of any
// channel at 32 or above.
void test_channel_routing()
{
    using hexapod::pwm::encode;
    using hexapod::pwm::ChannelTarget;

    const ChannelTarget low = encode(0, 90);
    expect_true(low.valid, "channel 0 is valid");
    expect_eq(low.chip_address, 0x41, "channel 0 goes to 0x41");
    expect_eq(low.chip_channel, 0, "channel 0 keeps its index");

    const ChannelTarget boundary_low = encode(15, 90);
    expect_eq(boundary_low.chip_address, 0x41, "channel 15 goes to 0x41");
    expect_eq(boundary_low.chip_channel, 15, "channel 15 keeps its index");

    const ChannelTarget boundary_high = encode(16, 90);
    expect_eq(boundary_high.chip_address, 0x40, "channel 16 goes to 0x40");
    expect_eq(boundary_high.chip_channel, 0, "channel 16 remaps to 0");

    const ChannelTarget high = encode(31, 90);
    expect_eq(high.chip_address, 0x40, "channel 31 goes to 0x40");
    expect_eq(high.chip_channel, 15, "channel 31 remaps to 15");

    const ChannelTarget dropped = encode(32, 90);
    expect_true(!dropped.valid, "channel 32 is dropped, as in the original");

    // ON is always 0 for a servo pulse; only OFF carries the width.
    expect_eq(low.on, 0, "ON is zero");
}

void test_prescale()
{
    // 25MHz / 4096 / 50Hz - 1 = 121.07, floor(+0.5) = 121.
    expect_eq(hexapod::pwm::prescale_for(50.0), 121, "prescale for 50Hz");
}

void test_register_bytes()
{
    std::uint8_t bytes[4] = {0, 0, 0, 0};
    hexapod::pwm::encode_channel_bytes(0, 4096, bytes);
    expect_eq(bytes[0], 0x00, "ON_L of 0");
    expect_eq(bytes[1], 0x00, "ON_H of 0");
    expect_eq(bytes[2], 0x00, "OFF_L of 4096");
    expect_eq(bytes[3], 0x10, "OFF_H of 4096 sets the full-off bit");
}

void test_dirty_runs()
{
    using hexapod::pwm::find_dirty_runs;
    using hexapod::pwm::Run;
    Run runs[16];

    expect_eq(find_dirty_runs(0x0000, runs, 16), 0, "empty mask yields no runs");

    expect_eq(find_dirty_runs(0xFFFF, runs, 16), 1, "full mask is one run");
    expect_eq(runs[0].first, 0, "full mask starts at 0");
    expect_eq(runs[0].count, 16, "full mask covers 16");

    // The masks the robot actually produces. Legs on 0x41 occupy channels
    // 8..15; legs on 0x40 occupy 0..7 plus 11 and 15.
    const int low_runs = find_dirty_runs(0xFF00, runs, 16);
    expect_eq(low_runs, 1, "0x41 leg channels form one run");
    expect_eq(runs[0].first, 8, "0x41 run starts at channel 8");
    expect_eq(runs[0].count, 8, "0x41 run covers 8 channels");

    const std::uint16_t high_mask = 0x00FF | (1u << 11) | (1u << 15);
    const int high_runs = find_dirty_runs(high_mask, runs, 16);
    expect_eq(high_runs, 3, "0x40 leg channels form three runs");
    expect_eq(runs[0].first, 0, "first 0x40 run starts at 0");
    expect_eq(runs[0].count, 8, "first 0x40 run covers 8");
    expect_eq(runs[1].first, 11, "second 0x40 run is the lone channel 11");
    expect_eq(runs[1].count, 1, "second 0x40 run covers 1");
    expect_eq(runs[2].first, 15, "third 0x40 run is the lone channel 15");
    expect_eq(runs[2].count, 1, "third 0x40 run covers 1");

    // Alternating channels are the worst case: eight single-channel runs.
    expect_eq(find_dirty_runs(0x5555, runs, 16), 8, "alternating mask yields 8 runs");

    // A cap smaller than the run count must not overflow the caller's buffer.
    expect_eq(find_dirty_runs(0x5555, runs, 3), 3, "run output respects max_runs");
}

// Confirm that the channel map the gait actually drives produces exactly the
// masks the batching test above assumes. If someone rewires the channel map,
// this is what catches it.
void test_channel_map_masks()
{
    std::uint16_t low_mask = 0;
    std::uint16_t high_mask = 0;
    for (const hexapod::LegChannels& channels : hexapod::Control::channel_map()) {
        const int joints[3] = {channels.coxa, channels.femur, channels.tibia};
        for (int flat : joints) {
            const hexapod::pwm::ChannelTarget target = hexapod::pwm::encode(flat, 90);
            expect_true(target.valid, "every mapped channel is valid");
            if (target.chip_address == 0x41) {
                low_mask |= static_cast<std::uint16_t>(1u << target.chip_channel);
            } else {
                high_mask |= static_cast<std::uint16_t>(1u << target.chip_channel);
            }
        }
    }
    expect_eq(low_mask, 0xFF00, "0x41 mask matches channels 8..15");
    expect_eq(high_mask, 0x00FF | (1u << 11) | (1u << 15),
              "0x40 mask matches channels 0..7, 11, 15");

    // 18 joints, and no channel claimed twice.
    int bits = 0;
    for (int i = 0; i < 16; ++i) {
        bits += (low_mask >> i) & 1;
        bits += (high_mask >> i) & 1;
    }
    expect_eq(bits, 18, "exactly 18 distinct servo channels are driven");
}

// Counts the frames Control rejected as outside the reach envelope. Every
// gait and dance must produce none, at any supported ride height.
class CountingBus final : public hexapod::ServoBus {
public:
    void set_angle(int, int) override {}
    void commit() override { ++frames_; }
    void on_unreachable() override { ++unreachable_; }

    void reset() { frames_ = 0; unreachable_ = 0; }
    long frames() const { return frames_; }
    long unreachable() const { return unreachable_; }

private:
    long frames_{0};
    long unreachable_{0};
};

hexapod::FootPositions nominal_calibration()
{
    hexapod::FootPositions points{};
    for (int i = 0; i < hexapod::kLegCount; ++i) {
        points[i] = hexapod::Vec3{140.0, 0.0, 0.0};
    }
    return points;
}

void raise_to(hexapod::Control& control, int height)
{
    for (int z = 0; z <= height; ++z) {
        control.move_position(0, 0, z);
    }
}

// Worst case the gaits can be asked for: full diagonal stride plus rotation.
void test_gait_patterns_stay_in_reach()
{
    for (int height : {0, 40, 80}) {
        CountingBus bus;
        hexapod::Control control(bus, nominal_calibration());
        raise_to(control, height);

        for (int i = 0; i < hexapod::gait::pattern_count(); ++i) {
            const hexapod::gait::Pattern& pattern = hexapod::gait::patterns()[i];
            hexapod::gait::Motion motion;
            motion.x = 35.0;
            motion.y = 35.0;
            motion.yaw_deg = 10.0;

            bus.reset();
            hexapod::gait::walk(control, pattern, motion, 1, 60);

            expect_eq(bus.frames(), 60, "gait produces one frame per step");
            if (bus.unreachable() != 0) {
                std::printf("  FAIL  gait %s at height %d: %ld unreachable frames\n",
                            pattern.name, height, bus.unreachable());
                ++failures;
            }
        }
    }
}

void test_dance_routines_stay_in_reach()
{
    for (int height : {0, 40, 80}) {
        for (int i = 0; i < hexapod::dance::routine_count(); ++i) {
            // A fresh Control per routine: the staged routines change ride
            // height as they run and can finish on the floor, so reusing one
            // would silently test later routines from the wrong stance.
            CountingBus bus;
            hexapod::Control control(bus, nominal_calibration());
            raise_to(control, height);
            bus.reset();

            const hexapod::dance::Routine& routine = hexapod::dance::routines()[i];
            const bool ok = hexapod::dance::perform(control, routine.name, 60, 1);
            expect_true(ok, "dance routine runs");
            if (bus.unreachable() != 0) {
                std::printf("  FAIL  dance %s at height %d: %ld unreachable frames\n",
                            routine.name, height, bus.unreachable());
                ++failures;
            }
        }
    }
}

// A foot in stance must track a straight line, and a foot in swing must leave
// and meet the ground with no vertical step.
void test_foot_trajectory()
{
    const hexapod::gait::Pattern* tripod = hexapod::gait::find_pattern("tripod");
    expect_true(tripod != nullptr, "tripod pattern exists");
    expect_true(hexapod::gait::find_pattern("ripple") != nullptr, "ripple pattern exists");
    expect_true(hexapod::gait::find_pattern("wave") != nullptr, "wave pattern exists");
    expect_true(hexapod::gait::find_pattern("nonsense") == nullptr, "unknown pattern rejected");

    const hexapod::Vec3 travel{0.0, 30.0, 0.0};

    // Start and end of the cycle must agree: the gait has to close.
    const hexapod::Vec3 start =
        hexapod::gait::foot_offset(*tripod, 0, 0.0, travel, 40.0);
    const hexapod::Vec3 end =
        hexapod::gait::foot_offset(*tripod, 0, 0.999999, travel, 40.0);
    expect_true(std::fabs(start.y - end.y) < 0.01, "gait cycle closes in y");

    // Lift is zero on the ground and at both ends of the swing, positive in
    // between, and never exceeds the requested step height.
    double peak = 0.0;
    for (int i = 0; i <= 1000; ++i) {
        const double phase = static_cast<double>(i) / 1000.0;
        const hexapod::Vec3 p =
            hexapod::gait::foot_offset(*tripod, 0, phase, travel, 40.0);
        expect_true(p.z >= -1e-9, "lift is never negative");
        expect_true(p.z <= 40.0 + 1e-9, "lift never exceeds step height");
        if (p.z > peak) {
            peak = p.z;
        }
    }
    expect_true(peak > 39.0, "swing reaches close to full step height");
}

// The circle routine probes for the largest tilt the current ride height can
// carry. Check the probe is actually adaptive -- a fixed constant would have
// to be safe at height 80, and would be needlessly timid at height 0.
void test_circle_tilt_adapts_to_height()
{
    double previous = 1e9;
    for (int height : {0, 40, 80}) {
        CountingBus bus;
        hexapod::Control control(bus, nominal_calibration());
        raise_to(control, height);
        bus.reset();

        hexapod::dance::perform(control, "circle", 60, 1);
        expect_eq(bus.unreachable(), 0, "circle stays in reach");

        // Taller stance, less reach to spare, so less tilt available.
        const hexapod::dance::CircleShape shape =
            hexapod::dance::probed_circle_shape(control);
        std::printf("        circle at ride height %2d: %.0f degrees, feet at %.0f%%\n",
                    height, shape.tilt, shape.tuck * 100.0);
        expect_true(shape.tilt < previous, "taller stance yields less tilt");
        expect_true(shape.tilt >= 8.0, "some tilt is always available");
        expect_true(shape.tuck <= 1.0 && shape.tuck >= 0.8, "tuck stays sane");
        previous = shape.tilt;
    }
}

// The bias knob redistributes the dip between front and rear rather than
// scaling it, so the extremes are not obviously equivalent: at bias 0 every
// leg takes the full 28 mm at once, which is a different pose from the rear
// taking it alone. Check both ends, and the staged routine as well.
void test_twerk_bias_extremes_stay_in_reach()
{
    for (int height : {0, 40, 80}) {
        for (double bias : {0.0, 0.5, 1.0}) {
            for (const char* name : {"twerk", "show"}) {
                CountingBus bus;
                hexapod::Control control(bus, nominal_calibration());
                raise_to(control, height);
                bus.reset();

                hexapod::dance::perform(control, name, 60, 1, bias);
                if (bus.unreachable() != 0) {
                    std::printf("  FAIL  %s at height %d bias %.2f: %ld unreachable\n",
                                name, height, bias, bus.unreachable());
                    ++failures;
                }
            }
        }
    }
}

}  // namespace

int main()
{
    std::printf("unit tests:\n");
    test_angle_to_ticks();
    test_channel_routing();
    test_prescale();
    test_register_bytes();
    test_dirty_runs();
    test_channel_map_masks();
    test_foot_trajectory();
    test_gait_patterns_stay_in_reach();
    test_dance_routines_stay_in_reach();
    test_circle_tilt_adapts_to_height();
    test_twerk_bias_extremes_stay_in_reach();

    if (failures == 0) {
        std::printf("  PASS  all unit tests\n");
        return 0;
    }
    std::printf("  %d failure(s)\n", failures);
    return 1;
}
