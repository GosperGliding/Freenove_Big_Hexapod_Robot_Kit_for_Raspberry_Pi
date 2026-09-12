#pragma once

#include <cstdint>

namespace hexapod::pwm {

// Everything in this header is pure arithmetic on purpose. Turning a joint
// angle into PCA9685 register bytes is where the original silently routes
// channels between two chips and truncates a float, so it is worth having
// under the same differential test as the kinematics rather than hidden
// inside an I2C driver that only runs on the robot.

inline constexpr int kChannelsPerChip = 16;
inline constexpr int kRegisterBytesPerChannel = 4;
inline constexpr int kBankBytes = kChannelsPerChip * kRegisterBytesPerChannel;  // 64

// LED0_ON_L. Channel n occupies kLed0OnL + 4n through kLed0OnL + 4n + 3,
// in the order ON_L, ON_H, OFF_L, OFF_H.
inline constexpr std::uint8_t kLed0OnL = 0x06;

inline constexpr std::uint8_t kRegMode1 = 0x00;
inline constexpr std::uint8_t kRegPrescale = 0xFE;

inline constexpr std::uint8_t kMode1Restart = 0x80;
inline constexpr std::uint8_t kMode1Sleep = 0x10;
// Auto-increment. The original never sets this, which is why it needs four
// separate transactions per channel; with it, a whole 16-channel bank is one.
inline constexpr std::uint8_t kMode1AutoIncrement = 0x20;

// Flat channel 0..31 maps across two chips. This split, and the -16 remap, is
// the original's, including its behaviour for out-of-range channels.
inline constexpr int kChipLowAddress = 0x41;   // flat channels 0..15
inline constexpr int kChipHighAddress = 0x40;  // flat channels 16..31

inline constexpr double kPwmFrequencyHz = 50.0;
inline constexpr double kMinPulseUs = 500.0;
inline constexpr double kMaxPulseUs = 2500.0;
inline constexpr double kPeriodUs = 20000.0;
inline constexpr double kTickMax = 4095.0;

// Writing both the full-on and full-off bits parks a channel with its output
// disabled, which is what the original calls "relax".
inline constexpr int kFullOnOffTicks = 4096;

struct ChannelTarget {
    bool valid{false};    // false for a channel the original would have ignored
    int chip_address{0};
    int chip_channel{0};
    int on{0};
    int off{0};
};

// 0..180 degrees -> 500..2500us -> 0..4095 ticks.
//
// The result is TRUNCATED, not rounded: the original ends with int(duty_cycle).
// Rounding here would shift roughly half of all joint commands by one tick.
int angle_to_ticks(int angle);

// Route a flat 0..31 channel to a chip and encode the angle. A channel of 32
// or more comes back with valid == false, matching the original, whose
// if/elif chain simply falls through and writes nothing.
ChannelTarget encode(int flat_channel, int angle);

// PCA9685 prescale for a target frequency, using the original's rounding.
int prescale_for(double frequency_hz);

// Serialise one channel into its four register bytes, in register order.
void encode_channel_bytes(int on, int off, std::uint8_t* out);

// Serialise a whole 16-channel bank into kBankBytes bytes, ready to follow a
// single kLed0OnL register address in one auto-incrementing transaction.
void encode_bank(const int* on, const int* off, std::uint8_t* out);

// A contiguous span of channels that can go out in one auto-incrementing
// transaction.
struct Run {
    int first{0};
    int count{0};
};

// Split a 16-bit set of touched channels into contiguous runs.
//
// Writing one 16-channel block per chip would be fewer transactions, but it
// would also overwrite channels this program never set -- the head pan/tilt
// servos live on 0x41 channels 0 and 1, and six channels on 0x40 are unused.
// Runs keep the transaction count low without touching anything we do not own.
//
// Returns the number of runs written to `out`, capped at max_runs.
int find_dirty_runs(std::uint16_t dirty_mask, Run* out, int max_runs);

}  // namespace hexapod::pwm
