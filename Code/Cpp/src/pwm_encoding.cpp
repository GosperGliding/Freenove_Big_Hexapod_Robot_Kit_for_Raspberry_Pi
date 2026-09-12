#include "hexapod/pwm_encoding.hpp"

#include "hexapod/kinematics.hpp"

#include <cmath>

namespace hexapod::pwm {

int angle_to_ticks(int angle)
{
    // Reusing kinematics::map_value rather than restating the formula keeps
    // the operation order identical to the Python, which computes
    // (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low
    // -- multiply first, then divide.
    const double pulse_us =
        kinematics::map_value(angle, 0.0, 180.0, kMinPulseUs, kMaxPulseUs);
    const double ticks =
        kinematics::map_value(pulse_us, 0.0, kPeriodUs, 0.0, kTickMax);

    // Truncation toward zero, matching Python's int().
    return static_cast<int>(ticks);
}

ChannelTarget encode(int flat_channel, int angle)
{
    ChannelTarget target;
    target.on = 0;
    target.off = angle_to_ticks(angle);

    if (flat_channel < kChannelsPerChip) {
        target.valid = true;
        target.chip_address = kChipLowAddress;
        target.chip_channel = flat_channel;
    } else if (flat_channel < 2 * kChannelsPerChip) {
        target.valid = true;
        target.chip_address = kChipHighAddress;
        target.chip_channel = flat_channel - kChannelsPerChip;
    }
    // else: valid stays false. The original's if/elif falls through here and
    // writes nothing at all, so neither do we.

    return target;
}

int prescale_for(double frequency_hz)
{
    double prescale = 25000000.0;  // internal oscillator
    prescale /= 4096.0;            // 12-bit counter
    prescale /= frequency_hz;
    prescale -= 1.0;
    return static_cast<int>(std::floor(prescale + 0.5));
}

void encode_channel_bytes(int on, int off, std::uint8_t* out)
{
    out[0] = static_cast<std::uint8_t>(on & 0xFF);
    out[1] = static_cast<std::uint8_t>((on >> 8) & 0xFF);
    out[2] = static_cast<std::uint8_t>(off & 0xFF);
    out[3] = static_cast<std::uint8_t>((off >> 8) & 0xFF);
}

void encode_bank(const int* on, const int* off, std::uint8_t* out)
{
    for (int channel = 0; channel < kChannelsPerChip; ++channel) {
        encode_channel_bytes(on[channel], off[channel],
                             out + channel * kRegisterBytesPerChannel);
    }
}

int find_dirty_runs(std::uint16_t dirty_mask, Run* out, int max_runs)
{
    int count = 0;
    int channel = 0;
    while (channel < kChannelsPerChip && count < max_runs) {
        if ((dirty_mask & (1u << channel)) == 0) {
            ++channel;
            continue;
        }
        const int first = channel;
        while (channel < kChannelsPerChip && (dirty_mask & (1u << channel)) != 0) {
            ++channel;
        }
        out[count].first = first;
        out[count].count = channel - first;
        ++count;
    }
    return count;
}

}  // namespace hexapod::pwm
