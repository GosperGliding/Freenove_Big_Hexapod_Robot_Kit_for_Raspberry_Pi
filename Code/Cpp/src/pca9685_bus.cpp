#include "hexapod/pca9685_bus.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <time.h>

namespace hexapod {

namespace {

void set_error(std::string* error, const std::string& text)
{
    if (error != nullptr) {
        *error = text + ": " + std::strerror(errno);
    }
}

void sleep_ms(long milliseconds)
{
    timespec request;
    request.tv_sec = milliseconds / 1000;
    request.tv_nsec = (milliseconds % 1000) * 1000000L;
    nanosleep(&request, nullptr);
}

}  // namespace

Pca9685Bus::~Pca9685Bus()
{
    close();
}

bool Pca9685Bus::open(const std::string& device, std::string* error)
{
    const int addresses[2] = {pwm::kChipLowAddress, pwm::kChipHighAddress};

    for (int index = 0; index < 2; ++index) {
        Chip& chip = chips_[index];
        chip.address = addresses[index];

        // One file descriptor per chip, each bound to its own address, so no
        // ioctl is needed on the hot path.
        chip.fd = ::open(device.c_str(), O_RDWR);
        if (chip.fd < 0) {
            set_error(error, "open " + device);
            close();
            return false;
        }
        if (ioctl(chip.fd, I2C_SLAVE, chip.address) < 0) {
            set_error(error, "I2C_SLAVE for address " + std::to_string(chip.address));
            close();
            return false;
        }

        // Untouched channels stage as full-off rather than as a zero-width
        // pulse, so that anything this program never drives stays disabled
        // instead of being commanded to the bottom of its range.
        for (int channel = 0; channel < pwm::kChannelsPerChip; ++channel) {
            chip.on[channel] = pwm::kFullOnOffTicks;
            chip.off[channel] = pwm::kFullOnOffTicks;
        }

        if (!configure(chip, error)) {
            close();
            return false;
        }
    }
    return true;
}

void Pca9685Bus::close()
{
    for (Chip& chip : chips_) {
        if (chip.fd >= 0) {
            ::close(chip.fd);
            chip.fd = -1;
        }
    }
}

bool Pca9685Bus::configure(Chip& chip, std::string* error)
{
    // Wake, then the original's frequency sequence: sleep, set prescale,
    // restore the previous mode, wait for the oscillator, then restart.
    if (!write_register(chip, pwm::kRegMode1, 0x00, error)) {
        return false;
    }

    std::uint8_t old_mode = 0;
    if (!read_register(chip, pwm::kRegMode1, &old_mode, error)) {
        return false;
    }

    const std::uint8_t sleep_mode =
        static_cast<std::uint8_t>((old_mode & 0x7F) | pwm::kMode1Sleep);
    const std::uint8_t prescale =
        static_cast<std::uint8_t>(pwm::prescale_for(pwm::kPwmFrequencyHz));

    if (!write_register(chip, pwm::kRegMode1, sleep_mode, error) ||
        !write_register(chip, pwm::kRegPrescale, prescale, error) ||
        !write_register(chip, pwm::kRegMode1, old_mode, error)) {
        return false;
    }
    sleep_ms(5);
    if (!write_register(chip, pwm::kRegMode1,
                        static_cast<std::uint8_t>(old_mode | pwm::kMode1Restart), error)) {
        return false;
    }

    // Auto-increment. This is the one thing the original never does, and the
    // whole reason a frame can be four transactions instead of seventy-two.
    std::uint8_t mode = 0;
    if (!read_register(chip, pwm::kRegMode1, &mode, error)) {
        return false;
    }
    return write_register(chip, pwm::kRegMode1,
                          static_cast<std::uint8_t>(mode | pwm::kMode1AutoIncrement), error);
}

bool Pca9685Bus::write_register(const Chip& chip, std::uint8_t reg, std::uint8_t value,
                                std::string* error)
{
    const std::uint8_t payload[2] = {reg, value};
    if (::write(chip.fd, payload, sizeof(payload)) != static_cast<ssize_t>(sizeof(payload))) {
        set_error(error, "write register " + std::to_string(reg));
        return false;
    }
    return true;
}

bool Pca9685Bus::read_register(const Chip& chip, std::uint8_t reg, std::uint8_t* value,
                               std::string* error)
{
    if (::write(chip.fd, &reg, 1) != 1) {
        set_error(error, "select register " + std::to_string(reg));
        return false;
    }
    if (::read(chip.fd, value, 1) != 1) {
        set_error(error, "read register " + std::to_string(reg));
        return false;
    }
    return true;
}

Pca9685Bus::Chip* Pca9685Bus::chip_for(int address)
{
    for (Chip& chip : chips_) {
        if (chip.address == address) {
            return &chip;
        }
    }
    return nullptr;
}

void Pca9685Bus::set_angle(int channel, int angle)
{
    const pwm::ChannelTarget target = pwm::encode(channel, angle);
    if (!target.valid) {
        return;
    }
    Chip* chip = chip_for(target.chip_address);
    if (chip == nullptr || chip->fd < 0) {
        return;
    }
    chip->on[target.chip_channel] = target.on;
    chip->off[target.chip_channel] = target.off;
    const std::uint16_t bit = static_cast<std::uint16_t>(1u << target.chip_channel);
    chip->dirty |= bit;
    chip->ever_written |= bit;
}

void Pca9685Bus::commit()
{
    for (Chip& chip : chips_) {
        if (chip.fd >= 0 && chip.dirty != 0) {
            flush(chip);
        }
    }
    ++frames_;
}

bool Pca9685Bus::flush(Chip& chip)
{
    pwm::Run runs[pwm::kChannelsPerChip];
    const int run_count =
        pwm::find_dirty_runs(chip.dirty, runs, pwm::kChannelsPerChip);

    bool ok = true;
    for (int index = 0; index < run_count; ++index) {
        const pwm::Run& run = runs[index];

        // One register address followed by the run's bytes, delivered as a
        // single transaction because auto-increment is enabled.
        std::uint8_t payload[1 + pwm::kBankBytes];
        payload[0] = static_cast<std::uint8_t>(
            pwm::kLed0OnL + pwm::kRegisterBytesPerChannel * run.first);
        for (int offset = 0; offset < run.count; ++offset) {
            const int channel = run.first + offset;
            pwm::encode_channel_bytes(
                chip.on[channel], chip.off[channel],
                payload + 1 + offset * pwm::kRegisterBytesPerChannel);
        }

        const std::size_t length =
            1 + static_cast<std::size_t>(run.count) * pwm::kRegisterBytesPerChannel;
        if (::write(chip.fd, payload, length) != static_cast<ssize_t>(length)) {
            ok = false;
        }
        ++transactions_;
    }

    chip.dirty = 0;
    return ok;
}

void Pca9685Bus::relax()
{
    for (Chip& chip : chips_) {
        if (chip.fd < 0) {
            continue;
        }
        for (int channel = 0; channel < pwm::kChannelsPerChip; ++channel) {
            if ((chip.ever_written & (1u << channel)) != 0) {
                chip.on[channel] = pwm::kFullOnOffTicks;
                chip.off[channel] = pwm::kFullOnOffTicks;
            }
        }
        chip.dirty = chip.ever_written;
        flush(chip);
    }
}

}  // namespace hexapod
