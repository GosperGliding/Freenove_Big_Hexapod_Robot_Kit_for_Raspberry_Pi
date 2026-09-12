#pragma once

#include "hexapod/pwm_encoding.hpp"
#include "hexapod/servo_bus.hpp"

#include <cstdint>
#include <string>

namespace hexapod {

// ServoBus over two PCA9685 chips on a Linux I2C bus.
//
// The original issued four separate I2C transactions per channel -- 72 per
// frame -- because it never enabled the chip's auto-increment bit. This one
// stages a frame in a shadow register file and flushes each contiguous run of
// touched channels in a single transaction, which the robot's channel map
// reduces to four.
//
// Linux only: it talks to /dev/i2c-N directly.
class Pca9685Bus final : public ServoBus {
public:
    Pca9685Bus() = default;
    ~Pca9685Bus() override;

    Pca9685Bus(const Pca9685Bus&) = delete;
    Pca9685Bus& operator=(const Pca9685Bus&) = delete;

    // Opens the bus and configures both chips for 50 Hz with auto-increment
    // enabled. Returns false and fills `error` on failure.
    bool open(const std::string& device, std::string* error);
    void close();

    // Stage one joint. Nothing reaches the wire until commit().
    void set_angle(int channel, int angle) override;

    // Flush the staged frame.
    void commit() override;

    // Park every channel this bus has ever driven with its output disabled,
    // which is what the original calls "relax". Takes effect immediately.
    void relax();

    long transactions() const { return transactions_; }
    long frames() const { return frames_; }

private:
    struct Chip {
        int fd{-1};
        int address{0};
        int on[pwm::kChannelsPerChip]{};
        int off[pwm::kChannelsPerChip]{};
        std::uint16_t dirty{0};
        std::uint16_t ever_written{0};
    };

    Chip* chip_for(int address);
    bool configure(Chip& chip, std::string* error);
    bool write_register(const Chip& chip, std::uint8_t reg, std::uint8_t value,
                        std::string* error);
    bool read_register(const Chip& chip, std::uint8_t reg, std::uint8_t* value,
                       std::string* error);
    bool flush(Chip& chip);

    Chip chips_[2];
    long transactions_{0};
    long frames_{0};
};

}  // namespace hexapod
