#pragma once

#include <string>

namespace hexapod {

// The servo power rail, on BCM GPIO 4.
//
// The line is an active-high DISABLE, which is why the Python constructor
// calls servo_power_disable.off() -- driving it low is what turns the servos
// on. Get this backwards and the I2C traffic is perfect while nothing moves.
//
// Uses the GPIO character device directly (kernel uAPI v2, so Linux 5.10 or
// newer) rather than libgpiod, to avoid a build dependency. The right chip is
// found by label, because the 40-pin header is on a different gpiochip on a
// Pi 5 (RP1) than on earlier boards.
class ServoPower {
public:
    static constexpr unsigned kDefaultLine = 4;

    ServoPower() = default;
    ~ServoPower();

    ServoPower(const ServoPower&) = delete;
    ServoPower& operator=(const ServoPower&) = delete;

    bool open(std::string* error, unsigned line = kDefaultLine);
    void close();

    bool is_open() const { return line_fd_ >= 0; }

    // Drive the line low: servos powered.
    bool enable(std::string* error = nullptr);

    // Drive the line high: servos unpowered.
    bool disable(std::string* error = nullptr);

    // Which gpiochip was selected, for diagnostics.
    const std::string& chip_path() const { return chip_path_; }

private:
    bool set_value(bool high, std::string* error);

    int line_fd_{-1};
    std::string chip_path_;
};

}  // namespace hexapod
