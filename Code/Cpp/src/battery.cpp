#include "hexapod/battery.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace hexapod {

namespace {

// ADS7830 single-ended conversion command.
constexpr unsigned char kCommand = 0x84;

// Divider ratio between the pack and the ADC input.
//
// Carried over verbatim from adc.py, whose own comment says this depends on
// the PCB version -- yet unlike led.py it never consults ParameterManager for
// it. So treat a reading as indicative, not calibrated: it is reliable for
// "is this pack flat or not", which is what it is used for here, and should
// not be trusted to a tenth of a volt.
constexpr double kVoltageCoefficient = 3.0;

constexpr double kAdcReference = 5.0;
constexpr double kAdcFullScale = 255.0;

void set_error(std::string* error, const std::string& text)
{
    if (error != nullptr) {
        *error = text + ": " + std::strerror(errno);
    }
}

}  // namespace

BatteryMonitor::~BatteryMonitor()
{
    close();
}

bool BatteryMonitor::open(const std::string& device, std::string* error)
{
    close();

    fd_ = ::open(device.c_str(), O_RDWR);
    if (fd_ < 0) {
        set_error(error, "open " + device);
        return false;
    }
    if (ioctl(fd_, I2C_SLAVE, kAddress) < 0) {
        set_error(error, "I2C_SLAVE for the ADS7830");
        close();
        return false;
    }
    return true;
}

void BatteryMonitor::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool BatteryMonitor::read_stable_byte(unsigned char* value, std::string* error)
{
    // The original loops until two consecutive reads agree, to skip a sample
    // caught mid-conversion. Bounded here so a stuck ADC cannot hang the robot.
    unsigned char first = 0;
    unsigned char second = 0;
    for (int attempt = 0; attempt < 16; ++attempt) {
        if (::read(fd_, &first, 1) != 1 || ::read(fd_, &second, 1) != 1) {
            set_error(error, "read ADS7830");
            return false;
        }
        if (first == second) {
            *value = first;
            return true;
        }
    }
    // Never settled; the last sample is still better than failing outright.
    *value = second;
    return true;
}

bool BatteryMonitor::read_channel(int channel, double* volts, std::string* error)
{
    const unsigned char selector =
        static_cast<unsigned char>((((channel << 2) | (channel >> 1)) & 0x07) << 4);
    const unsigned char command = static_cast<unsigned char>(kCommand | selector);

    if (::write(fd_, &command, 1) != 1) {
        set_error(error, "select ADS7830 channel " + std::to_string(channel));
        return false;
    }

    unsigned char raw = 0;
    if (!read_stable_byte(&raw, error)) {
        return false;
    }

    *volts = raw / kAdcFullScale * kAdcReference * kVoltageCoefficient;
    return true;
}

bool BatteryMonitor::read(double* pack_a, double* pack_b, std::string* error)
{
    if (fd_ < 0) {
        if (error != nullptr) {
            *error = "battery monitor is not open";
        }
        return false;
    }
    // Channels 0 and 4, as in adc.py's read_battery_voltage.
    return read_channel(0, pack_a, error) && read_channel(4, pack_b, error);
}

}  // namespace hexapod
