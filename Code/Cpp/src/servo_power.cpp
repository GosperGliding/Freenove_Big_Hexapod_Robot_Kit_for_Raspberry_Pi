#include "hexapod/servo_power.hpp"

#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace hexapod {

namespace {

void set_error(std::string* error, const std::string& text)
{
    if (error != nullptr) {
        *error = text + ": " + std::strerror(errno);
    }
}

// The header pins live on the SoC pin controller: pinctrl-bcm2835 on older
// boards, pinctrl-bcm2711 on a Pi 4, pinctrl-rp1 on a Pi 5. Chip numbering is
// not stable across those, so match on the label instead.
bool is_header_chip(const gpiochip_info& info)
{
    return std::strncmp(info.label, "pinctrl-", 8) == 0;
}

}  // namespace

ServoPower::~ServoPower()
{
    close();
}

bool ServoPower::open(std::string* error, unsigned line)
{
    close();

    for (int index = 0; index < 8; ++index) {
        const std::string path = "/dev/gpiochip" + std::to_string(index);
        const int chip_fd = ::open(path.c_str(), O_RDONLY);
        if (chip_fd < 0) {
            continue;
        }

        gpiochip_info info;
        std::memset(&info, 0, sizeof(info));
        if (ioctl(chip_fd, GPIO_GET_CHIPINFO_IOCTL, &info) < 0 ||
            !is_header_chip(info) || line >= info.lines) {
            ::close(chip_fd);
            continue;
        }

        gpio_v2_line_request request;
        std::memset(&request, 0, sizeof(request));
        request.offsets[0] = line;
        request.num_lines = 1;
        request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
        std::strncpy(request.consumer, "hexapod-servo-power",
                     sizeof(request.consumer) - 1);

        if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request) < 0) {
            set_error(error, "GPIO_V2_GET_LINE_IOCTL on " + path);
            ::close(chip_fd);
            return false;
        }
        ::close(chip_fd);

        line_fd_ = request.fd;
        chip_path_ = path;
        return true;
    }

    if (error != nullptr) {
        *error = "no gpiochip with a pinctrl- label exposes line " +
                 std::to_string(line) +
                 " (needs Linux 5.10 or newer for the v2 GPIO uAPI)";
    }
    return false;
}

void ServoPower::close()
{
    if (line_fd_ >= 0) {
        ::close(line_fd_);
        line_fd_ = -1;
    }
    chip_path_.clear();
}

bool ServoPower::set_value(bool high, std::string* error)
{
    if (line_fd_ < 0) {
        if (error != nullptr) {
            *error = "servo power line is not open";
        }
        return false;
    }

    gpio_v2_line_values values;
    std::memset(&values, 0, sizeof(values));
    values.mask = 1;
    values.bits = high ? 1 : 0;

    if (ioctl(line_fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &values) < 0) {
        set_error(error, "GPIO_V2_LINE_SET_VALUES_IOCTL");
        return false;
    }
    return true;
}

bool ServoPower::enable(std::string* error)
{
    return set_value(false, error);
}

bool ServoPower::disable(std::string* error)
{
    return set_value(true, error);
}

}  // namespace hexapod
