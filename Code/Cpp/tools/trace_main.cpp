// Emits the C++ side of the differential trace.
//
// Runs the same scenarios, against the same calibration file, as
// test/gen_reference.py runs against the original Python, and writes the same
// line format. Any difference between the two traces is a porting defect.

#include "hexapod/control.hpp"
#include "hexapod/pwm_encoding.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

// Records the write sequence instead of touching I2C.
class TraceBus final : public hexapod::ServoBus {
public:
    explicit TraceBus(std::ostream& out) : out_(out) {}

    void set_angle(int channel, int angle) override
    {
        out_ << "w " << channel << ' ' << angle << '\n';
        ++writes_;

        // Encode the same way the real bus will, so the trace also pins down
        // the tick value and the chip routing -- the layer where the original
        // truncates a float and splits channels across two PCA9685s.
        const hexapod::pwm::ChannelTarget target = hexapod::pwm::encode(channel, angle);
        if (target.valid) {
            char line[64];
            std::snprintf(line, sizeof(line), "t 0x%02x %d %d %d\n",
                          target.chip_address, target.chip_channel,
                          target.on, target.off);
            out_ << line;
            ++ticks_;
        }
    }

    void on_unreachable() override
    {
        out_ << "!range\n";
        ++skips_;
    }

    long writes() const { return writes_; }
    long ticks() const { return ticks_; }
    long skips() const { return skips_; }

private:
    std::ostream& out_;
    long writes_ = 0;
    long ticks_ = 0;
    long skips_ = 0;
};

std::vector<std::string> tokenize(const std::string& raw)
{
    const std::string line = raw.substr(0, raw.find('#'));
    std::istringstream stream(line);
    std::vector<std::string> tokens;
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

// point.txt: six rows of tab-separated integers, the measured foot positions.
hexapod::FootPositions read_calibration(const std::string& path)
{
    std::ifstream file(path);
    if (!file) {
        std::cerr << "cannot open " << path << '\n';
        std::exit(1);
    }

    hexapod::FootPositions points{};
    for (int i = 0; i < hexapod::kLegCount; ++i) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!(file >> x >> y >> z)) {
            std::cerr << "short read in " << path << " at row " << i << '\n';
            std::exit(1);
        }
        points[i] = hexapod::Vec3{x, y, z};
    }
    return points;
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string scenario_path = (argc > 1) ? argv[1] : "test/scenarios.txt";
    const std::string point_path = (argc > 2) ? argv[2] : "../Server/point.txt";
    const std::string output_path = (argc > 3) ? argv[3] : "test/cpp.trace";

    std::ifstream scenario_file(scenario_path);
    if (!scenario_file) {
        std::cerr << "cannot open " << scenario_path << '\n';
        return 1;
    }

    std::vector<std::vector<std::string>> scenarios;
    std::string raw;
    while (std::getline(scenario_file, raw)) {
        std::vector<std::string> tokens = tokenize(raw);
        if (!tokens.empty()) {
            scenarios.push_back(tokens);
        }
    }

    std::ofstream out(output_path, std::ios::binary);
    if (!out) {
        std::cerr << "cannot write " << output_path << '\n';
        return 1;
    }

    TraceBus bus(out);
    // Construction calibrates and applies the stance, exactly as the Python
    // constructor does, so those writes belong in the trace too.
    hexapod::Control control(bus, read_calibration(point_path));

    for (std::size_t index = 0; index < scenarios.size(); ++index) {
        const std::vector<std::string>& parts = scenarios[index];

        out << "s " << index;
        for (const std::string& part : parts) {
            out << ' ' << part;
        }
        out << '\n';

        const std::string& verb = parts[0];
        if (verb == "gait") {
            hexapod::GaitCommand command;
            command.gait = std::atoi(parts[1].c_str());
            command.x = std::atoi(parts[2].c_str());
            command.y = std::atoi(parts[3].c_str());
            command.speed = std::atoi(parts[4].c_str());
            command.angle = std::atoi(parts[5].c_str());
            control.run_gait(command);
        } else if (verb == "position") {
            control.move_position(std::atoi(parts[1].c_str()),
                                  std::atoi(parts[2].c_str()),
                                  std::atoi(parts[3].c_str()));
        } else if (verb == "attitude") {
            const hexapod::FootPositions points = control.calculate_posture_balance(
                std::atoi(parts[1].c_str()),
                std::atoi(parts[2].c_str()),
                std::atoi(parts[3].c_str()));
            control.transform_coordinates(points);
            control.set_leg_angles();
        } else if (verb == "calibrate") {
            control.calibrate();
            control.set_leg_angles();
        } else {
            std::cerr << "unknown scenario verb: " << verb << '\n';
            return 1;
        }
    }

    std::cerr << "scenarios: " << scenarios.size()
              << "  servo writes: " << bus.writes()
              << "  pwm writes: " << bus.ticks()
              << "  out-of-range frames: " << bus.skips() << '\n';
    std::cerr << "wrote " << output_path << '\n';
    return 0;
}
