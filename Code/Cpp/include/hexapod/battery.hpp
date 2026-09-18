#pragma once

#include <string>

namespace hexapod {

// Reads the two battery packs through the ADS7830 at 0x48, mirroring
// Code/Server/adc.py.
//
// This exists because a flat servo pack is invisible from the I2C side: the
// PCA9685s take their logic supply from the Pi, so every register write
// succeeds and the run reports a clean 268 transactions while the legs never
// move. Checking the packs up front turns that silent failure into a message.
//
// Linux only.
class BatteryMonitor {
public:
    static constexpr int kAddress = 0x48;

    // The board carries two independent packs, one per DC socket.
    static constexpr double kMinPackVolts = 7.0;   // tutorial's stated minimum
    static constexpr double kFullPackVolts = 8.4;  // two cells at 4.2 V

    BatteryMonitor() = default;
    ~BatteryMonitor();

    BatteryMonitor(const BatteryMonitor&) = delete;
    BatteryMonitor& operator=(const BatteryMonitor&) = delete;

    bool open(const std::string& device, std::string* error);
    void close();
    bool is_open() const { return fd_ >= 0; }

    // Voltages of the two packs. Returns false only on an I2C failure.
    bool read(double* pack_a, double* pack_b, std::string* error);

private:
    bool read_channel(int channel, double* volts, std::string* error);
    bool read_stable_byte(unsigned char* value, std::string* error);

    int fd_{-1};
};

}  // namespace hexapod
