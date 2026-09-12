#pragma once

namespace hexapod {

// The hardware seam. Everything above this interface is pure computation and
// runs anywhere; everything below it is I2C. The Python had no such boundary --
// Control called PCA9685 directly -- which is why none of it could be tested
// off-robot.
class ServoBus {
public:
    virtual ~ServoBus() = default;

    // channel is the flat 0..31 address used by the original: 0..15 on the
    // 0x41 chip, 16..31 on 0x40.
    virtual void set_angle(int channel, int angle) = 0;

    // End of frame: all 18 joints for this pose have been handed over. A
    // batching implementation flushes here, and a pacing one sleeps here.
    // Not called for a frame that was skipped as unreachable, because nothing
    // was staged for it.
    virtual void commit() {}

    // Called instead of any set_angle for a frame whose foot targets fall
    // outside the reach envelope. The original printed a message and wrote
    // nothing, freezing the legs for that frame.
    virtual void on_unreachable() {}
};

}  // namespace hexapod
