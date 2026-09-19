// Drive the hexapod: open the I2C bus, power the servos, walk, park.
//
// Linux only. Build on the robot with `make robot`.

#include "hexapod/battery.hpp"
#include "hexapod/control.hpp"
#include "hexapod/dance.hpp"
#include "hexapod/gait.hpp"
#include "hexapod/paced_bus.hpp"
#include "hexapod/pca9685_bus.hpp"
#include "hexapod/servo_power.hpp"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

namespace {

volatile sig_atomic_t g_stop = 0;

void on_signal(int)
{
    g_stop = 1;
}

// Counts frames without touching hardware, for --dry-run.
class NullBus final : public hexapod::ServoBus {
public:
    void set_angle(int, int) override { ++writes_; }
    void commit() override { ++frames_; }
    void on_unreachable() override { ++skips_; }

    long writes() const { return writes_; }
    long frames() const { return frames_; }
    long skips() const { return skips_; }

private:
    long writes_{0};
    long frames_{0};
    long skips_{0};
};

struct Options {
    int gait{1};
    int x{0};
    int y{35};
    int speed{10};
    int angle{0};
    int cycles{3};
    int height{40};
    long period_ms{30};
    long arm_delay_ms{3000};
    std::string device{"/dev/i2c-1"};
    std::string points{"../Server/point.txt"};
    bool dry_run{false};
    bool keep_powered{false};
    bool no_servo_power{false};
    bool straighten{false};
    bool list{false};
    int frames{60};
    int twerk_bias{90};  // percent
    std::string pattern;
    std::string routine;
};

// The angles servo.py holds every channel at while horns and legs are fitted.
// This is the mechanical zero the whole kinematic chain is referenced to: a
// leg attached one spline tooth away from here is permanently wrong by that
// tooth, and nothing downstream can detect it, because no joint position is
// ever read back.
int install_angle(int channel)
{
    if (channel == 10 || channel == 13 || channel == 31) {
        return 10;
    }
    if (channel == 18 || channel == 21 || channel == 27) {
        return 170;
    }
    return 90;
}

void usage()
{
    std::printf(
        "usage: hexapod_walk [options]\n"
        "\n"
        "  --pattern NAME    gait from the phase-based engine: tripod, ripple, wave\n"
        "  --dance NAME      a routine instead of walking; --list to see them\n"
        "  --frames N        frames per cycle for --pattern and --dance (default 60)\n"
        "  --twerk-bias N    0..100, how much more the rear moves than the front\n"
        "                    during twerk and show. 0 is a plain bob, 85 was the\n"
        "                    first version, 100 holds the front legs still (default 90)\n"
        "  --list            print the available patterns and dances, then exit\n"
        "\n"
        "  --gait N          original engine: 1 = tripod, 2 = wave  (default 1)\n"
        "  --x N             sideways travel per cycle, mm, -35..35  (default 0)\n"
        "  --y N             forward travel per cycle, mm, -35..35   (default 35)\n"
        "  --speed N         2..10; higher means fewer, larger frames (default 10)\n"
        "  --angle N         yaw per cycle, degrees          (default 0)\n"
        "  --cycles N        gait cycles to run              (default 3)\n"
        "  --height N        ride height, -20..80; higher stands taller (default 40)\n"
        "  --period-ms N     frame period                    (default 30)\n"
        "  --arm-delay-ms N  pause before the first servo command (default 3000)\n"
        "  --i2c PATH        I2C device                      (default /dev/i2c-1)\n"
        "  --points PATH     calibration file                (default ../Server/point.txt)\n"
        "  --straighten      hold every servo at its assembly reference angle\n"
        "                    and wait, for checking or refitting the legs\n"
        "  --dry-run         run the gait maths and pacing, touch no hardware\n"
        "  --no-servo-power  drive I2C for real, but never energise the servo\n"
        "                    rail: bench-test the full driver without motion\n"
        "  --keep-powered    leave the servo rail on at exit\n"
        "  --help, -h        this message\n"
        "\n"
        "--speed applies to --gait only; --frames applies to --pattern and\n"
        "--dance only. Setting the wrong one for the engine you picked does\n"
        "nothing.\n"
        "\n"
        "The default 30 ms period is roughly the original Python's effective frame\n"
        "rate (~21 ms of I2C plus its 10 ms sleep). Lowering it walks FASTER,\n"
        "because the stride per cycle is fixed -- the servos have to keep up.\n");
}

bool parse_long(const char* text, long* out)
{
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        return false;
    }
    *out = value;
    return true;
}

bool parse_options(int argc, char** argv, Options* options)
{
    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        const bool has_value = (i + 1) < argc;

        auto take_long = [&](long* target) {
            if (!has_value || !parse_long(argv[i + 1], target)) {
                std::fprintf(stderr, "%s needs a number\n", flag.c_str());
                return false;
            }
            ++i;
            return true;
        };
        auto take_int = [&](int* target) {
            long value = 0;
            if (!take_long(&value)) {
                return false;
            }
            *target = static_cast<int>(value);
            return true;
        };

        if (flag == "--help" || flag == "-h") {
            usage();
            std::exit(0);
        } else if (flag == "--gait") {
            if (!take_int(&options->gait)) return false;
        } else if (flag == "--x") {
            if (!take_int(&options->x)) return false;
        } else if (flag == "--y") {
            if (!take_int(&options->y)) return false;
        } else if (flag == "--speed") {
            if (!take_int(&options->speed)) return false;
        } else if (flag == "--angle") {
            if (!take_int(&options->angle)) return false;
        } else if (flag == "--cycles") {
            if (!take_int(&options->cycles)) return false;
        } else if (flag == "--height") {
            if (!take_int(&options->height)) return false;
        } else if (flag == "--period-ms") {
            if (!take_long(&options->period_ms)) return false;
        } else if (flag == "--arm-delay-ms") {
            if (!take_long(&options->arm_delay_ms)) return false;
        } else if (flag == "--i2c") {
            if (!has_value) {
                std::fprintf(stderr, "--i2c needs a path\n");
                return false;
            }
            options->device = argv[++i];
        } else if (flag == "--points") {
            if (!has_value) {
                std::fprintf(stderr, "--points needs a path\n");
                return false;
            }
            options->points = argv[++i];
        } else if (flag == "--frames") {
            if (!take_int(&options->frames)) return false;
        } else if (flag == "--twerk-bias") {
            if (!take_int(&options->twerk_bias)) return false;
        } else if (flag == "--list") {
            options->list = true;
        } else if (flag == "--pattern") {
            if (!has_value) {
                std::fprintf(stderr, "--pattern needs a name\n");
                return false;
            }
            options->pattern = argv[++i];
        } else if (flag == "--dance") {
            if (!has_value) {
                std::fprintf(stderr, "--dance needs a name\n");
                return false;
            }
            options->routine = argv[++i];
        } else if (flag == "--straighten") {
            options->straighten = true;
        } else if (flag == "--dry-run") {
            options->dry_run = true;
        } else if (flag == "--no-servo-power") {
            options->no_servo_power = true;
        } else if (flag == "--keep-powered") {
            options->keep_powered = true;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", flag.c_str());
            usage();
            return false;
        }
    }
    return true;
}

void sleep_ms(long milliseconds)
{
    if (milliseconds <= 0) {
        return;
    }
    timespec request;
    request.tv_sec = milliseconds / 1000;
    request.tv_nsec = (milliseconds % 1000) * 1000000L;
    nanosleep(&request, nullptr);
}

}  // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parse_options(argc, argv, &options)) {
        return 2;
    }

    if (options.list) {
        std::printf("gait patterns (--pattern):\n");
        for (int i = 0; i < hexapod::gait::pattern_count(); ++i) {
            const hexapod::gait::Pattern& p = hexapod::gait::patterns()[i];
            std::printf("  %-8s  %s\n", p.name, p.description);
        }
        std::printf("\ndances (--dance):\n");
        for (int i = 0; i < hexapod::dance::routine_count(); ++i) {
            const hexapod::dance::Routine& r = hexapod::dance::routines()[i];
            std::printf("  %-8s  %s\n", r.name, r.description);
        }
        return 0;
    }

    // Reject unknown names before touching any hardware, rather than after
    // the robot has already stood up.
    if (!options.pattern.empty() &&
        hexapod::gait::find_pattern(options.pattern.c_str()) == nullptr) {
        std::fprintf(stderr, "unknown pattern: %s (try --list)\n", options.pattern.c_str());
        return 2;
    }
    if (!options.routine.empty() &&
        hexapod::dance::find_routine(options.routine.c_str()) == nullptr) {
        std::fprintf(stderr, "unknown dance: %s (try --list)\n", options.routine.c_str());
        return 2;
    }

    hexapod::FootPositions calibration{};
    std::string error;
    if (!hexapod::read_calibration_file(options.points, &calibration, &error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 1;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    NullBus null_bus;
    hexapod::Pca9685Bus i2c_bus;
    hexapod::ServoPower power;

    hexapod::ServoBus* inner = &null_bus;

    if (!options.dry_run) {
        if (!i2c_bus.open(options.device, &error)) {
            std::fprintf(stderr, "i2c: %s\n", error.c_str());
            return 1;
        }
        inner = &i2c_bus;

        // Check the packs before doing anything else. The PCA9685s draw their
        // logic supply from the Pi, so a flat servo pack is completely
        // invisible from the I2C side: every write succeeds, the run reports a
        // clean 268 transactions, and nothing moves.
        hexapod::BatteryMonitor battery;
        if (battery.open(options.device, &error)) {
            double pack_a = 0.0;
            double pack_b = 0.0;
            if (battery.read(&pack_a, &pack_b, &error)) {
                std::printf("battery packs: %.2f V and %.2f V (need %.1f V each)\n",
                            pack_a, pack_b, hexapod::BatteryMonitor::kMinPackVolts);
                if (pack_a < hexapod::BatteryMonitor::kMinPackVolts ||
                    pack_b < hexapod::BatteryMonitor::kMinPackVolts) {
                    std::printf("WARNING: a pack is below the %.1f V minimum. "
                                "The servos will not move, however clean the I2C looks.\n",
                                hexapod::BatteryMonitor::kMinPackVolts);
                }
            } else {
                std::fprintf(stderr, "battery read failed: %s\n", error.c_str());
            }
        } else {
            std::fprintf(stderr, "battery monitor unavailable: %s\n", error.c_str());
        }

        if (options.no_servo_power) {
            // Every register write still goes out on the wire; only the rail
            // stays dead. This is the mode for validating the driver on a
            // bench, and for a first run with the batteries in.
            std::printf("servo rail left unpowered (--no-servo-power)\n");
        } else if (!power.open(&error) || !power.enable(&error)) {
            std::fprintf(stderr, "servo power: %s\n", error.c_str());
            return 1;
        } else {
            std::printf("servo power enabled via %s line %u\n",
                        power.chip_path().c_str(), hexapod::ServoPower::kDefaultLine);
        }
    }

    hexapod::PacedBus bus(*inner, options.period_ms * 1000);

    // Constructing Control calibrates and immediately drives all 18 joints to
    // the stance pose, from wherever the legs currently are. That is a fast,
    // full-authority move -- the same one the Python makes on startup.
    // Only a warning worth making when something can actually move: with the
    // rail unpowered the same writes go out and the legs stay put.
    const bool can_move = !options.dry_run && !options.no_servo_power;
    if (can_move && options.arm_delay_ms > 0) {
        std::printf("about to move ALL 18 servos to the stance pose in %ld ms.\n",
                    options.arm_delay_ms);
        std::printf("support the body now, or Ctrl-C to abort.\n");
        std::fflush(stdout);
        sleep_ms(options.arm_delay_ms);
        if (g_stop) {
            std::printf("aborted before arming\n");
            if (power.is_open() && !options.keep_powered) {
                power.disable();
            }
            return 0;
        }
    }

    if (options.straighten) {
        // Bypass Control entirely. These are raw channel angles, not a pose
        // derived from IK, so what you see is the servos' own reference
        // unfiltered by kinematics or by point.txt calibration -- which is the
        // point: it is the thing the rest of the chain is measured against.
        //
        // Written through `inner` rather than the paced bus because a static
        // pose has no frames to pace.
        std::printf("holding all 32 channels at their assembly reference angles\n");
        for (int channel = 0; channel < 32; ++channel) {
            inner->set_angle(channel, install_angle(channel));
        }
        inner->commit();

        std::printf("check each leg; refit any horn that sits off. Ctrl-C when done.\n");
        while (g_stop == 0) {
            sleep_ms(100);
        }
        std::printf("\n");

        if (!options.dry_run) {
            i2c_bus.relax();
            if (power.is_open() && !options.keep_powered) {
                power.disable();
                std::printf("servo power disabled\n");
            }
        }
        return 0;
    }

    bus.reset();
    hexapod::Control control(bus, calibration);

    // Stand up before walking.
    //
    // body_height defaults to -25 mm, which leaves the chassis on the ground:
    // the gait runs, the legs cycle, and the robot drags itself rather than
    // stepping. move_position sets body_height to -30 - height, so a larger
    // height stands taller.
    //
    // The original protocol clamps this to +/-20, but that is a limit of the
    // client, not of the legs. Measured against the worst case the gait can
    // produce -- a full diagonal stride, x and y both saturated -- peak leg
    // reach runs 196 mm at height 20, 204 at 40, 214 at 60 and 224 at 80,
    // against a physical maximum of 233 mm (33 + 90 + 110). Past 80 the legs
    // run out of reach mid-stride and the IK silently clamps.
    //
    // Note the reach limit is not the practical one: at height 80 the leg is
    // 96% extended, where it has almost no mechanical advantage left and the
    // servos struggle to hold the body up at all.
    //
    // Ramped one millimetre per call rather than applied in one go, because
    // move_position drives straight to the new pose -- a large jump across all
    // 18 joints at once is a violent move. Each step is one frame, so PacedBus
    // times the ascent for free.
    constexpr int kMaxHeight = 80;
    constexpr int kTallHeight = 60;  // above this, stride margin gets thin
    int height = options.height;
    if (height > kMaxHeight) {
        height = kMaxHeight;
    }
    if (height < -20) {
        height = -20;
    }
    if (height > kTallHeight) {
        std::printf("note: ride height %d leaves under 20 mm of leg reach at full "
                    "stride, and little torque to hold the body up.\n", height);
    }
    std::printf("standing up: ride height %d (body %d mm)\n", height, -30 - height);
    const int rise = (height >= 0) ? 1 : -1;
    for (int z = 0;; z += rise) {
        control.move_position(0, 0, z);
        if (z == height) {
            break;
        }
    }

    int completed = 0;

    if (!options.routine.empty()) {
        const double bias = options.twerk_bias / 100.0;
        std::printf("dance %s  repeats %d  frames %d  period %ld ms  twerk bias %d%%\n",
                    options.routine.c_str(), options.cycles, options.frames,
                    options.period_ms, options.twerk_bias);
        for (int beat = 0; beat < options.cycles && g_stop == 0; ++beat) {
            hexapod::dance::perform(control, options.routine.c_str(), options.frames, 1,
                                    bias);
            ++completed;
        }

    } else if (!options.pattern.empty()) {
        // The phase-based engine. Checked for a valid name before any hardware
        // was touched, so this lookup cannot fail here.
        const hexapod::gait::Pattern* pattern =
            hexapod::gait::find_pattern(options.pattern.c_str());
        hexapod::gait::Motion motion;
        motion.x = options.x;
        motion.y = options.y;
        motion.yaw_deg = options.angle;

        std::printf("pattern %s  x %d  y %d  yaw %d  cycles %d  frames %d  period %ld ms\n",
                    pattern->name, options.x, options.y, options.angle,
                    options.cycles, options.frames, options.period_ms);
        for (int cycle = 0; cycle < options.cycles && g_stop == 0; ++cycle) {
            hexapod::gait::walk(control, *pattern, motion, 1, options.frames);
            ++completed;
        }

    } else {
        // The original engine, byte-for-byte equivalent to the Python.
        hexapod::GaitCommand command;
        command.gait = options.gait;
        command.x = options.x;
        command.y = options.y;
        command.speed = options.speed;
        command.angle = options.angle;

        std::printf("gait %d  x %d  y %d  speed %d  angle %d  cycles %d  period %ld ms\n",
                    command.gait, command.x, command.y, command.speed, command.angle,
                    options.cycles, options.period_ms);
        for (int cycle = 0; cycle < options.cycles && g_stop == 0; ++cycle) {
            control.run_gait(command);
            ++completed;
        }
    }

    if (g_stop) {
        std::printf("\ninterrupted after %d of %d cycles\n", completed, options.cycles);
    }

    std::printf("frames %ld  overruns %ld  worst overrun %ld us\n",
                bus.frames(), bus.overruns(), bus.worst_overrun_us());

    if (options.dry_run) {
        std::printf("dry run: %ld servo writes, %ld frames, %ld unreachable\n",
                    null_bus.writes(), null_bus.frames(), null_bus.skips());
        return 0;
    }

    std::printf("i2c transactions %ld over %ld frames (%.1f per frame), %ld failed\n",
                i2c_bus.transactions(), i2c_bus.frames(),
                i2c_bus.frames() > 0
                    ? static_cast<double>(i2c_bus.transactions()) / i2c_bus.frames()
                    : 0.0,
                i2c_bus.failed_writes());
    if (i2c_bus.failed_writes() > 0) {
        std::printf("WARNING: %ld i2c writes were not accepted -- a chip stopped "
                    "acknowledging, so some joints held their last position.\n",
                    i2c_bus.failed_writes());
    }

    // Sit down before cutting power. relax() disables the outputs, so from a
    // standing pose the robot would simply drop; lowering first lets it settle.
    //
    // Start from where the robot actually is, not from the --height option: a
    // staged dance changes ride height as it runs and may already have
    // finished on the floor, and stepping from the option value would stand it
    // back up first.
    for (int z = static_cast<int>(std::lround(-30.0 - control.body_height()));
         z >= -20; --z) {
        control.move_position(0, 0, z);
    }

    // Park the legs before cutting power, so the robot relaxes rather than
    // dropping under its own weight with the outputs still asserted.
    i2c_bus.relax();
    if (power.is_open() && !options.keep_powered) {
        power.disable();
        std::printf("servo power disabled\n");
    }
    return 0;
}
