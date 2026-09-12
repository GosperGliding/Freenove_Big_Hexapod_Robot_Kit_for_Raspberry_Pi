#!/usr/bin/env python3
"""Generate the golden servo trace from the original Python implementation.

This is the oracle for the C++ port. It imports Code/Server/control.py
unmodified and runs it against test/scenarios.txt with every hardware
dependency stubbed out, recording each (channel, angle) pair the real code
would have written to the PCA9685.

Nothing here may "fix" the Python. Quirks are the specification.
"""

import os
import sys
import types
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
CPP_ROOT = os.path.dirname(HERE)
SERVER = os.path.normpath(os.path.join(CPP_ROOT, "..", "Server"))


# --- hardware stubs ---------------------------------------------------------
# Installed into sys.modules before control.py is imported, so the import graph
# resolves without a Raspberry Pi attached.

def _install_stubs():
    smbus = types.ModuleType("smbus")

    class SMBus:
        def __init__(self, bus):
            self.bus = bus

        def write_byte_data(self, addr, reg, value):
            pass

        def read_byte_data(self, addr, reg):
            return 0

        def close(self):
            pass

    smbus.SMBus = SMBus
    sys.modules["smbus"] = smbus

    gpiozero = types.ModuleType("gpiozero")

    class OutputDevice:
        def __init__(self, pin, *a, **kw):
            self.pin = pin
            self.value = 0

        def on(self):
            self.value = 1

        def off(self):
            self.value = 0

    class DistanceSensor:
        def __init__(self, *a, **kw):
            pass

    gpiozero.OutputDevice = OutputDevice
    gpiozero.DistanceSensor = DistanceSensor
    gpiozero.PWMSoftwareFallback = type("PWMSoftwareFallback", (Warning,), {})
    gpiozero.DistanceSensorNoEcho = type("DistanceSensorNoEcho", (Warning,), {})
    sys.modules["gpiozero"] = gpiozero

    mpu_mod = types.ModuleType("mpu6050")

    class mpu6050:
        ACCEL_RANGE_2G = 0x00
        GYRO_RANGE_250DEG = 0x00

        def __init__(self, address=0x68, bus=1):
            pass

        def set_accel_range(self, r):
            pass

        def set_gyro_range(self, r):
            pass

        def get_accel_data(self):
            return {"x": 0.0, "y": 0.0, "z": 9.8}

        def get_gyro_data(self):
            return {"x": 0.0, "y": 0.0, "z": 0.0}

    mpu_mod.mpu6050 = mpu6050
    sys.modules["mpu6050"] = mpu_mod


def build_control(trace):
    """Import control.py with hardware stubbed and instrumentation attached."""
    _install_stubs()
    sys.path.insert(0, SERVER)
    os.chdir(SERVER)          # control.read_from_txt('point') resolves via cwd

    import control as control_mod
    import servo as servo_mod
    import pca9685 as pca9685_mod

    # Wrap rather than replace, so the real servo.py angle-to-duty maths and
    # the real chip routing still run and can be compared too. Patch the class,
    # because control.py did `from servo import Servo` and holds the class
    # object, not the module.
    original_set_servo_angle = servo_mod.Servo.set_servo_angle

    def record_angle(self, channel, angle):
        trace.append(("w", channel, angle))
        original_set_servo_angle(self, channel, angle)

    servo_mod.Servo.set_servo_angle = record_angle

    # The bottom of the stack: which chip, which channel, what tick values.
    # Recorded immediately after the angle that produced it, so the two
    # interleave deterministically and one comparison covers both layers.
    def record_pwm(self, channel, on, off):
        trace.append(("t", self.address, channel, on, off))

    pca9685_mod.PCA9685.set_pwm = record_pwm

    # run_gait sleeps 0.01s per frame; a full sweep would take minutes.
    control_mod.time.sleep = lambda _seconds: None

    # set_leg_angles silently writes nothing when the target is unreachable.
    # That branch is part of the contract, so mark it in the trace.
    original_set_leg_angles = control_mod.Control.set_leg_angles

    def instrumented(self):
        if not self.check_point_validity():
            trace.append(("!range",))
        original_set_leg_angles(self)

    control_mod.Control.set_leg_angles = instrumented

    return control_mod.Control()


def parse_scenarios(path):
    out = []
    with open(path, "r") as handle:
        for raw in handle:
            line = raw.split("#", 1)[0].strip()
            if line:
                out.append(line.split())
    return out


def run(control, scenarios, trace):
    for index, parts in enumerate(scenarios):
        verb, args = parts[0], parts[1:]
        trace.append(("s", index, " ".join(parts)))
        if verb == "gait":
            control.run_gait(["CMD_MOVE"] + args)
        elif verb == "position":
            control.move_position(int(args[0]), int(args[1]), int(args[2]))
        elif verb == "attitude":
            points = control.calculate_posture_balance(
                int(args[0]), int(args[1]), int(args[2]))
            control.transform_coordinates(points)
            control.set_leg_angles()
        elif verb == "calibrate":
            control.calibrate()
            control.set_leg_angles()
        else:
            raise SystemExit("unknown scenario verb: " + verb)


def emit(trace, path):
    with open(path, "w", newline="\n") as handle:
        for entry in trace:
            if entry[0] == "w":
                handle.write("w %d %d\n" % (entry[1], entry[2]))
            elif entry[0] == "t":
                handle.write("t 0x%02x %d %d %d\n"
                             % (entry[1], entry[2], entry[3], entry[4]))
            elif entry[0] == "s":
                handle.write("s %d %s\n" % (entry[1], entry[2]))
            else:
                handle.write("!range\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", default=os.path.join(HERE, "reference.trace"))
    options = parser.parse_args()
    output = os.path.abspath(options.output)

    scenarios = parse_scenarios(os.path.join(HERE, "scenarios.txt"))
    trace = []
    control = build_control(trace)   # construction itself calls set_leg_angles
    run(control, scenarios, trace)
    emit(trace, output)

    writes = sum(1 for e in trace if e[0] == "w")
    ticks = sum(1 for e in trace if e[0] == "t")
    skips = sum(1 for e in trace if e[0] == "!range")
    print("scenarios: %d  servo writes: %d  pwm writes: %d  out-of-range frames: %d"
          % (len(scenarios), writes, ticks, skips))
    print("wrote " + output)


if __name__ == "__main__":
    main()
