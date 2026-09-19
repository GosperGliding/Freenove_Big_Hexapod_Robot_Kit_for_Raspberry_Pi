# Hexapod kinematics, gait, and servo driver, in C++

A port of `Code/Server/control.py` — inverse kinematics, body-frame transforms,
calibration, both gait generators — plus a PCA9685 driver, servo power control,
and a paced runner that actually walks the robot.

The computation half is **bit-exact** against the Python original: identical
joint angles *and* identical PWM tick values, in identical order, over the whole
scenario set. That is enforced by a differential test rather than asserted.

## Build

```
make          # host tools and unit tests (Linux also builds hexapod_walk)
make test     # unit tests, then the differential test
make robot    # hexapod_walk only; Linux only
```

The x86-only flags (`-msse2 -mfpmath=sse`) are gated on `uname -m`, so the same
Makefile works on a development machine and on the Pi.

## Running it on the robot

```bash
make test                       # do this FIRST, on the Pi -- see below
make robot
sudo ./hexapod_walk --dry-run   # gait maths and pacing, no hardware touched
sudo ./hexapod_walk             # 3 tripod cycles forward
```

`--help` lists every option. Useful ones:

```
--gait 2            wave gait instead of tripod
--y -35             walk backwards
--angle 10          turn while walking
--list              show every gait pattern and dance
--pattern ripple    phase-based gait engine: tripod, ripple, wave
--dance twerk       a routine instead of walking; --list shows them all
--frames 90         frames per cycle for --pattern and --dance (default 60)
--twerk-bias 100    0..100, rear-vs-front differential (default 90)
--straighten        hold the assembly reference pose and wait
--height 60         stand taller; -20..80, default 40
--period-ms 20      faster frames -- see the warning below
--cycles 10
--arm-delay-ms 0    skip the pre-arm pause
--keep-powered      leave the servo rail on at exit
```

### Working without batteries

Most of this can be exercised with no battery pack at all.

| what | needs |
|---|---|
| `make test`, `make`, `bash test/mutate.sh` | nothing but a laptop |
| `make test` on ARM — settles the libm question | Pi on USB power |
| `make robot` — compile against the real Linux headers | Pi on USB power |
| `./hexapod_walk --dry-run` — gait maths, pacing, overrun stats | Pi on USB power |
| `sudo ./hexapod_walk --no-servo-power` — the whole I2C driver for real | Pi + shield, *if* the PCA9685 logic rail is Pi-powered |
| actual motion | batteries |

To find out whether the second-to-last row applies to your board:

```bash
i2cdetect -y 1
```

`0x40` and `0x41` are the two PCA9685s; `0x48` is the ADS7830 that reads
battery voltage. If they show up with no batteries fitted, their logic supply
comes from the Pi, and `--no-servo-power` will exercise chip configuration,
register writes, batching, transaction counts and real bus timing — everything
except the servos moving, since their V+ rail is dead.

`--no-servo-power` is also the right way to make the *first* run with batteries
fitted: identical I2C traffic, rail never energised, nothing moves.

### Gaits and dances

`Control::run_gait` is the faithful port and stays untouched. Alongside it sits
a second engine where a gait is *data* -- a phase offset per leg and a duty
factor -- rather than a chain of frame-index branches. Foot position is an
absolute function of phase, so nothing accumulates, and swing follows a
half-sine: a foot leaves and meets the ground with zero vertical velocity,
instead of the original jump to full height on the first frame.

| `--pattern` | feet down | character |
|---|---|---|
| `tripod` | 3 | fastest, least margin |
| `ripple` | 4 | travelling wave, smooth compromise |
| `wave` | 5 | slowest, steadiest |

| `--dance` | |
|---|---|
| `sway` | rolls side to side |
| `twist` | rotates the body about its centre |
| `circle` | roll and pitch in quadrature; the body describes a cone (tilt adapts to ride height) |
| `bob` | rises and dips on the spot |
| `pushup` | dips deep and presses back up |
| `wave` | plants five legs and waves the sixth |
| `twerk` | rear-biased double-time bounce with a hip sway |
| `moonwalk` | glides backwards with the feet skimming the floor |
| `show` | staged: rise circling, half turn, twerk, sink circling |

`sway` through `twerk` keep six feet planted, so the support polygon never
changes and they hold at amplitudes a gait could not.

**`twerk`** dips 28 mm at the rear, distributed across the legs by
`--twerk-bias`. The differential does more for how the motion reads than the
amplitude does, which is why it is the knob that got exposed rather than the
dip:

| `--twerk-bias` | front legs take | reads as |
|---|---|---|
| 0 | 100% | a plain bob, no differential |
| 85 | 15% | the first version |
| 90 (default) | 10% | |
| 100 | 0% | body hinges about its front feet |

It applies to the twerk phase of `show` too.

**`show`** is a four-phase routine rather than one repeating beat, about 20
seconds at the defaults: spirals up to ride height 60 over two circles, turns
180 degrees on the ripple pattern, twerks with a slow circle laid over it, then
spirals down to the floor. `--cycles` sets the length of the twerk phase only.

**`circle`** picks its own shape. It searches two axes before emitting a
frame: how far the body leans, and how far in the feet are drawn. Drawing the
feet in unloads the outer reach limit and buys a lot of lean.

| ride height | tilt | feet at |
|---|---|---|
| 0 | 60 degrees | 90% |
| 40 (default) | 36 degrees | 92% |
| 80 | 24 degrees | 86% |

Tucking is **not** monotonic, which is why this is a search and not a
constant. Past roughly 0.85 the envelope's *lower* bound takes over -- the leg
folds in so tight the foot comes closer to the coxa than 90 mm -- and the
available tilt collapses. At ride height 0 it runs 47 degrees at nominal
stance, 60 at 0.9, then 11 at 0.8.

Probing moves nothing: `transform_coordinates` only writes `leg_positions`,
and the servos see nothing until `set_leg_angles` is called.

Stand the robot lower with `--height` and it leans further. The tuck is capped
at 0.86 -- deeper would pass the reach check but shrinks the polygon the centre
of mass has to stay inside while the body is leaning hard.

**`moonwalk`** is an honest approximation. The real illusion needs one foot
sliding while another takes weight, and six legs in a wave never give that
moment. What survives is what the eye actually reads: feet skimming 7 mm
instead of stepping, and a forward lean against the direction of travel.

`test/unit_test.cpp` runs every pattern at worst-case stride and every routine,
at ride heights 0, 40 and 80, and asserts Control never rejects a frame as
unreachable. Both tables are iterated, so a new entry is covered the moment it
is added.

### Straightening the legs

```bash
sudo ./hexapod_walk --straighten
```

Holds all 32 channels at the angles `servo.py` uses while horns are fitted --
90 degrees, except 10/13/31 at 10 and 18/21/27 at 170 -- and waits for Ctrl-C.

These are raw channel angles, deliberately bypassing Control: no IK, no
`point.txt` calibration. That is the point. Nothing in this robot reads a joint
position back, so this pose IS the reference the whole kinematic chain is
measured against. A leg fitted one spline tooth off is permanently wrong by
that tooth, and no amount of calibration downstream can see it.

Fix the mechanics here first, then calibrate, then tune ride height -- in that
order. Each step assumes the one before it.

### Ride height

`body_height` defaults to -25 mm, which leaves the chassis on the ground -- the
gait runs correctly and the robot drags itself instead of stepping.
`--height N` sets it to `-30 - N` and ramps there a millimetre per frame.

The original protocol clamps this to +/-20, but that is a limit of the client,
not of the legs. Peak leg reach against the worst case the gait can produce (a
full diagonal stride, x and y both saturated):

| `--height` | body height | peak reach | margin to 233 mm |
|---|---|---|---|
| 20 | -50 mm | 196 mm | 37 mm |
| 40 (default) | -70 mm | 204 mm | 29 mm |
| 60 | -90 mm | 214 mm | 20 mm |
| 80 (max) | -110 mm | 224 mm | 9 mm |

Past 80 the legs run out of reach mid-stride and the IK clamps silently. The
reach limit is not the practical one though: at 80 the leg is 96% extended,
with almost no mechanical advantage left to hold the body up.

### Silent failures the tool now reports

Two states used to look identical to a clean run, because neither shows up on
the I2C side:

**A flat servo pack.** The PCA9685s take their logic supply from the Pi, so
every register write succeeds and the run reports its usual transaction count
while nothing moves. `hexapod_walk` now reads both packs through the ADS7830 at
startup and warns below the 7 V minimum. Treat the number as indicative rather
than calibrated — `adc.py` hardcodes a divider coefficient its own comment says
is PCB-version dependent, and never looks the version up. It is reliable for
"flat or not", which is what it is used for.

**A chip that stops acknowledging.** `Pca9685Bus::flush()` always knew when a
write was refused and threw the result away. Failures are now counted and
reported on the summary line.

### Read this before the first run

**Startup is a full-authority move.** Constructing `Control` calibrates and
immediately drives all 18 joints to the stance pose, from wherever the legs
happen to be. The Python does exactly the same thing, so it is not new, but it
is the *first* thing that happens. `hexapod_walk` pauses 3 s and warns first;
support the body, and keep the power switch reachable.

**Lowering `--period-ms` makes the robot walk faster, not smoother.** The stride
per cycle is fixed, so a shorter frame period means higher body velocity. The
default 30 ms approximates the original's *effective* rate (~21 ms of I2C plus
its 10 ms sleep — its nominal 100 Hz was never real). Batched I2C leaves plenty
of headroom to go faster; the servos are what limit you, not the bus.

**Stop the Python server first.** It holds BCM 4 through gpiozero, and the GPIO
line request here will fail with `Device or resource busy` if it is running.

### The bit-exactness result does not transfer from x86 automatically

`make test` passing on a laptop says nothing about the Pi: `sin`, `cos`, `asin`,
`acos` and `atan2` are not correctly-rounded, and ARM's libm can differ from
x86's in the last ulp. The harness is built to settle this — it runs entirely on
the robot, comparing the Pi's CPython against the Pi's C++ using the same libm.
Run `make test` on the Pi before trusting anything else.

## How the differential test works

```
test/scenarios.txt   one shared list of commands, executed in order
      |
      +--> test/gen_reference.py --> test/reference.trace   (original Python)
      +--> hexapod_trace         --> test/cpp.trace         (this port)
                                      |
                                 test/compare.py  -> must be identical
```

`gen_reference.py` imports `Code/Server/control.py` **unmodified**, with
`smbus`, `gpiozero` and `mpu6050` stubbed. It wraps rather than replaces the
servo layer, so the real `servo.py` angle-to-duty maths and the real chip
routing run too. Every trace carries two interleaved lines per joint:

```
w 15 90            joint angle: flat channel 15 -> 90 degrees
t 0x41 15 0 307    what that becomes: chip 0x41, channel 15, ON 0, OFF 307
```

Scenarios are stateful on purpose: `move_position` mutates `body_height`, so a
gait that runs after one must see the mutated stance.

## Checking the test itself

```
bash test/mutate.sh
```

Reintroduces seven specific porting mistakes and reports which the harness
catches. Five are caught. Two are not:

- `math.degrees` spelled as `(x*180)/pi` instead of `x*(180/pi)`
- mount bearing spelled as `deg*(pi/180)` instead of `(deg/180)*pi`

Both differ only in the last ulp and never enough to move a rounded integer
degree over this scenario set. They are matched anyway because it costs nothing,
but they are precautionary rather than load-bearing.

`test/unit_test.cpp` covers what a trace cannot reach: the batching algorithm
(no trace line corresponds to a transaction) and the edges of the channel and
angle ranges.

## What had to be preserved exactly

| behaviour | where | why it matters |
|---|---|---|
| `round()` is round-half-to-even | `pyround.hpp` | every joint angle passes through it; `std::round` is half-away-from-zero |
| `round(x, 2)` before `asin`/`acos` | `kinematics.cpp` | quantises the IK result; dropping it changes angles |
| axis shuffle `(-z, x, y)` into the IK | `Control::solve_leg` | the IK frame is *not* the leg frame |
| asymmetric left/right mirroring | `Control::set_leg_angles` | `90 - x` one side, `90 + x` and `180 - x` the other |
| phase boundaries in float (`3*F/8`) | `Control::run_gait` | integer division shifts the gait phases |
| servo write *order* | `Control::channel_map` | legs go out 1,2,3,6,5,4, and leg 3's tibia is on the other chip |
| ticks are truncated, not rounded | `pwm::angle_to_ticks` | the original ends with `int(duty_cycle)` |
| BCM 4 is an active-high *disable* | `ServoPower` | drive it low to power the servos; invert this and nothing moves |

## What is deliberately different

**Batched I2C.** The original issues four transactions per channel — 72 per
frame — because it never sets the PCA9685 auto-increment bit. `Pca9685Bus`
stages a frame and flushes each contiguous run of touched channels in one
transaction: **4 per frame** for this robot's channel map. At the Pi's default
100 kHz that is roughly 21 ms down to about 7 ms per frame; raising the bus to
400 kHz (`dtparam=i2c_arm_baudrate=400000`) takes it under 2 ms.

Runs rather than whole banks, because a full 16-channel write would also clobber
channels this program never set — the head pan/tilt servos sit on 0x41 channels
0 and 1.

**No sleeping inside the gait.** `Control::run_gait` has no timing policy at
all; `PacedBus` wraps the bus and holds an absolute deadline that advances by
exactly one period per frame, so a slow frame does not push every later frame
late. It reports overrun count and worst overrun, which is how you tell whether
the period you picked is real. Skipped (unreachable) frames are paced too — the
original sleeps on those as well.

**Hardware behind one interface.** `ServoBus` has three implementations:
`TraceBus` (writes a file), `NullBus` (counts), `Pca9685Bus` (I2C). Nothing
above that interface knows hardware exists.

## Layout

```
include/hexapod/  pyround.hpp  types.hpp  kinematics.hpp  control.hpp
                  servo_bus.hpp  pwm_encoding.hpp
                  pca9685_bus.hpp  servo_power.hpp  paced_bus.hpp   (Linux)
src/              kinematics.cpp  control.cpp  pwm_encoding.cpp
                  pca9685_bus.cpp  servo_power.cpp  paced_bus.cpp   (Linux)
tools/            trace_main.cpp        the C++ trace generator
                  hexapod_walk.cpp      the robot runner            (Linux)
test/             scenarios.txt  gen_reference.py  compare.py
                  unit_test.cpp  mutate.sh
```

## Known issue in the original, carried over

At full diagonal stride (`x` and `y` both saturated at 35), the tripod gait
drives feet outside the 90–248 mm reach envelope on frames 2, 3, 7 and 8 of a
22-frame cycle. `set_leg_angles` writes nothing on those frames, so the legs
freeze for 18% of the cycle and then jump. This is reproduced faithfully rather
than fixed, because fixing it would break the differential test — but it is a
real stride-limit bug and a good argument for making the reach check a hard
error once the port is trusted.
