# Hexapod — project contract

**The Raspberry Pi is the target. This Windows machine is a scratchpad.**

Everything here ships to a Pi 4 driving a Freenove Big Hexapod over I2C. The
live work is `Code/Cpp` — a bit-exact C++ port of `Code/Server/control.py`.
`Code/Cpp/README.md` documents *what the port is and why*; this file is the
*working contract*. Do not duplicate one into the other.

## Edit for the Pi, always

Write for ARM Linux first. Concretely:

- **Never let x86 assumptions into the build.** `-msse2 -mfpmath=sse` are gated
  on `uname -m` in the Makefile and must stay gated — ARM's GCC rejects them
  outright. `-ffp-contract=off` must stay: ARM has hardware FMA and GCC
  contracts by default, which silently widens intermediates and breaks
  bit-exactness. **Never add `-ffast-math`** — `py_round` depends on the
  rounding mode staying `FE_TONEAREST`.
- **Assume 64-bit ARM, but verify before relying on it.** Word size, `long`,
  and char signedness differ from x86. `char` is *unsigned* on ARM by default;
  any code that stores a byte in a bare `char` and compares it to a negative
  value works here and fails there.
- **Prefer the primitive.** This port already talks to `/dev/i2c-1` and
  `/dev/gpiochipN` directly rather than through a wrapper library. Keep it
  that way; do not reintroduce a dependency to save a few lines.

## What this machine can and cannot verify

| half | files | here |
|---|---|---|
| portable computation | `kinematics` `control` `pwm_encoding`, `trace_main`, `unit_test` | builds, runs, `make test` passes |
| robot / hardware | `pca9685_bus` `paced_bus` `servo_power` `hexapod_walk` | **cannot even compile** |

The robot half needs `<linux/gpio.h>`, `<linux/i2c-dev.h>`, `<sys/ioctl.h>`.
The Makefile gates it on `uname -s` = Linux, so on Windows it is silently
skipped — `make` succeeding here says *nothing* about those four files.

WSL Ubuntu is installed but has no `g++` and no `linux-libc-dev`. Installing
both would give an x86-Linux compile check for the robot half — type and API
errors only, never ARM behaviour. That is worth doing; it is not done yet.

**When editing the robot half, say plainly that the change is unverified until
it runs on the Pi.** Do not report it as working.

## Bit-exactness does not transfer from x86

`sin` `cos` `asin` `acos` `atan2` are not correctly-rounded, and ARM's libm can
differ from x86's in the last ulp. A green `make test` on this laptop is
necessary and **not sufficient**. The harness exists to settle this *on the
robot*: it runs the Pi's CPython against the Pi's C++ using the same libm.

> `make test` must pass **on the Pi** before any result is trusted.

## Deployment

The Pi pulls from git — it is not pushed to from here. There is no SSH key on
this machine (`192.168.1.193`, password auth only), so Pi-side commands are run
by Patricia, not by Claude.

This means **anything unpushed does not exist as far as the robot is
concerned.** Commit and push before asking for a Pi-side test.

**Never commit build artifacts.** `.gitignore` covers them, but it was added
one commit *after* the port, so 30 artifacts stayed tracked until `6d275ac`.
That broke the Pi deterministically: git checks out in index order, so `.o`
always lands newer than its `.cpp`, make skips every compile, and the ARM
linker gets Intel COFF objects. Re-check with
`git ls-files | grep -E '\.(o|d|exe|trace|pyc)$'` — only
`Application/windows/windows.exe` should match.

## Before anything moves

- **Startup is a full-authority move.** Constructing `Control` calibrates and
  immediately drives all 18 joints to the stance pose from wherever they are.
  It is the *first* thing that happens. The Python does the same, so it is not
  new — but support the body and keep the power switch reachable.
- **Stop the Python server first.** It holds BCM 4 via gpiozero; the GPIO line
  request will fail with `Device or resource busy`.
- Escalate in order: `--dry-run` → `--no-servo-power` → powered.
  `--no-servo-power` is the correct first run with batteries fitted: identical
  I2C traffic, rail never energised, nothing moves.

## Known real bug, deliberately preserved

At full diagonal stride (`x` and `y` both 35), the tripod gait drives feet
outside the 90–248 mm reach envelope on 4 of 22 frames; `set_leg_angles` writes
nothing, so the legs freeze for 18% of the cycle and jump. Faithful to the
Python. Fixing it breaks the differential test, so it waits until the port is
trusted on hardware — then the reach check should become a hard error.
