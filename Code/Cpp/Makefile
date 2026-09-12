CXX      := g++
PYTHON   := python

UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)

CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wpedantic -Iinclude

# Bit-exactness against the Python reference depends on these. Without them
# the port is still "correct" in the usual sense but the differential test goes
# red on last-ulp noise, which hides real defects.
#
#   -ffp-contract=off   stop GCC fusing a*b+c into an FMA, which keeps a wider
#                       intermediate and changes the result. Costs nothing on
#                       x86; matters a great deal on ARM, where the hardware
#                       has FMA and GCC contracts by default.
#
# Never add -ffast-math: py_round depends on the rounding mode staying
# FE_TONEAREST, which is what makes it match Python's round-half-to-even.
CXXFLAGS += -ffp-contract=off

# x87 80-bit intermediates are an x86 problem only. On ARM these options do not
# exist and the compiler rejects them outright, so they must never reach a Pi
# build -- gate them on the host architecture rather than hardcoding them.
ifneq (,$(filter i386 i486 i586 i686 x86_64 amd64,$(UNAME_M)))
CXXFLAGS += -msse2 -mfpmath=sse
endif

# Header dependencies. Without these, editing a header does not rebuild the
# objects that include it -- and since the rounding contract lives entirely in
# pyround.hpp, a stale object silently tests the wrong code.
CXXFLAGS += -MMD -MP

# Portable: pure computation, builds and runs anywhere.
LIB_SRC  := src/kinematics.cpp src/control.cpp src/pwm_encoding.cpp
TOOL_SRC := tools/trace_main.cpp
UNIT_SRC := test/unit_test.cpp

BIN      := hexapod_trace
UNIT_BIN := hexapod_unit

# Robot-only: opens /dev/i2c-N and /dev/gpiochipN.
ROBOT_SRC  := src/pca9685_bus.cpp src/paced_bus.cpp src/servo_power.cpp
ROBOT_TOOL := tools/hexapod_walk.cpp
ROBOT_BIN  := hexapod_walk

HOST_OBJ := $(LIB_SRC:.cpp=.o) $(TOOL_SRC:.cpp=.o) $(UNIT_SRC:.cpp=.o)

ifeq ($(UNAME_S),Linux)
ROBOT_OBJ := $(ROBOT_SRC:.cpp=.o) $(ROBOT_TOOL:.cpp=.o)
ROBOT_TARGETS := $(ROBOT_BIN)
else
ROBOT_OBJ :=
ROBOT_TARGETS :=
endif

OBJ := $(HOST_OBJ) $(ROBOT_OBJ)
DEP := $(OBJ:.o=.d)

REFERENCE := test/reference.trace
ACTUAL := test/cpp.trace

all: $(BIN) $(UNIT_BIN) $(ROBOT_TARGETS)

$(BIN): $(LIB_SRC:.cpp=.o) $(TOOL_SRC:.cpp=.o)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(UNIT_BIN): $(LIB_SRC:.cpp=.o) $(UNIT_SRC:.cpp=.o)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

ifeq ($(UNAME_S),Linux)
$(ROBOT_BIN): $(LIB_SRC:.cpp=.o) $(ROBOT_SRC:.cpp=.o) $(ROBOT_TOOL:.cpp=.o)
	$(CXX) $(CXXFLAGS) -o $@ $^

robot: $(ROBOT_BIN)
else
robot:
	@echo "hexapod_walk builds on Linux only: it opens /dev/i2c-N and /dev/gpiochipN."
	@echo "Copy Code/Cpp to the Pi and run make there."
	@exit 1
endif

# Unit tests, then the differential test: generate both traces from the same
# scenario list and the same calibration file, then require them identical.
test: $(BIN) $(UNIT_BIN)
	./$(UNIT_BIN)
	$(PYTHON) test/gen_reference.py -o $(REFERENCE)
	./$(BIN) test/scenarios.txt ../Server/point.txt $(ACTUAL)
	$(PYTHON) test/compare.py $(REFERENCE) $(ACTUAL)

# Build artifacts only. The mutation script calls this between every mutation,
# so it must not touch the reference trace it is comparing against.
clean:
	rm -f $(OBJ) $(DEP) $(BIN) $(BIN).exe $(UNIT_BIN) $(UNIT_BIN).exe \
	      $(ROBOT_BIN) $(ROBOT_BIN).exe

distclean: clean
	rm -f $(REFERENCE) $(ACTUAL) test/mutant.trace

-include $(DEP)

.PHONY: all robot test clean distclean
