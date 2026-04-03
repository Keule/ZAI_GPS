# ============================================================================
# AgSteer PC Simulation Makefile
#
# Usage:
#   cd pc_sim && make        # build
#   cd pc_sim && make run    # build + run
#   cd pc_sim && make clean  # remove binary
# ============================================================================

CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall
INCLUDES := -I../src -I../include -I.

# --- Source files ---
MAIN     := main_pc.cpp
HAL_PC   := hal_pc/hal_impl.cpp
LOGIC    := ../src/logic/global_state.cpp \
           ../src/logic/aog_udp_protocol.cpp \
           ../src/logic/gnss.cpp \
           ../src/logic/imu.cpp \
           ../src/logic/steer_angle.cpp \
           ../src/logic/actuator.cpp \
           ../src/logic/control.cpp \
           ../src/logic/net.cpp \
           ../src/logic/hw_status.cpp

SRCS     := $(MAIN) $(HAL_PC) $(LOGIC)
TARGET   := sim

.PHONY: all run clean

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ -lm -lpthread

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
