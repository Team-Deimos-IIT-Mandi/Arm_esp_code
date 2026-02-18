CXX = g++
CXXFLAGS = -Wall -Imocks -I../include -std=c++11
BUILD_DIR = build

UTILS_SRC = ../src/Utils.cpp
JOINT_SRC = ../src/Joint.cpp
MOCK_HARDWARE_SRC = mocks/Hardware.cpp

TARGETS = $(BUILD_DIR)/test_utils $(BUILD_DIR)/test_joint $(BUILD_DIR)/test_comms

all: $(BUILD_DIR) $(TARGETS) run_tests

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/test_utils: test_utils.cpp $(UTILS_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD_DIR)/test_joint: test_joint.cpp $(JOINT_SRC) $(UTILS_SRC) $(MOCK_HARDWARE_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD_DIR)/test_comms: test_comms.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

run_tests: $(TARGETS)
	$(BUILD_DIR)/test_utils
	$(BUILD_DIR)/test_joint
	$(BUILD_DIR)/test_comms

clean:
	rm -rf $(BUILD_DIR)
