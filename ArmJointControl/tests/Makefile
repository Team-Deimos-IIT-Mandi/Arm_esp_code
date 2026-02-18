CXX = g++
CXXFLAGS = -Wall -Imocks -I../include -std=c++11

UTILS_SRC = ../src/Utils.cpp
JOINT_SRC = ../src/Joint.cpp
MOCK_HARDWARE_SRC = mocks/Hardware.cpp

all: run_tests

test_utils: test_utils.cpp $(UTILS_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@

test_joint: test_joint.cpp $(JOINT_SRC) $(UTILS_SRC) $(MOCK_HARDWARE_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@

test_comms: test_comms.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@

run_tests: test_utils test_joint test_comms
	./test_utils
	./test_joint
	./test_comms

clean:
	rm -f test_utils test_joint
