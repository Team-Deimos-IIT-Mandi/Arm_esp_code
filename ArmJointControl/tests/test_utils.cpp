#include <iostream>
#include <cassert>
#include <cmath>
#include "../include/Utils.h"

void test_wrapValue() {
    std::cout << "Testing wrapValue..." << std::endl;
    assert(wrapValue(500) == 500);
    assert(wrapValue(4096) == 0);
    assert(wrapValue(4500) == 404);
    assert(wrapValue(-1) == 4095);
    assert(wrapValue(-4096) == 0);
    std::cout << "  Passed!" << std::endl;
}

void test_wrapError() {
    std::cout << "Testing wrapError..." << std::endl;
    assert(wrapError(100) == 100);
    assert(wrapError(2048) == 2048); // Boundary case, current implementation doesn't wrap 2048 to -2048 but 2048 is fine as ENCODER_HALF
    assert(wrapError(2049) == -2047);
    assert(wrapError(-2049) == 2047);
    std::cout << "  Passed!" << std::endl;
}

void test_applyDeadzone() {
    std::cout << "Testing applyDeadzone..." << std::endl;
    assert(applyDeadzone(2.0, 5.0) == 0.0);
    assert(applyDeadzone(-2.0, 5.0) == 0.0);
    assert(applyDeadzone(10.0, 5.0) == 5.0);
    assert(applyDeadzone(-10.0, 5.0) == -5.0);
    std::cout << "  Passed!" << std::endl;
}

void test_conversions() {
    std::cout << "Testing conversions..." << std::endl;
    // motorDegToSteps(360) -> 4096
    assert(motorDegToSteps(360.0f) == 4096.0f);
    assert(motorStepsToDeg(4096.0f) == 360.0f);
    
    // M3 Gear ratio is 2.0f
    // outputToMotorDeg(90) -> 180
    assert(outputToMotorDeg(90.0f) == 180.0f);
    assert(motorToOutputDeg(180.0f) == 90.0f);

    std::cout << "  Passed!" << std::endl;
}

int main() {
    test_wrapValue();
    test_wrapError();
    test_applyDeadzone();
    test_conversions();
    std::cout << "All Utils tests passed!" << std::endl;
    return 0;
}
