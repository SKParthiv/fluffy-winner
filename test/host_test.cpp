/**
 * host_test.cpp
 * =============
 * Host-PC build of the self-test suite (no Arduino hardware needed):
 *
 *   g++ -std=c++17 -Isrc -Itest/arduino_stubs \
 *       test/host_test.cpp src/control/*.cpp src/motion/*.cpp \
 *       src/calibration/*.cpp src/test/SelfTest.cpp -o host_test && ./host_test
 *
 * A minimal Arduino stub header provides micros()/Serial/etc. so the
 * same source files compile on ESP32 and on a desktop compiler.
 */

#include <cstdio>
#include <cstdint>

// ---- Arduino API stubs used by the modules under test -------------------
static uint64_t g_virtualMicros = 1000000;
uint32_t micros() { return (uint32_t)g_virtualMicros; }
uint32_t millis() { return (uint32_t)(g_virtualMicros / 1000); }

#include "test/SelfTest.h"

int main() {
    const int failures = selftest::runAll();
    return failures == 0 ? 0 : 1;
}
