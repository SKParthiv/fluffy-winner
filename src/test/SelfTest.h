/**
 * SelfTest.h
 * ==========
 * Unit-test-style checks for the mathematical components (task §22),
 * runnable on the ESP32 at boot (RUN_SELF_TEST_AT_BOOT) or on a host
 * PC via test/host_test.cpp (plain g++).
 *
 * Covered:
 *   PIDController      : zero error, +/- error, saturation, windup,
 *                        derivative with variable dt
 *   PointKinematics    : body/world point-velocity -> (v, omega)
 *   DifferentialDrive  : wheel mixing + inverse
 *   CalibrationMetrics : metric computation on synthetic windows
 *   PIDCalibrator      : candidate accept / reject / failure / recovery
 *
 * The tests use MockLineSensor + MockIMU and a MockDrive so no hardware
 * is touched. FAILURES print over Serial (rate-limited by nature: the
 * suite runs once).
 */

#ifndef SELF_TEST_H
#define SELF_TEST_H

#include "../config/RobotConfig.h"

namespace selftest {

/// Runs all checks. Returns the number of failures (0 = pass).
/// Each check prints "[PASS]/[FAIL] name" via Serial (or printf on host).
int runAll();

}  // namespace selftest

#endif // SELF_TEST_H
