/**
 * main.ino — ESP32 differential-drive line follower
 * ================================================
 * Integration ONLY. No control math, no driver details: those live in
 * the modules under src/. This file wires the modules together and runs
 * two INDEPENDENT, never-blocking loops:
 *
 *   FAST loop  (200 Hz, micros()-scheduled): PathController
 *   SLOW loop  (~1 Hz,  micros()-scheduled): PIDCalibrator (optional)
 *   Diagnostics: serial commands + 1 Hz heartbeat
 *
 * Architecture (task §26) — the dependency direction is:
 *
 *   LineSensor -> PathController -> DifferentialDrive -> Motors
 *   (calibration, when enabled, only OBSERVES and calls setGains())
 *
 * With CALIBRATION_ENABLED = false (see RobotConfig.h) the calibrator is
 * never constructed and the robot runs purely on the baseline gains.
 */

#include <Arduino.h>

#include "src/config/RobotConfig.h"
#include "src/config/PinConfig.h"

#include "src/sensors/RLS08LineSensor.h"
#include "src/sensors/MockSensors.h"

#include "src/motion/Motor.h"
#include "src/motion/DifferentialDrive.h"

#include "src/control/PathController.h"

#if CALIBRATION_ENABLED
#include "src/sensors/IMUInterface.h"
#include "src/calibration/PIDCalibrator.h"
#endif

#include "src/diagnostics/Diagnostics.h"
#include "src/test/SelfTest.h"

// ---------------------------------------------------------------------------
// Build options
// ---------------------------------------------------------------------------

/// Set true to run the self-test suite once in setup() (over Serial).
#define RUN_SELF_TEST_AT_BOOT false

/// Set true to use the mock line sensor instead of the RLS08 (bench test
/// without hardware). TODO(hardware): set false for the real robot.
#define USE_MOCK_LINE_SENSOR false

// ---------------------------------------------------------------------------
// Static configuration (no dynamic allocation anywhere)
// ---------------------------------------------------------------------------

static RobotConfig robotConfig;

static RLS08LineSensor::Config rls08Config() {
    RLS08LineSensor::Config c;
    c.pins = defaultRLS08Pins();
    // TODO(hardware): verify polarity and channel order for your board!
    c.lineIsHigh = true;
    c.reverseOrder = false;
    return c;
}

static Motor::Config leftMotorConfig() {
    Motor::Config c;
    const L298NPinConfig pins = defaultL298NPins();
    c.pwmPin = pins.ena; c.in1Pin = pins.in1; c.in2Pin = pins.in2;
    c.pwmChannel = 0;      ///< LEDC channel 0 (unique per motor)
    c.invert = false;      ///< TODO(hardware): flip if a motor runs backwards
    return c;
}

static Motor::Config rightMotorConfig() {
    Motor::Config c;
    const L298NPinConfig pins = defaultL298NPins();
    c.pwmPin = pins.enb; c.in1Pin = pins.in3; c.in2Pin = pins.in4;
    c.pwmChannel = 1;
    c.invert = false;      ///< TODO(hardware): flip if a motor runs backwards
    return c;
}

// ---------------------------------------------------------------------------
// Module instances (static storage, constructed once)
// ---------------------------------------------------------------------------

static RLS08LineSensor rls08(rls08Config());

// Mock sensor script: a gentle sine sweep across the array, used when
// USE_MOCK_LINE_SENSOR is true. Period 20 ms per step.
static const float kMockPositions[] = {
    0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.4f, 0.3f,
    0.2f, 0.1f, 0.0f, -0.1f, -0.2f, -0.3f, -0.2f, -0.1f
};
static MockLineSensor mockLineSensor(kMockPositions,
                                      sizeof(kMockPositions) / sizeof(kMockPositions[0]),
                                      20000);

static Motor        leftMotor(leftMotorConfig(), robotConfig.motion);
static Motor        rightMotor(rightMotorConfig(), robotConfig.motion);
static DifferentialDrive drive(leftMotor, rightMotor,
                               robotConfig.geometry, robotConfig.motion);

/// The fast controller runs against whichever sensor is selected.
static LineSensor& activeLineSensor() {
#if USE_MOCK_LINE_SENSOR
    return mockLineSensor;
#else
    return rls08;
#endif
}

static PathController fastController(activeLineSensor(), drive, robotConfig);

#if CALIBRATION_ENABLED
// The IMU is UNKNOWN hardware (task §5, §24): the baseline runs with a
// nullptr IMU; plug a concrete IMUInterface implementation in here when
// the model is chosen. Calibration degrades gracefully without it.
static IMUInterface* imu = nullptr;  // TODO(hardware): instantiate real IMU

/**
 * Adapter that exposes the PathController to the calibrator through the
 * narrow CalibrationTarget interface.
 *
 * WHY an adapter instead of making PathController inherit
 * CalibrationTarget: the fast control layer must not include or depend
 * on ANY calibration header (task §26 dependency direction). The
 * coupling happens here, in the integration file, where the two layers
 * are allowed to meet.
 */
class PathControllerCalibrationAdapter : public CalibrationTarget {
public:
    explicit PathControllerCalibrationAdapter(PathController& c) : c_(c) {}
    void setGains(const PIDGains& g) override { c_.setGains(g); }
    const PIDGains& getGains() const override { return c_.getGains(); }
    float getLastLineError() const override { return c_.getLastLineError(); }
    bool  isLineLost() const override { return c_.isLineLost(); }
private:
    PathController& c_;
};

static PathControllerCalibrationAdapter calibrationTarget(fastController);
static PIDCalibrator calibrator(calibrationTarget, imu, robotConfig.calibration);
#endif

static Diagnostics diagnostics(robotConfig);

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void setup() {
    fastController.begin();

#if CALIBRATION_ENABLED
    // Calibration starts DISABLED even when compiled in; enable it via
    // the serial command "cal on" or by calling calibrator.enable() here.
#endif

    diagnostics.attach(&fastController, &rls08,
#if CALIBRATION_ENABLED
                        &calibrator
#else
                        nullptr
#endif
    );
    diagnostics.begin();

#if RUN_SELF_TEST_AT_BOOT
    selftest::runAll();
#endif
}

void loop() {
    const uint32_t now = micros();

    // ---- FAST control loop (200 Hz, deterministic, never blocks) -------
    if (fastController.shouldRun(now)) {
        fastController.update(now);

#if CALIBRATION_ENABLED
        // Feed the supervisor's observers. This is a cheap write INTO the
        // calibrator; the calibrator never calls back into this loop
        // except through setGains() (a non-blocking assignment).
        // NOTE: only while calibration is active.
        if (calibrator.isEnabled()) {
            calibrator.onFastLoopSample(now);
        }
#endif
    }

#if CALIBRATION_ENABLED
    // ---- SLOW supervisor loop (~1 Hz), fully independent timing -------
    if (calibrator.shouldRun(now)) {
        calibrator.update(now);
    }
#endif

    // ---- Diagnostics (serial commands + rate-limited heartbeat) -------
    diagnostics.update(now);
}
