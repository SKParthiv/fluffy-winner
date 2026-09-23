/**
 * main.ino — ESP32 differential-drive line follower
 * ================================================
 * Integration ONLY. No control math, no driver details: those live in
 * the modules under src/. This file wires the modules together and runs
 * independent, never-blocking loops:
 *
 *   FAST loop  (200 Hz, micros()-scheduled): PathController
 *   UI loop    (every pass, internally rate-limited): OLED menu + buttons
 *   SLOW loop  (~1 Hz,  micros()-scheduled): PIDCalibrator (optional)
 *   Diagnostics: serial commands + 1 Hz heartbeat
 *
 * Architecture — the dependency direction is:
 *
 *   LineSensor -> PathController -> DifferentialDrive -> Motors
 *   (calibration, when enabled, only OBSERVES and calls setGains())
 *   OLED <-> 4-button UI -> configuration / calibration / system state
 *
 * With CALIBRATION_ENABLED = false (RobotConfig.h) the calibrator is
 * never constructed and the robot runs purely on the baseline gains.
 *
 * The IMU (MPU6050) is deliberately COMPLETELY ABSENT from this build:
 * no sensing, no fusion, no calibration, no test code. It is future
 * functionality; nothing here fails or blocks because it is missing.
 *
 * Boot behaviour: the system starts in the OFF state (motors stopped).
 * Turn it ON via the OLED menu (SYSTEM) — the robot must never start
 * driving the moment it is powered.
 */

#include <Arduino.h>

#include "src/config/RobotConfig.h"
#include "src/config/PinConfig.h"

#include "src/sensors/RLS08LineSensor.h"
#include "src/sensors/MockSensors.h"

#include "src/motion/Motor.h"
#include "src/motion/DifferentialDrive.h"

#include "src/control/PathController.h"

#include "src/system/PersistentConfig.h"

#include "src/ui/Display.h"
#include "src/ui/Buttons.h"
#include "src/ui/Menu.h"

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
    // TODO(hardware): verify polarity and channel order with test_sensor.ino!
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
// The IMU is future functionality (MPU6050 NOT implemented in this
// milestone): the baseline runs with a nullptr IMU; plug a concrete
// IMUInterface implementation in here when IMU support is added.
static IMUInterface* imu = nullptr;  // TODO(future): instantiate MPU6050

/**
 * Adapter that exposes the PathController to the calibrator through the
 * narrow CalibrationTarget interface.
 *
 * WHY an adapter instead of making PathController inherit
 * CalibrationTarget: the fast control layer must not include or depend
 * on ANY calibration header. The coupling happens here, in the
 * integration file, where the two layers are allowed to meet.
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

// ---- UI: OLED + four buttons ----------------------------------------------

static Display display(defaultOLEDPins());
static Buttons buttons(defaultButtonPins());
#if CALIBRATION_ENABLED
static Menu menu(display, buttons, fastController, rls08, robotConfig,
                 &calibrator);
#else
static Menu menu(display, buttons, fastController, rls08, robotConfig);
#endif

static Diagnostics diagnostics(robotConfig);

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void setup() {
    // Load user-confirmed values from NVS (if any). Missing keys keep the
    // compiled-in defaults: the robot boots and runs WITHOUT any prior
    // calibration or saved config — calibration is never a boot dependency.
    PersistentConfig::load(robotConfig);
    fastController.setGains(robotConfig.controller.gains);
    fastController.setIntegralEnabled(robotConfig.controller.integralEnabled);

    // Controller starts DISABLED (motors stopped) — see PathController.
    // The user turns the system ON via the OLED SYSTEM menu.
    fastController.begin();

#if CALIBRATION_ENABLED
    // Calibration starts DISABLED even when compiled in; enable it via
    // the OLED AUTO MODE screen or the serial command "cal on".
#endif

    diagnostics.attach(&fastController, &rls08,
#if CALIBRATION_ENABLED
                        &calibrator
#else
                        nullptr
#endif
    );
    diagnostics.begin();

    // UI: OLED absence is non-fatal (headless mode via serial).
    display.begin();
    buttons.begin();
    menu.begin();

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

    // ---- UI (buttons + OLED; internally rate-limited to ~5 Hz) --------
    menu.update();

    // ---- Diagnostics (serial commands + rate-limited heartbeat) -------
    diagnostics.update(now);
}
