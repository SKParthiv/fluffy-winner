/**
 * RobotConfig.h
 * =============
 * Central configuration for every tunable/non-tunable parameter of the robot.
 *
 * WHY: the task explicitly forbids scattering constants through the code.
 * All geometry, controller gains, limits and timing live here so that
 * parameters can be changed without touching controller implementation code.
 *
 * UNITS (SI internally, see README "Coordinate frames & units"):
 *   - lengths  : metres   [m]
 *   - velocity : m/s     [m/s]
 *   - rotation : radians [rad], angular velocity [rad/s]
 *   - time     : seconds  [s] (microseconds [us] only at the timing boundary)
 *
 * Line-error sign convention (documented once, used everywhere):
 *   e > 0  <=>  the line centre is to the RIGHT of the robot centreline
 *               (as seen when looking along +x_body, the forward direction).
 *   Therefore a positive controller output must command a positive yaw rate
 *   omega (counter-clockwise, right-hand rule around +z up) to steer the
 *   robot's nose toward the line.
 *
 * HARDWARE UNCERTAINTY (TODOs, do not trust blindly):
 *   - wheel track          : measure on the physical robot (placeholder below)
 *   - gearbox ratio        : not needed for open-loop PWM mixing, but needed
 *                           later for odometry (TODO)
 *   - actual motor voltage : ~6-7 V supply stated, L298N drops ~1.5-2.5 V
 *   - IMU model            : unknown -> abstract interface only
 *   - RLS08 pin mapping    : see PinConfig.h, verify against your wiring
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Feature flags
// ---------------------------------------------------------------------------

/**
 * CALIBRATION_ENABLED
 * The single switch that couples the optional slow calibration supervisor
 * to the fast control loop. When false the calibration module is never
 * invoked (see main.ino) and the robot runs purely on the baseline gains.
 * The fast controller does NOT depend on this flag in any way.
 */
#ifndef CALIBRATION_ENABLED
#define CALIBRATION_ENABLED false
#endif

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

struct GeometryConfig {
    /// Wheel radius [m]. N20 robots typically ~0.02 m (task statement).
    float wheelRadius = 0.02f;

    /// Distance between the two wheel contact points [m].
    /// TODO(hardware): MEASURE on the physical chassis; placeholder value.
    float wheelTrack = 0.15f;

    /**
     * Virtual control-point distance l [m], measured forward from the wheel
     * axle midpoint along the robot heading. The PathController commands the
     * velocity of this point (see PointKinematics.h). Larger l => smoother,
     * less oscillatory but slower-responding tracking.
     */
    float controlPointDistance = 0.08f;
};

// ---------------------------------------------------------------------------
// PID gains (baseline; the calibrator may only propose values inside the
// bounds in CalibrationLimits below)
// ---------------------------------------------------------------------------

struct PIDGains {
    float kp;
    float ki;
    float kd;

    PIDGains() : kp(0), ki(0), kd(0) {}
    PIDGains(float p, float i, float d) : kp(p), ki(i), kd(d) {}

    bool operator==(const PIDGains& o) const {
        return kp == o.kp && ki == o.ki && kd == o.kd;
    }
    bool operator!=(const PIDGains& o) const { return !(*this == o); }
};

struct ControllerConfig {
    /// Baseline gains. Units: [1/s] for kp/ki/kd acting on a normalised
    /// line error in [-1, 1] producing a point velocity in [m/s].
    /// These are safe starting values; tune on hardware.
    PIDGains gains = PIDGains(2.0f, 0.0f, 0.15f);

    /// If true the integral term is computed but NOT added to the output.
    /// WHY: on a line follower the integral mostly compensates systematic
    /// steering bias; leaving it off by default avoids windup surprises.
    bool integralEnabled = false;

    /// Target line position (normalised, 0 = centred). Non-zero when the
    /// sensor array is mounted off-centre on the chassis.
    float targetPosition = 0.0f;

    /// Forward (cruise) velocity of the robot [m/s]. The virtual point is
    /// driven forward at this speed plus the lateral correction.
    float forwardVelocity = 0.35f;

    /// Hard output limits.
    float maxLinearVelocity  = 0.8f;   ///< [m/s]
    float maxAngularVelocity = 6.0f;   ///< [rad/s]

    /// Symmetric clamp on the line error fed to the PID [normalised units].
    /// WHY: a single wild sensor frame must not produce a huge P kick.
    float errorLimit = 1.0f;

    /// Integral anti-windup: |integral state| clamped to this value [m/s].
    float integralLimit = 0.3f;

    /**
     * Derivative low-pass coefficient alpha in (0, 1].
     *   d_filtered = alpha * d_raw + (1 - alpha) * d_filtered_prev
     * Smaller alpha = heavier filtering (less noise, more lag).
     * The derivative always uses the MEASURED dt, never a nominal period.
     */
    float derivativeFilterAlpha = 0.4f;

    /// Fast control-loop period [microseconds]. 5 ms => 200 Hz.
    uint32_t controlLoopPeriodUs = 5000;
};

// ---------------------------------------------------------------------------
// Motion / motor limits
// ---------------------------------------------------------------------------

struct MotionConfig {
    /// PWM resolution bits used by the ESP32 LEDC driver (8 => 0..255).
    uint8_t pwmBits = 8;

    /// PWM frequency [Hz]. L298N works fine from a few hundred Hz to ~20 kHz.
    /// TODO(hardware): above ~15 kHz the L298N loses efficiency; verify.
    uint32_t pwmFrequencyHz = 5000;

    /// Maximum |PWM| duty applied to any motor, in the 0..(2^pwmBits - 1)
    /// scale. Safety cap independent of the velocity limits.
    uint16_t maxPwm = 255;

    /// If true, setting drive(0) actively brakes (both H-bridge inputs
    /// HIGH) instead of coasting (both LOW). L298N supports both.
    bool brakeOnZero = false;
};

// ---------------------------------------------------------------------------
// Calibration (only consumed by src/calibration, ignored when disabled)
// ---------------------------------------------------------------------------

struct CalibrationLimits {
    float kpMin = 0.1f,  kpMax = 12.0f;
    float kiMin = 0.0f,  kiMax = 4.0f;
    float kdMin = 0.0f,  kdMax = 2.0f;
};

struct CalibrationConfig {
    /// Supervisor update period [microseconds]. ~1 s per the task statement.
    uint32_t updatePeriodUs = 1000000;

    /// Evaluation window T [s] over which metrics are accumulated.
    float evaluationWindowS = 1.0f;

    /// Objective weights: J = w_e*E_tracking + w_o*E_osc + w_s*E_instability
    /// WHY the oscillation term alone is NOT minimised: the trivial optimum
    /// of "zero oscillation" is zero gain => no steering => the robot leaves
    /// the line. Tracking quality must dominate the objective.
    float weightTracking    = 1.0f;
    float weightOscillation  = 0.5f;
    float weightInstability = 2.0f;

    /// Coordinate-descent step sizes (relative perturbations).
    float kpStep = 0.25f;   ///< multiplied onto kp
    float kiStep = 0.25f;
    float kdStep = 0.25f;

    /// Candidate accepted only if J improves by at least this fraction of
    /// the current best (guards against noise-driven random walk).
    float minImprovementFraction = 0.01f;

    /// Hard cap on calibration wall-time [s]; on expiry the supervisor
    /// reverts to lastKnownGoodGains and disables itself.
    float timeoutS = 120.0f;

    /// If the line is lost for longer than this [s] during an evaluation,
    /// the window is aborted and the candidate is rejected.
    float lineLossAbortS = 0.3f;

    CalibrationLimits limits;
};

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

struct DiagnosticsConfig {
    uint32_t serialBaud       = 115200;
    /// Rate limit for periodic status prints [microseconds between prints].
    uint32_t printPeriodUs    = 1000000;
};

// ---------------------------------------------------------------------------
// Aggregate
// ---------------------------------------------------------------------------

struct RobotConfig {
    GeometryConfig    geometry;
    ControllerConfig  controller;
    MotionConfig      motion;
    CalibrationConfig calibration;
    DiagnosticsConfig diagnostics;
};

#endif // ROBOT_CONFIG_H
