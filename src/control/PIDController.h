/**
 * PIDController.h
 * ===============
 * A small, hardware-free, unit-testable PID on the line error.
 *
 * Design decisions (WHY, per task §6):
 *   - dt is ALWAYS the measured elapsed time, never a nominal period.
 *   - The derivative is low-pass filtered (first-order) because the raw
 *     difference of quantised sensor positions is noisy.
 *   - Integral anti-windup by symmetric clamping of the integral state.
 *   - The integral can be compiled out of the output while the state is
 *     still tracked (integralEnabled flag).
 *   - Error is clamped before use (errorLimit).
 *   - Output is clamped to outputLimit.
 *
 * This class knows NOTHING about motors, sensors or kinematics.
 * The PathController decides what the output means (here: a lateral
 * point-velocity command in [m/s], see PathController.h).
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "../config/RobotConfig.h"

class PIDController {
public:
    explicit PIDController(const ControllerConfig& cfg);

    /// Reset dynamic state (integral, derivative filter, timing).
    void reset();

    /// Change gains at runtime (used by the calibrator and diagnostics).
    void setGains(const PIDGains& gains);
    const PIDGains& getGains() const { return gains_; }

    void setIntegralEnabled(bool enabled) { integralEnabled_ = enabled; }
    bool isIntegralEnabled() const { return integralEnabled_; }

    /**
     * Compute one PID step.
     *   error     : setpoint-relative error (already signed per convention).
     *   dtSeconds : MEASURED elapsed time since the previous step [s].
     *               If <= 0 the step is skipped (returns last output) —
     *               a zero/negative dt must never produce a huge D kick.
     *   outputLimit : symmetric clamp on the returned value.
     * Returns the PID output in the same units as error * gain.
     */
    float compute(float error, float dtSeconds, float outputLimit);

    /// Read-only state for diagnostics/calibration.
    float getProportionalTerm() const { return pTerm_; }
    float getIntegralTerm() const    { return iTerm_; }
    float getDerivativeTerm() const  { return dTerm_; }
    float getIntegralState() const   { return integral_; }
    float getLastError() const       { return lastError_; }

private:
    // Configuration (copied so runtime changes are explicit via setGains)
    PIDGains gains_;
    bool     integralEnabled_;
    float    errorLimit_;
    float    integralLimit_;
    float    derivativeAlpha_;

    // Dynamic state
    float   integral_;
    float   lastError_;
    float   filteredDerivative_;  ///< low-pass state, in error units/s
    bool    firstStep_;
    float   pTerm_, iTerm_, dTerm_;
    float   lastOutput_;
};

#endif // PID_CONTROLLER_H
