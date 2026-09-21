/**
 * PIDController.cpp
 * =================
 * Continuous-form PID discretised with the measured dt:
 *
 *   u(t) = Kp*e(t) + Ki*∫e dt + Kd*de/dt
 *
 * Discrete implementation:
 *   P = Kp * e_k
 *   I += Ki * e_k * dt_k            (then clamped to ±integralLimit)
 *   D  = Kd * d_filtered_k, with
 *        d_raw_k     = (e_k - e_{k-1}) / dt_k
 *        d_filtered_k = α*d_raw_k + (1-α)*d_filtered_{k-1}
 *
 * α (derivativeAlpha_) trades noise rejection against phase lag; α = 1
 * degenerates to the raw derivative.
 */

#include "PIDController.h"

PIDController::PIDController(const ControllerConfig& cfg)
    : gains_(cfg.gains),
      integralEnabled_(cfg.integralEnabled),
      errorLimit_(cfg.errorLimit),
      integralLimit_(cfg.integralLimit),
      derivativeAlpha_(cfg.derivativeFilterAlpha),
      integral_(0.0f),
      lastError_(0.0f),
      filteredDerivative_(0.0f),
      firstStep_(true),
      pTerm_(0.0f), iTerm_(0.0f), dTerm_(0.0f),
      lastOutput_(0.0f) {}

void PIDController::reset() {
    integral_ = 0.0f;
    lastError_ = 0.0f;
    filteredDerivative_ = 0.0f;
    firstStep_ = true;
    pTerm_ = iTerm_ = dTerm_ = 0.0f;
    lastOutput_ = 0.0f;
}

void PIDController::setGains(const PIDGains& gains) {
    gains_ = gains;
}

float PIDController::compute(float error, float dtSeconds, float outputLimit) {
    // A non-positive or absurd dt means this step carries no information.
    // Return the previous output instead of dividing by ~0 in the D term.
    if (dtSeconds <= 0.0f || dtSeconds > 1.0f) {
        return lastOutput_;
    }

    // Clamp the raw error: a single glitched sensor frame must not slam
    // the proportional term.
    if (error >  errorLimit_) error =  errorLimit_;
    if (error < -errorLimit_) error = -errorLimit_;

    // --- Proportional -------------------------------------------------
    pTerm_ = gains_.kp * error;

    // --- Integral (trapezoidal would be marginally better, but the loop
    //     runs at 200 Hz; rectangular is standard and simpler to reason
    //     about for anti-windup) -----------------------------------------
    integral_ += gains_.ki * error * dtSeconds;
    if (integral_ >  integralLimit_) integral_ =  integralLimit_;
    if (integral_ < -integralLimit_) integral_ = -integralLimit_;
    iTerm_ = integralEnabled_ ? integral_ : 0.0f;

    // --- Derivative (filtered, measured dt) ------------------------------
    if (firstStep_) {
        filteredDerivative_ = 0.0f;   // no valid derivative on step 1
        firstStep_ = false;
    } else {
        const float dRaw = (error - lastError_) / dtSeconds;
        const float a = derivativeAlpha_;
        filteredDerivative_ = a * dRaw + (1.0f - a) * filteredDerivative_;
    }
    dTerm_ = gains_.kd * filteredDerivative_;

    lastError_ = error;

    // --- Sum and clamp ---------------------------------------------------
    float u = pTerm_ + iTerm_ + dTerm_;
    if (u >  outputLimit) u =  outputLimit;
    if (u < -outputLimit) u = -outputLimit;
    lastOutput_ = u;
    return u;
}
