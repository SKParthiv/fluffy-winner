/**
 * PathController.cpp
 * ==================
 * The fast loop. Every design decision here optimises for:
 *   deterministic timing, no blocking, no Serial, measured dt.
 *
 * Loop statistics are updated on EVERY update() call so the diagnostics
 * can report real min/max/avg periods (task §7) without the controller
 * itself printing anything.
 */

#include "PathController.h"

PathController::PathController(LineSensor& sensor, DifferentialDrive& drive,
                               const RobotConfig& cfg)
    : sensor_(sensor),
      drive_(drive),
      pid_(cfg.controller),
      kinematics_(cfg.geometry.controlPointDistance),
      targetPosition_(cfg.controller.targetPosition),
      forwardVelocity_(cfg.controller.forwardVelocity),
      maxLinearVelocity_(cfg.controller.maxLinearVelocity),
      maxAngularVelocity_(cfg.controller.maxAngularVelocity),
      loopPeriodUs_(cfg.controller.controlLoopPeriodUs),
      behaviour_{0.35f, 0.10f},
      lastRunUs_(0),
      lastLoopPeriodUs_(0),
      minLoopPeriodUs_(kLoopPeriodInit),
      maxLoopPeriodUs_(0),
      periodSumUs_(0),
      loopCount_(0),
      lastError_(0.0f),
      lineLost_(false),
      lastErrorSign_(0.0f),
      lastV_(0.0f), lastOmega_(0.0f) {}

void PathController::begin() {
    sensor_.begin();
    drive_.begin();
    drive_.setVelocityLimits(maxLinearVelocity_, maxAngularVelocity_);
    pid_.reset();
}

bool PathController::shouldRun(uint32_t nowUs) const {
    // Handles micros() wraparound correctly (unsigned arithmetic).
    return (uint32_t)(nowUs - lastRunUs_) >= loopPeriodUs_;
}

void PathController::update(uint32_t nowUs) {
    // ---- 1. Timing bookkeeping (measured dt, never nominal) -----------
    if (lastRunUs_ != 0) {
        lastLoopPeriodUs_ = nowUs - lastRunUs_;
        if (lastLoopPeriodUs_ < minLoopPeriodUs_) minLoopPeriodUs_ = lastLoopPeriodUs_;
        if (lastLoopPeriodUs_ > maxLoopPeriodUs_) maxLoopPeriodUs_ = lastLoopPeriodUs_;
        periodSumUs_ += lastLoopPeriodUs_;
        ++loopCount_;
    }
    const float dt = (lastRunUs_ != 0)
                         ? (float)(nowUs - lastRunUs_) * 1e-6f
                         : (float)loopPeriodUs_ * 1e-6f;
    lastRunUs_ = nowUs;

    // ---- 2. Sensor ------------------------------------------------------
    sensor_.update();
    const LineMeasurement m = sensor_.getMeasurement();

    // ---- 3. Error estimate ----------------------------------------------
    // e > 0 <=> line is RIGHT of the centreline (convention, RobotConfig.h)
    float error = m.position - targetPosition_;
    lastError_ = error;
    lineLost_ = !m.valid;

    if (m.valid) {
        if (error != 0.0f) lastErrorSign_ = (error > 0.0f) ? 1.0f : -1.0f;
    }

    // ---- 4. Control law --------------------------------------------------
    if (m.valid) {
        // PID output = desired LATERAL point velocity [m/s].
        // Output limit: keep the point velocity inside a sane band; the
        // twist-level saturation happens in DifferentialDrive as well.
        const float lateralLimit = 0.5f * maxLinearVelocity_;
        const float uLateral = -pid_.compute(error, dt, lateralLimit);

        // Drive the control point forward at cruise speed plus the
        // lateral correction (body-frame point velocity).
        PointVelocityBody u;
        u.uForward  = forwardVelocity_;
        u.uLateral  = uLateral;

        const Twist2D twist = kinematics_.bodyFrameTwist(u);
        drive_.setTwist(twist.v, twist.omega);
        lastV_ = twist.v;
        lastOmega_ = twist.omega;
    } else {
        // ---- Line lost: bounded search turn toward the last seen side.
        // The sensor already extrapolates position to ±1 on loss; the
        // PID still runs on that synthetic error, but we additionally
        // cap the turn rate and slow down, so the recovery is predictable.
        const float searchOmega =
            (lastErrorSign_ >= 0.0f ? -1.0f : 1.0f) *   // turn toward line
            behaviour_.searchTurnFraction * maxAngularVelocity_;
        drive_.setTwist(behaviour_.searchForwardSpeed, searchOmega);
        lastV_ = behaviour_.searchForwardSpeed;
        lastOmega_ = searchOmega;
    }
}

void PathController::setGains(const PIDGains& gains) {
    // Only interaction point with the calibrator: a cheap, non-blocking
    // write. The fast loop never waits for calibration.
    pid_.setGains(gains);
}

const PIDGains& PathController::getGains() const { return pid_.getGains(); }

void PathController::setIntegralEnabled(bool enabled) {
    pid_.setIntegralEnabled(enabled);
}

uint32_t PathController::getAvgLoopPeriodUs() const {
    return (loopCount_ > 0)
               ? (uint32_t)(periodSumUs_ / loopCount_)
               : 0;
}
