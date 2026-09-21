/**
 * DifferentialDrive.cpp — see header for the mixing equations.
 */

#include "DifferentialDrive.h"
#include <math.h>

DifferentialDrive::DifferentialDrive(Motor& leftMotor, Motor& rightMotor,
                                     const GeometryConfig& geom,
                                     const MotionConfig& motion)
    : left_(leftMotor),
      right_(rightMotor),
      wheelTrack_(geom.wheelTrack),
      maxLinearVelocity_(0.8f),
      maxAngularVelocity_(6.0f),
      // TODO(hardware): measure real full-duty wheel speed at 6-7 V.
      // Placeholder: N20 ~300 RPM geared, r=0.02 m  =>  v = 0.02*2*pi*5 ≈ 0.63 m/s
      maxWheelSpeed_(0.6f),
      lastV_(0.0f), lastOmega_(0.0f) {
    // Limits are taken from the controller config normally; they are set
    // explicitly via setVelocityLimits by the integrator (FastController).
}

void DifferentialDrive::setVelocityLimits(float maxLinear, float maxAngular) {
    maxLinearVelocity_  = (maxLinear  > 0.0f) ? maxLinear  : maxLinearVelocity_;
    maxAngularVelocity_ = (maxAngular > 0.0f) ? maxAngular : maxAngularVelocity_;
}

void DifferentialDrive::begin() {
    left_.begin();
    right_.begin();
}

WheelSpeeds DifferentialDrive::mixWheelSpeeds(float v, float omega) const {
    // Safety saturation BEFORE mixing: guarantees the wheel commands stay
    // inside the configured envelope regardless of controller behaviour.
    if (v >  maxLinearVelocity_)  v =  maxLinearVelocity_;
    if (v < -maxLinearVelocity_)  v = -maxLinearVelocity_;
    if (omega >  maxAngularVelocity_)  omega =  maxAngularVelocity_;
    if (omega < -maxAngularVelocity_)  omega = -maxAngularVelocity_;

    WheelSpeeds w;
    const float halfTrack = 0.5f * wheelTrack_;
    w.left  = v - halfTrack * omega;
    w.right = v + halfTrack * omega;
    return w;
}

void DifferentialDrive::setTwist(float v, float omega) {
    const WheelSpeeds w = mixWheelSpeeds(v, omega);
    writeMotors(w);
    lastV_ = v;
    lastOmega_ = omega;
}

WheelSpeeds DifferentialDrive::mixWheelSpeedsPure(float v, float omega,
                                                   float wheelTrack) {
    WheelSpeeds w;
    const float halfTrack = 0.5f * wheelTrack;
    w.left  = v - halfTrack * omega;
    w.right = v + halfTrack * omega;
    return w;
}

Twist2D DifferentialDrive::wheelsToTwist(const WheelSpeeds& w, float wheelTrack) {
    Twist2D t;
    t.v     = 0.5f * (w.right + w.left);
    t.omega = (wheelTrack > 0.0f) ? ((w.right - w.left) / wheelTrack) : 0.0f;
    return t;
}

void DifferentialDrive::stop() {
    left_.stop();
    right_.stop();
    lastV_ = 0.0f;
    lastOmega_ = 0.0f;
}

void DifferentialDrive::writeMotors(const WheelSpeeds& w) {
    // Convert wheel speed [m/s] to a normalised motor command [-1, 1].
    // The Motor class applies the final PWM saturation (maxPwm).
    if (maxWheelSpeed_ <= 0.0f) {
        left_.setSpeed(0.0f);
        right_.setSpeed(0.0f);
        return;
    }
    left_.setSpeed(w.left / maxWheelSpeed_);
    right_.setSpeed(w.right / maxWheelSpeed_);
}
