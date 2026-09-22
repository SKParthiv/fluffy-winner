/**
 * DifferentialDrive.h
 * ====================
 * Mixing stage: desired body twist (v, omega) -> wheel speeds -> motor
 * commands, plus the inverse (wheel speeds -> twist) for future odometry.
 *
 * MATH (task §8):
 *   For a differential-drive robot with wheel track b (distance between
 *   the tyre contact points) and equal-magnitude wheel speeds v_L, v_R:
 *
 *     v_L = v - (b/2) * omega
 *     v_R = v + (b/2) * omega
 *
 *   (omega > 0 = CCW = left wheel slower than right wheel.)
 *
 *   Inverse (for odometry):
 *     v     = (v_R + v_L) / 2
 *     omega = (v_R - v_L) / b
 *
 * SIGN CONVENTIONS:
 *   - v   [m/s], + = forward
 *   - omega [rad/s], + = CCW (nose swings LEFT)
 *   - positive wheel speed = forward rotation of that wheel
 *   - Left/right assignment is configurable (invert flags live in Motor).
 *
 * Wheel-speed-to-PWM conversion:
 *   The N20 + L298N chain is open-loop (no encoders in this milestone),
 *   so wheel speed [m/s] is mapped to a normalised motor command by
 *   dividing by a configurable maximum wheel speed `maxWheelSpeed`
 *   (an estimate of the achievable speed at full duty with the ~6-7 V
 *   supply; TODO(hardware): measure on the real robot and update).
 *   The result saturates naturally in Motor::setSpeed().
 */

#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include "Motor.h"
#include "../control/PointKinematics.h"  ///< Twist2D
#include "../config/RobotConfig.h"

struct WheelSpeeds {
    float left;   ///< [m/s], + = forward
    float right;  ///< [m/s], + = forward
};

class DifferentialDrive {
public:
    DifferentialDrive(Motor& leftMotor, Motor& rightMotor,
                      const GeometryConfig& geom, const MotionConfig& motion);

    /// Configure motors (call in setup()).
    void begin();

    /**
     * Command a body twist. Saturates v to maxLinearVelocity and omega to
     * maxAngularVelocity BEFORE mixing, so the wheel commands stay inside
     * the safety envelope even if the controller misbehaves.
     */
    void setTwist(float v, float omega);

    /// Same mixing math without touching hardware (unit-testable).
    WheelSpeeds mixWheelSpeeds(float v, float omega) const;

    /// Pure mixing math, no saturation, no hardware: v_L = v - b/2*omega,
    /// v_R = v + b/2*omega. Exposed for unit tests and odometry.
    static WheelSpeeds mixWheelSpeedsPure(float v, float omega, float wheelTrack);

    /// Update the velocity saturation limits (called by the integrator).
    void setVelocityLimits(float maxLinear, float maxAngular);

    /// Inverse mixing for odometry/tests: wheels -> twist.
    /// theta-dot convention identical to setTwist.
    static Twist2D wheelsToTwist(const WheelSpeeds& w, float wheelTrack);

    /// Emergency stop: coast or brake both motors.
    void stop();

    float getWheelTrack() const { return wheelTrack_; }

    /// Estimate of full-duty wheel speed [m/s]. TODO(hardware): measure.
    void setMaxWheelSpeed(float maxWheelSpeed) { maxWheelSpeed_ = maxWheelSpeed; }

    /// Last commanded twist (diagnostics).
    void getLastTwist(float& v, float& omega) const { v = lastV_; omega = lastOmega_; }

private:
    void writeMotors(const WheelSpeeds& w);

    Motor&  left_;
    Motor&  right_;
    float   wheelTrack_;
    float   maxLinearVelocity_;
    float   maxAngularVelocity_;
    float   maxWheelSpeed_;  ///< [m/s] full-duty estimate
    float   lastV_, lastOmega_;
};

#endif // DIFFERENTIAL_DRIVE_H
