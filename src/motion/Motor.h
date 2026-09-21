/**
 * Motor.h
 * =======
 * Abstraction of a single DC motor behind an H-bridge (L298N style).
 *
 * Contract:
 *   - setSpeed(float) takes a SIGNED normalised command in [-1, +1]:
 *       +1 = full forward, -1 = full reverse, 0 = stop.
 *     WHY normalised: the controller reasons in SI units; PWM scaling is a
 *     hardware boundary concern and lives only here and in MotionConfig.
 *   - stop()  : coast (or brake, per config)
 *   - brake() : active brake (both bridge inputs HIGH on L298N)
 *   - The driver saturates |command| to maxPwm internally.
 *
 * TODO(hardware): the L298N has no current sense output; deadband
 * compensation (minimum PWM that overcomes static friction) is NOT
 * implemented — add a per-motor deadband value here once measured.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "../config/RobotConfig.h"
#include "../config/PinConfig.h"

class Motor {
public:
    struct Config {
        uint8_t pwmPin;    ///< ENA / ENB
        uint8_t in1Pin;    ///< IN1 / IN3
        uint8_t in2Pin;    ///< IN2 / IN4
        uint8_t pwmChannel;///< ESP32 LEDC channel (must be unique per motor)
        bool    invert;    ///< flip sign if the motor is wired backwards
    };

    Motor(const Config& cfg, const MotionConfig& motionCfg);

    /// Configure GPIO + LEDC. Call once in setup().
    void begin();

    /// Signed command in [-1, +1]. Values outside are clamped.
    void setSpeed(float command);

    /// Immediate stop (coast or brake depending on configuration).
    void stop();

    /// Active short-brake (rapid stop).
    void brake();

    /// Last applied normalised command (diagnostics).
    float getLastCommand() const { return lastCommand_; }

private:
    void writeBridge(bool in1, bool in2);
    void writePwm(uint16_t duty);

    Config      cfg_;
    MotionConfig motionCfg_;
    float       lastCommand_;
};

#endif // MOTOR_H
