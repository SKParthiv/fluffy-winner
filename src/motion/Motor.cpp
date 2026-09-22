/**
 * Motor.cpp — L298N-backed motor driver for ESP32 (Arduino core).
 *
 * ESP32 LEDC notes:
 *   - ledcAttach(pin, freq, resolution) is the Arduino-ESP32 3.x API.
 *   - On older 2.x cores use ledcSetup/ledcAttachPin/ledcWrite instead.
 *   TODO(hardware): confirm your arduino-esp32 core version; a small
 *   compatibility shim may be required (kept out of the controller!).
 */

#include "Motor.h"
#include <Arduino.h>
#include <math.h>

Motor::Motor(const Config& cfg, const MotionConfig& motionCfg)
    : cfg_(cfg), motionCfg_(motionCfg), lastCommand_(0.0f) {}

void Motor::begin() {
    pinMode(cfg_.in1Pin, OUTPUT);
    pinMode(cfg_.in2Pin, OUTPUT);
    writeBridge(false, false);
    // ESP32 Arduino core 3.x:
    ledcAttach(cfg_.pwmPin, motionCfg_.pwmFrequencyHz, motionCfg_.pwmBits);
    writePwm(0);
}

void Motor::setSpeed(float command) {
    // Clamp to the normalised range; the hardware safety cap (maxPwm) is
    // applied at the duty-conversion step below.
    if (command >  1.0f) command =  1.0f;
    if (command < -1.0f) command = -1.0f;
    if (cfg_.invert) command = -command;

    lastCommand_ = command;

    if (command > 0.0f) {
        writeBridge(true, false);           // forward
    } else if (command < 0.0f) {
        writeBridge(false, true);           // reverse
    } else {
        if (motionCfg_.brakeOnZero) {
            writeBridge(true, true);        // active brake
        } else {
            writeBridge(false, false);     // coast
        }
    }

    writePwm((uint16_t)(fabsf(command) * motionCfg_.maxPwm));
}

void Motor::stop() {
    setSpeed(0.0f);
}

void Motor::brake() {
    writeBridge(true, true);
    writePwm(motionCfg_.maxPwm);
    lastCommand_ = 0.0f;
}

void Motor::writeBridge(bool in1, bool in2) {
    digitalWrite(cfg_.in1Pin, in1 ? HIGH : LOW);
    digitalWrite(cfg_.in2Pin, in2 ? HIGH : LOW);
}

void Motor::writePwm(uint16_t duty) {
    if (duty > motionCfg_.maxPwm) duty = motionCfg_.maxPwm;
    ledcWrite(cfg_.pwmPin, duty);
}
