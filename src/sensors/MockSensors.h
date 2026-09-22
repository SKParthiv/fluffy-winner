/**
 * MockSensors.h
 * =============
 * Simulated LineSensor and IMUInterface implementations.
 *
 * WHY: hardware specs are partly unknown and the task requires the codebase
 * to compile and be testable without hardware (§22, §24). MockLineSensor
 * replays a scripted position sequence; MockIMU synthesises a sinusoidal
 * yaw rate so oscillation metrics can be exercised offline.
 *
 * These are also the seeds of a future closed-loop simulation: feed
 * MockLineSensor from a robot model instead of a script.
 */

#ifndef MOCK_SENSORS_H
#define MOCK_SENSORS_H

#include <Arduino.h>
#include "LineSensor.h"
#include "IMUInterface.h"
#include <stdint.h>

/**
 * Replays a caller-supplied sequence of normalised positions.
 * update() advances the sequence every `stepPeriodUs` of virtual time.
 */
class MockLineSensor : public LineSensor {
public:
    MockLineSensor(const float* positions, int count, uint32_t stepPeriodUs)
        : positions_(positions), count_(count), stepPeriodUs_(stepPeriodUs),
          index_(0), lastAdvanceUs_(0) {}

    bool begin() override { index_ = 0; lastAdvanceUs_ = 0; return true; }

    bool update() override {
        const uint32_t now = micros();
        bool advanced = false;
        if (count_ > 0 &&
            (uint32_t)(now - lastAdvanceUs_) >= stepPeriodUs_) {
            index_ = (index_ + 1) % count_;
            lastAdvanceUs_ = now;
            advanced = true;
        }
        if (count_ > 0) {
            measurement_.position = positions_[index_];
            measurement_.valid = true;
            measurement_.confidence = 1.0f;
            measurement_.timestampUs = now;
        }
        return advanced;
    }

    LineMeasurement getMeasurement() const override { return measurement_; }

    /// Test hook: force an invalid (line-lost) sample.
    void invalidate() { measurement_.valid = false; measurement_.confidence = 0.0f; }

private:
    const float*   positions_;
    int           count_;
    uint32_t      stepPeriodUs_;
    int           index_;
    uint32_t      lastAdvanceUs_;
    LineMeasurement measurement_;
};

/**
 * Synthesises omega_z(t) = amplitude * sin(2*pi*f*t) [rad/s], i.e. a pure
 * oscillation, plus an optional constant yaw drift. Yaw is reported as
 * INVALID (a mock gyro cannot provide absolute heading — by design, to
 * make sure consumers check yawValid()).
 */
class MockIMU : public IMUInterface {
public:
    MockIMU(float amplitudeRadS, float frequencyHz)
        : amplitude_(amplitudeRadS), frequencyHz_(frequencyHz),
          startedUs_(0), fresh_(false), omegaZ_(0.0f) {}

    bool begin() override { startedUs_ = micros(); return true; }

    bool update() override {
        const uint32_t now = micros();
        const float t = (now - startedUs_) * 1e-6f;
        const float twoPi = 6.28318530718f;
        omegaZ_ = amplitude_ * sinf(twoPi * frequencyHz_ * t);
        fresh_ = true;
        return true;
    }

    bool hasNewData() const override { return fresh_; }
    float getAngularVelocityZ() const override { return omegaZ_; }
    float getYaw() const override { return 0.0f; }
    bool yawValid() const override { return false; }  // gyro-only mock

private:
    float    amplitude_;
    float    frequencyHz_;
    uint32_t startedUs_;
    bool     fresh_;
    float    omegaZ_;
};

#endif // MOCK_SENSORS_H
