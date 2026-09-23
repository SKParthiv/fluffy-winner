/**
 * RLS08LineSensor.cpp
 * ====================
 * Implementation notes:
 *  - update() polls only ASSIGNED pins (Sensors 1-6 confirmed so far).
 *    PIN_UNASSIGNED channels are reported as "no line" and are never
 *    touched — no invented hardware.
 *  - Index 0 == Sensor 1 == RIGHTMOST channel (CONFIRMED orientation),
 *    so the position sign convention stays: positive = line to the right.
 *  - No protocol, register map or analog thresholding is invented here.
 *    If your RLS08 variant exposes analog outputs, extend this file only;
 *    the LineSensor interface stays identical (that is the point of the
 *    abstraction).
 */

#include "RLS08LineSensor.h"
#include <Arduino.h>

RLS08LineSensor::RLS08LineSensor(const Config& cfg)
    : cfg_(cfg), activeCount_(0), assignedCount_(0),
      lastDirectionSign_(0.0f) {
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        raw_[i] = false;
        levels_[i] = false;
    }
}

bool RLS08LineSensor::begin() {
    assignedCount_ = 0;
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        // Failsafe: never pinMode/digitalRead an unassigned (TODO) pin.
        if (pinAssigned(cfg_.pins.sensorPins[i])) {
            pinMode(cfg_.pins.sensorPins[i], INPUT);
            ++assignedCount_;
        }
    }
    return true;  // digital inputs have no meaningful init failure mode;
                  // electrical validity is verified with test_sensor.ino.
}

bool RLS08LineSensor::update() {
    activeCount_ = 0;

    // Index 0 == Sensor 1 == RIGHTMOST. reverseOrder mirrors the array for
    // the (unconfirmed) case that the physical wiring turns out mirrored.
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        const int sensorIndex = cfg_.reverseOrder ? (NUM_CHANNELS - 1 - i) : i;
        const uint8_t pin = cfg_.pins.sensorPins[sensorIndex];
        if (!pinAssigned(pin)) {
            levels_[i] = false;
            raw_[i] = false;
            continue;
        }
        const bool level = (digitalRead(pin) == HIGH);
        levels_[i] = level;
        raw_[i] = (level == cfg_.lineIsHigh);
        if (raw_[i]) ++activeCount_;
    }

    const uint32_t now = micros();
    const bool lineDetected = (activeCount_ > 0);

    if (lineDetected) {
        measurement_.position = computePosition();
        measurement_.valid = true;

        // Confidence model:
        //  - a single active channel is a perfectly plausible line reading
        //    but gives no averaging; treat it slightly less confident.
        //  - all 8 active usually means a junction / 90-degree crossing:
        //    position is meaningless there.
        if (activeCount_ == NUM_CHANNELS) {
            measurement_.confidence = 0.0f;
            measurement_.valid = false;  // junction: do not steer on this
        } else {
            measurement_.confidence =
                (activeCount_ == 1) ? 0.6f : 0.9f;
        }

        if (measurement_.valid && measurement_.position != 0.0f) {
            lastDirectionSign_ = (measurement_.position > 0.0f) ? 1.0f : -1.0f;
        }
    } else {
        // Line completely lost: keep the last position (the PathController
        // decides what to do with a stale/invalid reading) and mark invalid.
        measurement_.valid = false;
        measurement_.confidence = 0.0f;
        // Extrapolate outward so a "search" behaviour steers toward the side
        // the line was last seen on. This is a *sensor-level hint*, the
        // controller remains free to ignore it.
        measurement_.position = (lastDirectionSign_ != 0.0f)
                                    ? lastDirectionSign_ * 1.0f
                                    : measurement_.position;
    }

    measurement_.timestampUs = now;
    return true;  // a fresh digital poll always yields a new sample
}

LineMeasurement RLS08LineSensor::getMeasurement() const {
    return measurement_;
}

void RLS08LineSensor::readRaw(bool out[NUM_CHANNELS]) const {
    for (int i = 0; i < NUM_CHANNELS; ++i) out[i] = raw_[i];
}

void RLS08LineSensor::readLevels(bool out[NUM_CHANNELS]) const {
    for (int i = 0; i < NUM_CHANNELS; ++i) out[i] = levels_[i];
}

float RLS08LineSensor::channelWeight(int index) const {
    // w_i in [-1, +1]: Sensor 1 (index 0, RIGHTMOST) -> +1,
    // Sensor 8 (index 7, leftmost) -> -1. Positive = right, matching the
    // error convention in RobotConfig.h.
    const float half = (NUM_CHANNELS - 1) / 2.0f;
    return (half - static_cast<float>(index)) / half;
}

float RLS08LineSensor::computePosition() const {
    float sum = 0.0f;
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        if (raw_[i]) sum += channelWeight(i);
    }
    return sum / static_cast<float>(activeCount_);
}
