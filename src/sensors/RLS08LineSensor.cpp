/**
 * RLS08LineSensor.cpp
 * ====================
 * Implementation notes:
 *  - update() does 8 digitalRead()s: cheap and non-blocking, safe at 200 Hz.
 *  - No protocol, register map or analog thresholding is invented here.
 *    If your RLS08 variant exposes analog outputs, replace this file only;
 *    the LineSensor interface stays identical (that is the point of the
 *    abstraction).
 */

#include "RLS08LineSensor.h"
#include <Arduino.h>

RLS08LineSensor::RLS08LineSensor(const Config& cfg)
    : cfg_(cfg), activeCount_(0), lastDirectionSign_(0.0f) {
    for (int i = 0; i < NUM_CHANNELS; ++i) raw_[i] = false;
}

bool RLS08LineSensor::begin() {
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        pinMode(cfg_.pins.channelPins[i], INPUT);
    }
    return true;
}

bool RLS08LineSensor::update() {
    // Channel 0 is defined as the LEFTMOST channel of the array.
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        int pinIndex = cfg_.reverseOrder ? (NUM_CHANNELS - 1 - i) : i;
        bool level = (digitalRead(cfg_.pins.channelPins[pinIndex]) == HIGH);
        raw_[i] = (level == cfg_.lineIsHigh);
    }

    activeCount_ = 0;
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        if (raw_[i]) ++activeCount_;
    }

    const uint32_t now = micros();
    const bool lineDetected = (activeCount_ > 0);

    if (lineDetected) {
        measurement_.position = computePosition(true);
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

float RLS08LineSensor::channelWeight(int index) const {
    // w_i in [-1, +1]: channel 0 (left) -> -1, channel 7 (right) -> +1.
    const float half = (NUM_CHANNELS - 1) / 2.0f;
    return (static_cast<float>(index) - half) / half;
}

float RLS08LineSensor::computePosition(bool anyActive) const {
    (void)anyActive;  // caller guarantees activeCount_ > 0
    float sum = 0.0f;
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        if (raw_[i]) sum += channelWeight(i);
    }
    return sum / static_cast<float>(activeCount_);
}
