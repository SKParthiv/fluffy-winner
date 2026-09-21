/**
 * RLS08LineSensor.h
 * ==================
 * Driver for the Smartflex RLS08 8-channel IR line sensor.
 *
 * WHAT IS KNOWN (from community ESP32/AVR projects using this sensor):
 *   - 8 IR reflectance channels in a linear array.
 *   - One DIGITAL output per channel; HIGH = line (dark surface) detected
 *     under that channel on the commonly used variant.
 *
 * WHAT IS NOT KNOWN (deliberately not invented — see README):
 *   - exact channel pitch [mm]  -> configurable, TODO(hardware): measure
 *   - whether YOUR board inverts the polarity -> configurable
 *   - left/right channel order on your wiring -> configurable
 *
 * Position estimation (weighted average over active channels):
 *
 *        position = sum_i( w_i * s_i ) / sum_i( s_i ),   s_i in {0, 1}
 *
 * with weights w_i = (i - (N-1)/2) / ((N-1)/2) in [-1, +1], so the raw
 * position is already normalised. When the line sits under an edge
 * channel we extrapolate the last known direction but lower confidence.
 */

#ifndef RLS08_LINE_SENSOR_H
#define RLS08_LINE_SENSOR_H

#include "LineSensor.h"
#include "../config/PinConfig.h"

class RLS08LineSensor : public LineSensor {
public:
    static const int NUM_CHANNELS = 8;

    struct Config {
        RLS08PinConfig pins;
        bool  lineIsHigh = true;    ///< TODO(hardware): verify polarity.
        bool  reverseOrder = false;///< TODO(hardware): set if channels are mirrored.
    };

    explicit RLS08LineSensor(const Config& cfg);

    bool begin() override;
    bool update() override;
    LineMeasurement getMeasurement() const override;

    /// Raw digital channel states of the last update (for diagnostics).
    void readRaw(bool out[NUM_CHANNELS]) const;

    /// Number of channels currently detecting the line.
    int activeChannelCount() const { return activeCount_; }

private:
    float channelWeight(int index) const;
    float computePosition(bool anyActive) const;

    Config  cfg_;
    bool    raw_[NUM_CHANNELS];
    int     activeCount_;
    /// Sign of the last valid reading; used to extrapolate when the line
    /// is only under an edge channel (partial confidence).
    float   lastDirectionSign_;
    LineMeasurement measurement_;
};

#endif // RLS08_LINE_SENSOR_H
