/**
 * RLS08LineSensor.h
 * ==================
 * Driver for the Smartflex RLS08 8-channel IR line sensor.
 *
 * ORIENTATION (CONFIRMED hardware fact):
 *   Sensor 1 is the RIGHTMOST channel when viewed from the robot's normal
 *   forward-facing orientation. Array index 0 == Sensor 1 == rightmost.
 *
 * Position convention (matches LineSensor.h / RobotConfig.h):
 *   position = +1  <=> line under the RIGHT edge (Sensor 1 side)
 *   position = -1  <=> line under the LEFT edge (Sensor 8 side)
 *   so a positive error means "line is to the RIGHT of the centreline".
 *
 * WHAT IS KNOWN / NOT KNOWN (nothing is invented — see README TODO):
 *   - 8 IR reflectance channels: KNOWN.
 *   - Digital output per channel, HIGH = line: ASSUMED (community wiring);
 *     TODO(hardware): verify polarity (lineIsHigh flag).
 *   - Channel order: Sensor 1..8 mapping is CONFIRMED for the pins given;
 *     TODO(hardware): verify with test_sensor.ino that Sensor 1 really is
 *     the rightmost channel on the physical robot.
 *   - Pins not yet assigned (Sensors 7/8) read as inactive; the driver
 *     never touches PIN_UNASSIGNED GPIOs.
 *
 * Position estimation (weighted average over active channels):
 *
 *        position = sum_i( w_i * s_i ) / sum_i( s_i ),   s_i in {0, 1}
 *
 * with weights w_i = ((N-1)/2 - i) / ((N-1)/2): Sensor 1 (index 0,
 * rightmost) -> +1, Sensor 8 (index 7, leftmost) -> -1. The raw position
 * is therefore already normalised to [-1, +1].
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
        bool  reverseOrder = false;///< TODO(hardware): set if the array is
                                   ///< mirrored on the physical robot.
    };

    explicit RLS08LineSensor(const Config& cfg);

    bool begin() override;
    bool update() override;
    LineMeasurement getMeasurement() const override;

    /// Raw digital channel states of the last update, indexed Sensor-1
    /// (index 0 = Sensor 1 = rightmost). Unassigned channels read false.
    void readRaw(bool out[NUM_CHANNELS]) const;

    /// GPIO level (HIGH/LOW) of each channel BEFORE polarity interpretation,
    /// same indexing. Useful to verify polarity with test_sensor.ino.
    void readLevels(bool out[NUM_CHANNELS]) const;

    /// Number of channels currently detecting the line.
    int activeChannelCount() const { return activeCount_; }

    /// Number of channels with a real pin assignment (Sensors 1-6 now;
    /// becomes 8 once Sensors 7/8 are confirmed).
    int assignedChannelCount() const { return assignedCount_; }

    /// Runtime polarity change (used by the MANUAL UI for verification).
    void setLineIsHigh(bool lineIsHigh) { cfg_.lineIsHigh = lineIsHigh; }
    bool getLineIsHigh() const { return cfg_.lineIsHigh; }

    /// Runtime order change (used by the MANUAL UI for verification).
    void setReverseOrder(bool reverse) { cfg_.reverseOrder = reverse; }
    bool getReverseOrder() const { return cfg_.reverseOrder; }

private:
    float channelWeight(int index) const;
    float computePosition() const;

    Config  cfg_;
    bool    raw_[NUM_CHANNELS];    ///< true = line detected (post-polarity)
    bool    levels_[NUM_CHANNELS]; ///< raw HIGH/LOW levels (pre-polarity)
    int     activeCount_;
    int     assignedCount_;
    /// Sign of the last valid reading; used to extrapolate when the line
    /// is only under an edge channel (partial confidence).
    float   lastDirectionSign_;
    LineMeasurement measurement_;
};

#endif // RLS08_LINE_SENSOR_H
