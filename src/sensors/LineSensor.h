/**
 * LineSensor.h
 * ============
 * Hardware-independent interface for the line-position sensor.
 *
 * WHY: the RLS08's exact electrical behaviour is not fully documented in
 * this repository, so all controller code talks to THIS interface. The
 * hardware driver (RLS08LineSensor) and a mock (MockLineSensor) both
 * implement it; swapping hardware never touches the controller.
 *
 * Measurement contract:
 *   position    : normalised lateral line position in [-1, +1].
 *                 0 = line centred under the array.
 *                 +1 = line at the right edge of the array.
 *                 -1 = line at the left edge.
 *                 (Sign convention matches RobotConfig.h: positive error
 *                  means the line is to the RIGHT of the robot centreline.)
 *   confidence  : 0..1 estimate of how trustworthy `position` is
 *                 (e.g. fraction of active channels, edge extrapolation
 *                  gets less confidence than an interior reading).
 *   valid       : false when no line was detected at all this update.
 *   timestampUs : micros() at the time the reading was taken.
 */

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

struct LineMeasurement {
    float    position;
    float    confidence;
    bool     valid;
    uint32_t timestampUs;

    LineMeasurement()
        : position(0.0f), confidence(0.0f), valid(false), timestampUs(0) {}
};

class LineSensor {
public:
    virtual ~LineSensor() {}

    /// Initialise hardware. Returns false if the sensor is unusable.
    virtual bool begin() = 0;

    /**
     * Poll the sensor. Called once per fast-loop iteration; must be
     * non-blocking. Returns true when a NEW measurement is available
     * since the previous update() call.
     */
    virtual bool update() = 0;

    /// Most recent measurement (only meaningful after update() returned true).
    virtual LineMeasurement getMeasurement() const = 0;
};

#endif // LINE_SENSOR_H
