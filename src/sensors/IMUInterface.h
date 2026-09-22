/**
 * IMUInterface.h
 * ==============
 * Abstract IMU interface. NO concrete IMU is implemented because the model
 * is unknown at this stage (task §5, §24) — do not pretend yaw is available.
 *
 * Contract:
 *   - update() polls the IMU, non-blocking, returns true on fresh data.
 *   - getAngularVelocityZ() [rad/s], positive = counter-clockwise (right-hand
 *     rule around +z, pointing up out of the robot). This is the one IMU
 *     signal the oscillation metrics rely on; a single-axis gyro suffices.
 *   - getYaw() returns an ABSOLUTE heading estimate [rad] and yawValid()
 *     reports whether the device actually provides one. A plain gyro CANNOT
 *     provide absolute yaw (only integrated, drifting relative yaw), so a
 *     gyro-only driver must return yawValid() == false. The calibration
 *     module must check yawValid() before using yaw-based metrics.
 *   - hasNewData() lets the slow calibration loop consume at the IMU's own
 *     output data rate rather than assuming it matches the loop rate.
 */

#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include <stdint.h>

class IMUInterface {
public:
    virtual ~IMUInterface() {}

    /// Initialise the device. Returns false if unavailable.
    virtual bool begin() = 0;

    /// Poll for new data. Non-blocking. True when a fresh sample arrived.
    virtual bool update() = 0;

    /// True when getAngularVelocityZ()/getYaw() hold a fresh sample.
    virtual bool hasNewData() const = 0;

    /// Yaw rate about the robot's vertical axis [rad/s]. CCW positive.
    virtual float getAngularVelocityZ() const = 0;

    /// Absolute yaw estimate [rad] IF the device provides one.
    virtual float getYaw() const = 0;

    /// Whether getYaw() is a trustworthy absolute heading.
    /// Gyro-only implementations must return false here.
    virtual bool yawValid() const = 0;
};

#endif // IMU_INTERFACE_H
