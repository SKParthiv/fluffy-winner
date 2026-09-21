/**
 * PathController.h
 * ================
 * The FAST line-following control loop. Owns the whole pipeline:
 *
 *   LineSensor -> error estimate -> PID -> desired point velocity
 *              -> PointKinematics -> (v, omega) -> DifferentialDrive
 *
 * IMPORTANT ARCHITECTURAL RULES (task §2, §26):
 *   - This class NEVER reads the IMU and NEVER talks to the calibrator.
 *   - The calibrator may set gains via setGains(); that is the ONLY
 *     interaction point, and it is a cheap, non-blocking write.
 *   - With CALIBRATION_ENABLED = false this class is fully functional.
 *
 * Error definition (see RobotConfig.h sign convention):
 *   e = sensorPosition - targetPosition
 *   e > 0 <=> line is to the RIGHT of the robot centreline.
 *
 * Control law (body-frame control point, l metres ahead of the axle):
 *   u_forward  = v_forward                       (cruise speed)
 *   u_lateral  = -PID(e)                         (steering correction)
 *   => v = u_forward,  omega = u_lateral / l     (PointKinematics)
 *
 * WHY u_lateral = -PID(e): a POSITIVE error (line to the right) must move
 * the control point to the RIGHT, i.e. negative body-y direction (+y is
 * LEFT), producing omega < 0 (nose swings right, toward the line).
 *
 * Heading error: NOT implemented. The 8-channel digital array provides a
 * lateral position only; it cannot measure the line's direction relative
 * to the robot (task §6: only implement what the sensor geometry supports).
 * Heading feedback can be added later via an IMU/fusion, through a
 * separate term — the architecture leaves room for it.
 *
 * Line-loss behaviour: when the sensor reports invalid data the controller
 * keeps the last valid error sign and performs a bounded search turn
 * (configurable fraction of max angular velocity) while keeping a slow
 * forward crawl — a standard, predictable recovery behaviour.
 */

#ifndef PATH_CONTROLLER_H
#define PATH_CONTROLLER_H

#include "PIDController.h"
#include "PointKinematics.h"
#include "../sensors/LineSensor.h"
#include "../motion/DifferentialDrive.h"
#include "../config/RobotConfig.h"
#include <stdint.h>

// Explicit UINT32_MAX for the min-period initialiser without depending
// on <stdint.h> macro availability across Arduino cores.
static const uint32_t kLoopPeriodInit = 0xFFFFFFFFu;

class PathController {
public:
    struct Config {
        float searchTurnFraction;  ///< line-loss turn rate as fraction of max omega
        float searchForwardSpeed;  ///< line-loss forward crawl [m/s]
    };

    PathController(LineSensor& sensor, DifferentialDrive& drive,
                   const RobotConfig& cfg);

    void begin();

    /// True when at least `controlLoopPeriodUs` elapsed since the last run.
    bool shouldRun(uint32_t nowUs) const;

    /// One control step. `nowUs` = micros(). Non-blocking, no delay().
    void update(uint32_t nowUs);

    /// --- Calibrator/diagnostics interaction points (cheap writes) ---
    void setGains(const PIDGains& gains);
    const PIDGains& getGains() const;
    void setIntegralEnabled(bool enabled);

    /// Loop-timing statistics (task §7): measured over real executions.
    uint32_t getLastLoopPeriodUs() const { return lastLoopPeriodUs_; }
    uint32_t getMinLoopPeriodUs()  const { return minLoopPeriodUs_; }
    uint32_t getMaxLoopPeriodUs()  const { return maxLoopPeriodUs_; }
    uint32_t getAvgLoopPeriodUs()  const;   ///< rolling average
    uint32_t getLoopCount() const { return loopCount_; }

    /// Current controller state (for calibration metrics / diagnostics).
    float getLastLineError() const { return lastError_; }
    bool  isLineLost() const { return lineLost_; }
    float getLastOutputV() const { return lastV_; }
    float getLastOutputOmega() const { return lastOmega_; }

    const PIDController& pid() const { return pid_; }

private:
    LineSensor&        sensor_;
    DifferentialDrive& drive_;
    PIDController      pid_;
    PointKinematics    kinematics_;

    // Config copies (fast access, no pointer chasing in the hot loop)
    float   targetPosition_;
    float   forwardVelocity_;
    float   maxLinearVelocity_;
    float   maxAngularVelocity_;
    uint32_t loopPeriodUs_;
    Config  behaviour_;

    // Timing state
    uint32_t lastRunUs_;
    uint32_t lastLoopPeriodUs_;
    uint32_t minLoopPeriodUs_;
    uint32_t maxLoopPeriodUs_;
    uint64_t periodSumUs_;     ///< for the average; 64-bit, no overflow
    uint32_t loopCount_;

    // Control state
    float lastError_;
    bool  lineLost_;
    float lastErrorSign_;   ///< sign of last valid error, for search turn
    float lastV_, lastOmega_;
};

#endif // PATH_CONTROLLER_H
