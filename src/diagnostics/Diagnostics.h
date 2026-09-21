/**
 * Diagnostics.h
 * ==============
 * Rate-limited serial diagnostics + a small serial COMMAND interface
 * (task §21).
 *
 * Commands (single line, newline-terminated, case-insensitive):
 *   gains        - print current PID gains
 *   metrics      - print last calibration metrics + objective
 *   sensor       - print sensor state (raw channels + position)
 *   controller   - print controller state (error, v, omega, line lost)
 *   timing       - print loop timing statistics
 *   cal on       - enable calibration (no-op if compiled out)
 *   cal off      - disable calibration, revert to last-known-good
 *   cal reset    - reset calibration state
 *   cal restore  - restore last-known-good gains immediately
 *   help         - list commands
 *
 * Periodic status printing is RATE-LIMITED to printPeriodUs (1 Hz
 * default). NOTHING in this class is ever called from the fast control
 * path; main.ino calls update() at its leisure.
 */

#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "../config/RobotConfig.h"
#include "../control/PathController.h"
#include "../sensors/LineSensor.h"
#include "../sensors/RLS08LineSensor.h"
#include "../calibration/PIDCalibrator.h"
#include "../calibration/CalibrationState.h"

class Diagnostics {
public:
    Diagnostics(const RobotConfig& cfg);

    /// Wire up the modules to inspect. Call after their construction.
    void attach(PathController* controller,
               RLS08LineSensor* rls08,
               PIDCalibrator* calibrator);

    void begin();

    /// Call from loop(): handles serial commands + rate-limited status.
    void update(uint32_t nowUs);

private:
    void handleCommand(const char* cmd);
    void printGains();
    void printMetrics();
    void printSensor();
    void printController();
    void printTiming();
    void printHelp();
    void printStatus();          ///< the periodic 1-line heartbeat

    const RobotConfig& cfg_;
    PathController*    controller_;
    RLS08LineSensor*   rls08_;
    PIDCalibrator*     calibrator_;   ///< may be nullptr (flag off)

    uint32_t lastPrintUs_;
    char     line_[32];
    uint8_t  lineLen_;
};

#endif // DIAGNOSTICS_H
