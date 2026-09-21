/**
 * CalibrationState.h
 * ===================
 * Explicit state machine for the slow calibration supervisor (task §17).
 *
 *   DISABLED  -- calibration off; robot runs on baseline/last-good gains
 *        |
 *   IDLE      -- armed, waiting for the next evaluation window to start
 *        |
 *   COLLECTING -- accumulating metrics over the evaluation window T with
 *        |       the CURRENT gains (baseline measurement)
 *   EVALUATING -- window closed; compute J for the current gains
 *        |
 *   TESTING_CANDIDATE -- a perturbed candidate is applied (via the fast
 *        |               controller's setGains) and evaluated over T
 *   ACCEPT/REJECT     -- compare J(candidate) vs J(current); commit or
 *        |              revert; then next coordinate / next round
 *   IDLE (loop)
 *
 * Invariants:
 *   - The fast controller keeps running in EVERY state.
 *   - Only setGains() is ever called on the fast controller.
 *   - On any failure (line lost too long, timeout, invalid candidate)
 *     the supervisor reverts to lastKnownGoodGains and goes DISABLED.
 */

#ifndef CALIBRATION_STATE_H
#define CALIBRATION_STATE_H

enum class CalibrationState {
    CAL_DISABLED,
    CAL_IDLE,
    CAL_COLLECTING,
    CAL_EVALUATING,
    CAL_TESTING_CANDIDATE,
    CAL_ACCEPT_REJECT
};

inline const char* calibrationStateName(CalibrationState s) {
    switch (s) {
        case CalibrationState::CAL_DISABLED:         return "DISABLED";
        case CalibrationState::CAL_IDLE:            return "IDLE";
        case CalibrationState::CAL_COLLECTING:      return "COLLECTING";
        case CalibrationState::CAL_EVALUATING:      return "EVALUATING";
        case CalibrationState::CAL_TESTING_CANDIDATE:return "TESTING_CANDIDATE";
        case CalibrationState::CAL_ACCEPT_REJECT:   return "ACCEPT_REJECT";
    }
    return "UNKNOWN";
}

#endif // CALIBRATION_STATE_H
