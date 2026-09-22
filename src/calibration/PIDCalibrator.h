/**
 * PIDCalibrator.h
 * ================
 * Slow supervisory PID-gain optimiser (task §11-§17).
 *
 * ARCHITECTURAL CONTRACT (the most important rule of the project):
 *   - The calibrator NEVER outputs motor commands and NEVER blocks the
 *     fast loop. Its only actuation is PathController::setGains().
 *   - It runs at ~1 Hz (updatePeriodUs), while the fast loop runs at
 *     200 Hz. It observes the robot through the fast controller's state
 *     (line error, validity) and the IMU (oscillation), both pushed into
 *     it by main.ino once per fast loop — a cheap function call, no
 *     coupling of the controller to the calibrator.
 *
 * ALGORITHM (deliberately simple, deterministic, replaceable — task §15):
 *   Bounded coordinate descent:
 *     1. Evaluate J for the current gains over window T (COLLECTING).
 *     2. Perturb ONE gain (kp, then ki, then kd) by a relative step,
 *        clamped to the hard bounds (TESTING_CANDIDATE).
 *     3. Evaluate J for the candidate over window T.
 *     4. If J improves by more than minImprovementFraction: ACCEPT
 *        (candidate becomes current, and lastKnownGood after a full
 *        accepted round); else REJECT and revert.
 *     5. Next coordinate; after kd, start a new round at step 2.
 *   Both +step and -step are tried per coordinate before moving on.
 *
 * SAFETY (task §16):
 *   - All candidates are clamped to [kpMin,kpMax] etc. BEFORE testing.
 *   - lastKnownGoodGains: only gains that survived a full accepted round
 *     are committed. On ANY failure the fast controller is immediately
 *     reverted to lastKnownGoodGains and calibration disables itself.
 *   - Failures: line lost longer than lineLossAbortS during a window,
 *     total time > timeoutS, window with excessive lineLostFraction.
 *
 * OBJECTIVE (task §14 — avoid the "zero gain = zero oscillation" trap):
 *   J = w_e * E_tracking + w_o * E_oscillation + w_s * E_instability
 *   with E_tracking   = meanSquareError          (dominant, w_e >= w_o)
 *        E_oscillation = omegaRms (normalised) + peakToPeakError (normalised)
 *        E_instability = lineLostFraction * penalty
 *   Zero gains produce a HUGE tracking error, so the optimiser can never
 *   "win" by not steering. The weights are configurable.
 */

#ifndef PID_CALIBRATOR_H
#define PID_CALIBRATOR_H

#include "CalibrationState.h"
#include "CalibrationMetrics.h"
#include "../config/RobotConfig.h"
#include "../sensors/IMUInterface.h"

/// Interface the calibrator needs from the fast controller. Implemented
/// by PathController. WHY a narrow interface: the calibrator must not
/// know about motors or sensors, only about gains + observable state.
class CalibrationTarget {
public:
    virtual ~CalibrationTarget() {}
    virtual void setGains(const PIDGains& g) = 0;
    virtual const PIDGains& getGains() const = 0;
    virtual float getLastLineError() const = 0;
    virtual bool  isLineLost() const = 0;
};

class PIDCalibrator {
public:
    PIDCalibrator(CalibrationTarget& target, IMUInterface* imu,
                  const CalibrationConfig& cfg);

    /// Enable/disable at runtime (diagnostics commands). Never blocks.
    void enable();
    void disable();
    bool isEnabled() const { return state_ != CalibrationState::CAL_DISABLED; }

    /// True when the slow supervisor period has elapsed.
    bool shouldRun(uint32_t nowUs) const;

    /// One supervisor step (~1 Hz). Non-blocking.
    void update(uint32_t nowUs);

    /**
     * Called by main.ino once per FAST loop to feed observations.
     * Deliberately cheap: two loads, a few adds. The fast loop calls it,
     * but the calibrator never calls INTO the fast loop except setGains.
     */
    void onFastLoopSample(uint32_t nowUs);

    /// Restore lastKnownGoodGains immediately (diagnostics command).
    void restoreLastKnownGood();

    /// Reset all calibration state (diagnostics command).
    void reset();

    // --- Diagnostics accessors ---
    CalibrationState getState() const { return state_; }
    const PIDGains& getLastKnownGoodGains() const { return lastKnownGoodGains_; }
    const PIDGains& getCurrentGains() const { return currentGains_; }
    const PIDGains& getCandidateGains() const { return candidateGains_; }
    float getLastObjective() const { return lastObjective_; }
    float getCandidateObjective() const { return candidateObjective_; }
    const MetricsWindow& getLastWindow() const { return lastWindow_; }
    uint32_t getRoundCount() const { return roundCount_; }
    const char* getFailReason() const { return failReason_; }

private:
    float computeObjective(const MetricsWindow& w) const;
    PIDGains perturb(const PIDGains& g) const;
    PIDGains clamped(const PIDGains& g) const;
    bool windowIsAcceptable(const MetricsWindow& w) const;
    void fail(const char* reason);          ///< revert + disable
    void startWindow(uint32_t nowUs);
    bool windowElapsed(uint32_t nowUs) const;
    bool proposeNextCandidate(uint32_t nowUs);  ///< build+apply next candidate, false if exhausted
    void advanceSearch();                   ///< next coordinate/direction, round commit

    CalibrationTarget& target_;
    IMUInterface*      imu_;        ///< may be nullptr: metrics degrade gracefully
    CalibrationConfig  cfg_;

    CalibrationState   state_;
    CalibrationMetrics metrics_;

    PIDGains baselineGains_;        ///< gains at enable() time
    PIDGains lastKnownGoodGains_;   ///< last fully-committed safe gains
    PIDGains currentGains_;         ///< gains under evaluation
    PIDGains candidateGains_;       ///< perturbed gains being tested

    float lastObjective_;           ///< J(currentGains)
    float candidateObjective_;      ///< J(candidateGains)
    MetricsWindow lastWindow_;

    // Coordinate descent bookkeeping
    bool     evaluatingCandidate_;  ///< is the live window a candidate window?
    int      coordinate_;           ///< 0=kp 1=ki 2=kd
    int      direction_;            ///< +1 try +step first, -1 then -step
    uint32_t roundCount_;
    bool     roundImproved_;        ///< did any candidate in this round pass?

    // Window timing
    uint32_t windowStartUs_;
    uint32_t lastRunUs_;
    uint32_t enabledAtUs_;
    uint32_t lastLineSeenUs_;       ///< for line-loss abort
    const char* failReason_;
};

#endif // PID_CALIBRATOR_H
