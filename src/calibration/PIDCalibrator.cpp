/**
 * PIDCalibrator.cpp
 * ==================
 * Bounded coordinate descent over the PID gains.
 *
 * Window semantics (one window per state visit, ~evaluationWindowS long):
 *   COLLECTING        : baseline window, evaluates J(currentGains)
 *   EVALUATING       : closes the window, computes J; if the window was
 *                      the baseline -> propose candidate and enter
 *                      TESTING_CANDIDATE; if it was a candidate window
 *                      -> enter ACCEPT_REJECT.
 *   TESTING_CANDIDATE : candidate gains are LIVE on the fast controller
 *                      while metrics accumulate.
 *   ACCEPT_REJECT     : compare J(candidate) vs J(current); accept =>
 *                      candidate becomes current (and lastKnownGood after
 *                      a full improved round); reject => revert. Then
 *                      propose the next candidate and re-enter
 *                      TESTING_CANDIDATE.
 *
 * All transitions are non-blocking; the fast loop runs throughout.
 */

#include "PIDCalibrator.h"
#include <Arduino.h>

PIDCalibrator::PIDCalibrator(CalibrationTarget& target, IMUInterface* imu,
                             const CalibrationConfig& cfg)
    : target_(target),
      imu_(imu),
      cfg_(cfg),
      state_(CalibrationState::CAL_DISABLED),
      evaluatingCandidate_(false),
      coordinate_(0),
      direction_(1),
      roundCount_(0),
      roundImproved_(false),
      windowStartUs_(0),
      lastRunUs_(0),
      enabledAtUs_(0),
      lastLineSeenUs_(0),
      lastObjective_(0.0f),
      candidateObjective_(0.0f),
      failReason_("") {
    baselineGains_      = target.getGains();
    lastKnownGoodGains_ = baselineGains_;
    currentGains_       = baselineGains_;
    candidateGains_      = baselineGains_;
}

// ---------------------------------------------------------------------------
// Enable / disable / reset
// ---------------------------------------------------------------------------

void PIDCalibrator::enable() {
    if (state_ != CalibrationState::CAL_DISABLED) return;

    baselineGains_       = target_.getGains();
    lastKnownGoodGains_  = baselineGains_;  // safe fallback from the start
    currentGains_        = baselineGains_;
    candidateGains_      = baselineGains_;
    coordinate_ = 0;
    direction_  = 1;
    roundCount_ = 0;
    roundImproved_ = false;
    evaluatingCandidate_ = false;
    failReason_ = "";
    enabledAtUs_ = micros();
    lastRunUs_ = enabledAtUs_;
    lastLineSeenUs_ = enabledAtUs_;

    // Start with a baseline evaluation of the current (safe) gains.
    state_ = CalibrationState::CAL_COLLECTING;
    startWindow(enabledAtUs_);
}

void PIDCalibrator::disable() {
    // Always leave the robot on the last known good gains.
    target_.setGains(lastKnownGoodGains_);
    currentGains_ = lastKnownGoodGains_;
    state_ = CalibrationState::CAL_DISABLED;
}

void PIDCalibrator::reset() {
    disable();
    // After a reset the "last known good" reverts to the configuration
    // baseline so a subsequent enable() starts from scratch.
    lastKnownGoodGains_ = baselineGains_;
    lastObjective_ = 0.0f;
    candidateObjective_ = 0.0f;
    roundCount_ = 0;
}

void PIDCalibrator::restoreLastKnownGood() {
    target_.setGains(lastKnownGoodGains_);
    currentGains_ = lastKnownGoodGains_;
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

bool PIDCalibrator::shouldRun(uint32_t nowUs) const {
    return isEnabled() && (uint32_t)(nowUs - lastRunUs_) >= cfg_.updatePeriodUs;
}

void PIDCalibrator::startWindow(uint32_t nowUs) {
    metrics_.beginWindow();
    windowStartUs_ = nowUs;
}

bool PIDCalibrator::windowElapsed(uint32_t nowUs) const {
    return (nowUs - windowStartUs_) >= (uint32_t)(cfg_.evaluationWindowS * 1e6f);
}

// ---------------------------------------------------------------------------
// Fast-loop observation feed (cheap; called at 200 Hz by main.ino)
// ---------------------------------------------------------------------------

void PIDCalibrator::onFastLoopSample(uint32_t nowUs) {
    if (state_ != CalibrationState::CAL_COLLECTING &&
        state_ != CalibrationState::CAL_TESTING_CANDIDATE) {
        return;  // only accumulate during live evaluation windows
    }

    const float lineError = target_.getLastLineError();
    const bool  lineValid = !target_.isLineLost();

    float omegaZ = 0.0f;
    bool  imuPresent = false;
    if (imu_ != nullptr && imu_->update()) {
        omegaZ = imu_->getAngularVelocityZ();
        imuPresent = true;
    }

    metrics_.onFastLoopSample(lineError, lineValid, omegaZ, imuPresent);

    if (lineValid) lastLineSeenUs_ = nowUs;
}

// ---------------------------------------------------------------------------
// Slow supervisor step (~1 Hz)
// ---------------------------------------------------------------------------

void PIDCalibrator::update(uint32_t nowUs) {
    if (state_ == CalibrationState::CAL_DISABLED) return;
    lastRunUs_ = nowUs;

    // --- Global safety: hard timeout ------------------------------------
    if ((nowUs - enabledAtUs_) > (uint32_t)(cfg_.timeoutS * 1e6f)) {
        fail("timeout");
        return;
    }

    // --- Global safety: line lost too long, even mid-window -------------
    if ((uint32_t)(nowUs - lastLineSeenUs_) >
        (uint32_t)(cfg_.lineLossAbortS * 1e6f)) {
        fail("line lost");
        return;
    }

    switch (state_) {
        case CalibrationState::CAL_IDLE:
            // (Reached only if a future strategy needs a pause; keep the
            //  machine total.) Re-arm a baseline window.
            evaluatingCandidate_ = false;
            state_ = CalibrationState::CAL_COLLECTING;
            startWindow(nowUs);
            break;

        case CalibrationState::CAL_COLLECTING:
        case CalibrationState::CAL_TESTING_CANDIDATE:
            if (windowElapsed(nowUs)) {
                state_ = CalibrationState::CAL_EVALUATING;
            }
            break;

        case CalibrationState::CAL_EVALUATING: {
            lastWindow_ = metrics_.endWindow(cfg_.evaluationWindowS);

            if (!windowIsAcceptable(lastWindow_)) {
                fail("window unacceptable (line lost too often)");
                return;
            }

            if (!evaluatingCandidate_) {
                // Baseline window: record J for the current gains and
                // propose the first candidate.
                lastObjective_ = computeObjective(lastWindow_);
                if (!proposeNextCandidate(nowUs)) {
                    // Nothing left to try; stay put and keep the gains.
                    state_ = CalibrationState::CAL_IDLE;
                    break;
                }
            } else {
                // Candidate window finished: decide next.
                candidateObjective_ = computeObjective(lastWindow_);
                state_ = CalibrationState::CAL_ACCEPT_REJECT;
            }
            break;
        }

        case CalibrationState::CAL_ACCEPT_REJECT: {
            // Accept only on a CLEAR improvement (noise guard).
            const float improvement = lastObjective_ - candidateObjective_;
            const float threshold =
                cfg_.minImprovementFraction *
                (lastObjective_ > 0.0f ? lastObjective_ : 1.0f);

            if (improvement > threshold) {
                // ACCEPT: candidate becomes current; its measured J is
                // the new baseline for the next comparisons.
                currentGains_ = candidateGains_;
                lastObjective_ = candidateObjective_;
                roundImproved_ = true;
            }
            // REJECT: currentGains_ stays; the revert happens below when
            // the next candidate (or the exhausted-search fallback) is
            // written to the fast controller.

            // Advance the search (next direction, then next coordinate)
            // BEFORE proposing, otherwise the same perturbation would be
            // re-proposed forever after a rejection.
            advanceSearch();

            if (!proposeNextCandidate(nowUs)) {
                // Search exhausted: revert to current gains and idle.
                target_.setGains(currentGains_);
                state_ = CalibrationState::CAL_IDLE;
            }
            break;
        }

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Internals
// ---------------------------------------------------------------------------

bool PIDCalibrator::proposeNextCandidate(uint32_t nowUs) {
    // Try up to 6 perturbations (3 coordinates x 2 directions) so a
    // candidate pinned at a hard bound does not stall the search.
    for (int attempt = 0; attempt < 6; ++attempt) {
        candidateGains_ = clamped(perturb(currentGains_));

        if (candidateGains_ != currentGains_) {
            // Put the candidate LIVE on the fast controller (the only
            // write the calibrator ever makes) and start its window.
            target_.setGains(candidateGains_);
            evaluatingCandidate_ = true;
            state_ = CalibrationState::CAL_TESTING_CANDIDATE;
            startWindow(nowUs);
            return true;
        }

        advanceSearch();
    }
    return false;  // every perturbation is clamped away: nothing to test
}

void PIDCalibrator::advanceSearch() {
    if (direction_ > 0) {
        direction_ = -1;                       // still try -step
    } else {
        direction_ = +1;
        coordinate_ = (coordinate_ + 1) % 3;
        if (coordinate_ == 0) {
            // A full round over (kp, ki, kd) is complete.
            ++roundCount_;
            if (roundImproved_) {
                // Commit: these gains survived a full round with at least
                // one accepted improvement (task §16).
                lastKnownGoodGains_ = currentGains_;
                roundImproved_ = false;
            }
        }
    }
}

PIDGains PIDCalibrator::perturb(const PIDGains& g) const {
    PIDGains p = g;
    const float step =
        (coordinate_ == 0) ? cfg_.kpStep :
        (coordinate_ == 1) ? cfg_.kiStep : cfg_.kdStep;
    const float sign = (direction_ > 0) ? 1.0f : -1.0f;

    switch (coordinate_) {
        case 0: p.kp = g.kp * (1.0f + sign * step); break;
        case 1: p.ki = g.ki * (1.0f + sign * step); break;
        case 2: p.kd = g.kd * (1.0f + sign * step); break;
    }
    return p;
}

PIDGains PIDCalibrator::clamped(const PIDGains& g) const {
    // Hard bounds (task §16): the calibrator can NEVER produce gains
    // outside these ranges, no matter what the search suggests.
    PIDGains c = g;
    if (c.kp < cfg_.limits.kpMin) c.kp = cfg_.limits.kpMin;
    if (c.kp > cfg_.limits.kpMax) c.kp = cfg_.limits.kpMax;
    if (c.ki < cfg_.limits.kiMin) c.ki = cfg_.limits.kiMin;
    if (c.ki > cfg_.limits.kiMax) c.ki = cfg_.limits.kiMax;
    if (c.kd < cfg_.limits.kdMin) c.kd = cfg_.limits.kdMin;
    if (c.kd > cfg_.limits.kdMax) c.kd = cfg_.limits.kdMax;
    return c;
}

bool PIDCalibrator::windowIsAcceptable(const MetricsWindow& w) const {
    // A window in which the line was missing a large fraction of the
    // time says nothing about tracking quality: treat as failure, not
    // as data (task §18: sensor confidence becoming invalid).
    return w.sampleCount > 0 && w.lineLostFraction < 0.25f;
}

float PIDCalibrator::computeObjective(const MetricsWindow& w) const {
    // J = w_e*E_tracking + w_o*E_oscillation + w_s*E_instability  (task §14)
    //
    // E_tracking = mean square line error. WHY this makes "zero gain"
    // unattractive: with no steering the robot drifts off the line, the
    // error saturates near ±1 and E_tracking -> ~1, dominating J. The
    // optimiser cannot win by refusing to steer.
    const float eTracking = w.meanSquareError;

    // E_oscillation = normalised gyro RMS + normalised peak-to-peak line
    // error. Without an IMU only the line-error proxy remains: calibration
    // still functions, just with a blinder oscillation estimate.
    const float omegaNorm = w.omegaRms / 3.0f;        // ~3 rad/s = strong shake
    const float p2pNorm   = w.peakToPeakError / 2.0f; // full range = 2
    const float eOscillation = omegaNorm + p2pNorm;

    // E_instability = line-loss fraction of the window.
    const float eInstability = w.lineLostFraction;

    return cfg_.weightTracking * eTracking +
           cfg_.weightOscillation * eOscillation +
           cfg_.weightInstability * eInstability;
}

void PIDCalibrator::fail(const char* reason) {
    // MANDATORY recovery (task §16, §18): revert the fast controller to
    // the last known good gains and disable the supervisor. The robot
    // keeps following the line with proven gains; calibration failure
    // can never stop the normal controller.
    target_.setGains(lastKnownGoodGains_);
    currentGains_ = lastKnownGoodGains_;
    state_ = CalibrationState::CAL_DISABLED;
    failReason_ = reason;
}
