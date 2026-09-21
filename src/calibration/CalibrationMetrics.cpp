/**
 * CalibrationMetrics.cpp
 * =======================
 * Discrete approximations of the continuous metric definitions (task §13):
 *
 *   E_e   = (1/T) ∫ e²(t) dt   ≈ (1/N) Σ e_k²        (uniform fast-loop rate)
 *   e_RMS = sqrt(E_e)
 *   ω_RMS = sqrt( (1/N) Σ ω_k² )
 *
 * Zero-crossing rate = (# sign changes of ω) / T  — a proxy for the
 * oscillation frequency of the robot's heading motion.
 */

#include "CalibrationMetrics.h"

void CalibrationMetrics::beginWindow() {
    errorSquareSumScaled_ = 0;
    omegaSquareSumScaled_ = 0;
    sampleCount_ = 0;
    invalidCount_ = 0;
    imuSamples_ = 0;
    zeroCrossings_ = 0;
    haveMinMax_ = false;
    haveOmegaMinMax_ = false;
    haveLastOmega_ = false;
    minError_ = maxError_ = 0.0f;
    minOmega_ = maxOmega_ = 0.0f;
    lastOmega_ = 0.0f;
}

void CalibrationMetrics::onFastLoopSample(float lineError, bool lineValid,
                                          float omegaZ, bool imuPresent) {
    ++sampleCount_;
    if (!lineValid) ++invalidCount_;

    // Track min/max of the error regardless of validity (peak-to-peak
    // error includes the excursion during a loss).
    if (!haveMinMax_ || lineError < minError_) { minError_ = lineError; haveMinMax_ = true; }
    if (!haveMinMax_ || lineError > maxError_) { maxError_ = lineError; haveMinMax_ = true; }

    // Fixed-point accumulate e² (12 fractional bits).
    const float e2 = lineError * lineError;
    errorSquareSumScaled_ += (uint64_t)(e2 * SCALE);

    if (imuPresent) {
        ++imuSamples_;
        const float w2 = omegaZ * omegaZ;
        omegaSquareSumScaled_ += (uint64_t)(w2 * SCALE);

        if (!haveOmegaMinMax_ || omegaZ < minOmega_) { minOmega_ = omegaZ; haveOmegaMinMax_ = true; }
        if (!haveOmegaMinMax_ || omegaZ > maxOmega_) { maxOmega_ = omegaZ; haveOmegaMinMax_ = true; }

        if (haveLastOmega_ &&
            ((lastOmega_ < 0.0f && omegaZ >= 0.0f) ||
             (lastOmega_ > 0.0f && omegaZ <= 0.0f))) {
            ++zeroCrossings_;
        }
        lastOmega_ = omegaZ;
        haveLastOmega_ = true;
    }
}

MetricsWindow CalibrationMetrics::endWindow(float windowDurationS) {
    MetricsWindow w;
    w.sampleCount = sampleCount_;
    w.lineLostFraction = (sampleCount_ > 0)
        ? (float)invalidCount_ / (float)sampleCount_ : 0.0f;
    w.imuDataPresent = (imuSamples_ > 0);

    if (sampleCount_ > 0) {
        // Undo fixed-point scaling and normalise by N.
        const float n = (float)sampleCount_;
        w.meanSquareError = ((float)errorSquareSumScaled_ / (float)SCALE) / n;
        w.rmsError = sqrtf(w.meanSquareError);
        w.peakAbsError = (fabsf(minError_) > fabsf(maxError_))
                             ? fabsf(minError_) : fabsf(maxError_);
        w.peakToPeakError = maxError_ - minError_;
    }

    if (imuSamples_ > 0) {
        const float n = (float)imuSamples_;
        w.omegaRms = sqrtf(((float)omegaSquareSumScaled_ / (float)SCALE) / n);
        w.omegaPeakToPeak = maxOmega_ - minOmega_;
        w.zeroCrossingRate = (windowDurationS > 0.0f)
            ? (float)zeroCrossings_ / windowDurationS : 0.0f;
    }

    return w;
}
