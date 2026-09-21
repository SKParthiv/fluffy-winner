/**
 * CalibrationMetrics.h
 * =====================
 * Independent, windowed performance metrics (task §13).
 *
 * Each metric is computed SEPARATELY so the calibration strategy (the
 * objective weighting) can be changed later without touching collectors.
 *
 * Accumulators run at the fast-loop rate (fed by the supervisor once per
 * fast loop via onFastLoopSample), but are only read/evaluated by the
 * slow supervisor at window boundaries.
 *
 * Metrics collected over a window of N samples:
 *   - Tracking:  mean square error  E_e = (1/N) * sum e_k^2
 *                RMS error          e_RMS = sqrt(E_e)
 *                peak |e|           (also used for instability detection)
 *   - Oscillation (IMU gyro based, when available):
 *                angular-velocity RMS  omega_RMS = sqrt((1/N) sum w_k^2)
 *                peak-to-peak omega    (max - min)
 *                zero-crossing rate of omega (oscillation frequency proxy)
 *   - Instability / failure:
 *                line-lost fraction of the window (invalid samples / N)
 *                peak-to-peak line error
 *
 * WHY separate classes for metrics and objective: the objective J is a
 * policy (weights), the metrics are physics. Policies change; physics
 * does not (task §13: "Implement the metrics independently").
 */

#ifndef CALIBRATION_METRICS_H
#define CALIBRATION_METRICS_H

#include <stdint.h>
#include <math.h>

struct MetricsWindow {
    // Tracking
    float meanSquareError = 0.0f;   ///< E_e = (1/N) sum e_k^2  [normalised^2]
    float rmsError        = 0.0f;   ///< sqrt(E_e)
    float peakAbsError    = 0.0f;   ///< max |e_k|
    float peakToPeakError = 0.0f;   ///< max e - min e

    // Oscillation (all zero when no IMU data was fed)
    float omegaRms        = 0.0f;   ///< [rad/s]
    float omegaPeakToPeak = 0.0f;   ///< [rad/s]
    float zeroCrossingRate = 0.0f; ///< crossings per second [1/s]

    // Instability
    float lineLostFraction = 0.0f;  ///< invalid samples / N in [0,1]

    bool  imuDataPresent = false;   ///< false if no IMU was connected
    uint32_t sampleCount = 0;
};

class CalibrationMetrics {
public:
    /// Start a fresh accumulation window.
    void beginWindow();

    /**
     * Feed one fast-loop sample.
     *   lineError     : current normalised line error (controller state)
     *   lineValid     : whether the sensor had the line this sample
     *   omegaZ        : IMU yaw rate [rad/s] (0 if no IMU)
     *   imuPresent    : whether an IMU sample was actually available
     */
    void onFastLoopSample(float lineError, bool lineValid,
                          float omegaZ, bool imuPresent);

    /// Close the window and compute the final metrics.
    MetricsWindow endWindow(float windowDurationS);

    bool hasImuData() const { return imuSamples_ > 0; }

private:
    // Running accumulators
    uint64_t errorSquareSumScaled_ = 0;  ///< sum of e_k^2 * SCALE (integer)
    uint32_t sampleCount_     = 0;
    uint32_t invalidCount_   = 0;
    float    minError_ = 0.0f, maxError_ = 0.0f;
    bool     haveMinMax_ = false;

    // Oscillation accumulators
    uint64_t omegaSquareSumScaled_ = 0;  ///< sum w_k^2 * SCALE
    uint32_t imuSamples_ = 0;
    float    minOmega_ = 0.0f, maxOmega_ = 0.0f;
    bool     haveOmegaMinMax_ = false;
    float    lastOmega_ = 0.0f;
    bool     haveLastOmega_ = false;
    uint32_t zeroCrossings_ = 0;

    // Fixed-point scaling keeps the hot-path accumulators in integers
    // (cheap on ESP32) while preserving plenty of precision:
    static const uint32_t SCALE = 4096;  // 12 fractional bits
};

#endif // CALIBRATION_METRICS_H
