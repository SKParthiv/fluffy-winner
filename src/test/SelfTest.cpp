/**
 * SelfTest.cpp — see SelfTest.h.
 *
 * Implementation notes:
 *  - A MockCalibrationTarget stands in for the PathController during
 *    calibrator tests, so the calibrator's logic is tested without any
 *    motors or sensors.
 *  - Timing inside tests uses a virtual clock (passed micros values),
 *    NOT real time, so the tests are deterministic.
 */

#include "SelfTest.h"
#include <Arduino.h>
#include "../control/PIDController.h"
#include "../control/PointKinematics.h"
#include "../motion/DifferentialDrive.h"
#include "../calibration/CalibrationMetrics.h"
#include "../calibration/PIDCalibrator.h"
#include "../sensors/MockSensors.h"
#include <math.h>
#include <stdio.h>

namespace selftest {

// ---------------------------------------------------------------------------
// Tiny test framework (no dynamic allocation, no dependencies)
// ---------------------------------------------------------------------------

static int g_failures = 0;

static void report(bool ok, const char* name) {
#ifdef ARDUINO
    Serial.print(ok ? F("[PASS] ") : F("[FAIL] "));
    Serial.println(name);
#else
    printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", name);
#endif
    if (!ok) ++g_failures;
}

static bool approxEq(float a, float b, float tol = 1e-4f) {
    return fabsf(a - b) <= tol;
}

// ---------------------------------------------------------------------------
// PIDController
// ---------------------------------------------------------------------------

static void testPID() {
    ControllerConfig cfg;
    cfg.gains = PIDGains(2.0f, 1.0f, 0.1f);
    cfg.integralEnabled = true;
    cfg.errorLimit = 1.0f;
    cfg.integralLimit = 0.3f;
    cfg.derivativeFilterAlpha = 1.0f;  // raw derivative for exactness

    // Zero error -> zero output.
    {
        PIDController pid(cfg);
        report(approxEq(pid.compute(0.0f, 0.01f, 10.0f), 0.0f), "pid: zero error");
    }

    // Positive error -> positive output (P + I contributions; D=0 on
    // the first step). u = Kp*e + Ki*e*dt = 2*0.5 + 1*0.5*0.01 = 1.005.
    {
        PIDController pid(cfg);
        const float u = pid.compute(0.5f, 0.01f, 10.0f);
        report(approxEq(u, 1.005f, 1e-3f), "pid: positive error");
    }

    // Negative error -> negative output (symmetric).
    {
        PIDController pid(cfg);
        const float u = pid.compute(-0.5f, 0.01f, 10.0f);
        report(approxEq(u, -1.005f, 1e-3f), "pid: negative error");
    }

    // Output saturation.
    {
        PIDController pid(cfg);
        const float u = pid.compute(1.0f, 0.01f, 0.5f);  // limit 0.5
        report(approxEq(u, 0.5f), "pid: output saturation");
    }

    // Error clamping: error beyond errorLimit behaves like errorLimit.
    {
        PIDController pid(cfg);
        const float uBig = pid.compute(5.0f, 0.01f, 100.0f);
        PIDController pid2(cfg);
        const float uRef = pid2.compute(1.0f, 0.01f, 100.0f);
        report(approxEq(uBig, uRef, 1e-3f), "pid: error clamp");
    }

    // Integral windup: sustained error must saturate the integral state.
    {
        PIDController pid(cfg);
        float u = 0;
        for (int i = 0; i < 10000; ++i) u = pid.compute(1.0f, 0.01f, 100.0f);
        report(pid.getIntegralState() <= 0.3f + 1e-4f && u <= 100.0f,
               "pid: integral anti-windup");
    }

    // Integral disable flag: output must not include the integral.
    {
        PIDController pid(cfg);
        pid.setIntegralEnabled(false);
        float u = 0;
        for (int i = 0; i < 1000; ++i) u = pid.compute(1.0f, 0.01f, 100.0f);
        report(approxEq(u, 2.0f, 1e-3f), "pid: integral disabled");
    }

    // Derivative with variable dt: with alpha=1 the filtered derivative
    // equals de/dt exactly, using the MEASURED dt each step.
    //   step 2: e 0->0.1 in 10 ms  => D = 0.1 * 10 = 1.0;  u2 = 0.2 + I + 1
    //   step 3: e 0.1->0.2 in 5 ms => D = 0.1 * 20 = 2.0; u3 = 0.4 + I + 2
    {
        PIDController pid(cfg);
        pid.compute(0.0f, 0.01f, 100.0f);                     // first step: D=0
        const float u2 = pid.compute(0.1f, 0.01f, 100.0f);    // d = 10 /s
        const float u3 = pid.compute(0.2f, 0.005f, 100.0f);   // d = 20 /s
        report(approxEq(u2, 2.0f * 0.1f + 1.0f * 0.1f * 0.01f + 0.1f * 10.0f, 1e-3f) &&
               approxEq(u3, 2.0f * 0.2f + 1.0f * 0.2f * 0.005f + 0.1f * 20.0f, 1e-3f),
               "pid: derivative uses measured dt");
    }

    // Non-positive dt: must not explode.
    {
        PIDController pid(cfg);
        pid.compute(0.5f, 0.01f, 100.0f);
        const float u = pid.compute(-0.5f, 0.0f, 100.0f);
        report(u <= 100.0f && u >= -100.0f, "pid: zero dt safe");
    }
}

// ---------------------------------------------------------------------------
// PointKinematics
// ---------------------------------------------------------------------------

static void testPointKinematics() {
    PointKinematics kin(0.08f);  // l = 8 cm

    // Body frame: pure forward point velocity -> v, omega=0.
    {
        Twist2D t = kin.bodyFrameTwist({0.3f, 0.0f});
        report(approxEq(t.v, 0.3f) && approxEq(t.omega, 0.0f),
               "kin: body forward only");
    }

    // Body frame: lateral point velocity -> omega = u_l / l.
    {
        Twist2D t = kin.bodyFrameTwist({0.0f, 0.4f});
        report(approxEq(t.v, 0.0f) && approxEq(t.omega, 0.4f / 0.08f),
               "kin: body lateral -> omega = u/l");
    }

    // World frame at theta=0: v = ux, omega = uy / l.
    {
        Twist2D t = kin.worldFrameTwist({0.3f, 0.4f}, 0.0f);
        report(approxEq(t.v, 0.3f) && approxEq(t.omega, 0.4f / 0.08f),
               "kin: world frame theta=0");
    }

    // World frame at theta=pi/2: forward is -y_world... verify the matrix:
    // v = cos(th)*ux + sin(th)*uy, omega = (-sin(th)*ux + cos(th)*uy)/l.
    {
        const float th = 1.5707963268f;
        Twist2D t = kin.worldFrameTwist({1.0f, 0.0f}, th);
        report(approxEq(t.v, 0.0f, 1e-3f) && approxEq(t.omega, -1.0f / 0.08f, 1e-2f),
               "kin: world frame theta=pi/2");
    }

    // Degenerate l <= 0: no rotation, no crash.
    {
        PointKinematics bad(0.0f);
        Twist2D t = bad.bodyFrameTwist({0.3f, 0.4f});
        report(approxEq(t.v, 0.3f) && approxEq(t.omega, 0.0f),
               "kin: l=0 fails safe");
    }
}

// ---------------------------------------------------------------------------
// DifferentialDrive mixing (pure math, no hardware)
// ---------------------------------------------------------------------------

static void testMixing() {
    const float b = 0.15f;

    // Straight: both wheels equal.
    {
        WheelSpeeds w = DifferentialDrive::mixWheelSpeedsPure(0.3f, 0.0f, b);
        report(approxEq(w.left, 0.3f) && approxEq(w.right, 0.3f),
               "mix: straight");
    }

    // Pure rotation (v=0, omega>0 CCW): left wheel backwards, right forwards.
    {
        WheelSpeeds w = DifferentialDrive::mixWheelSpeedsPure(0.0f, 2.0f, b);
        report(approxEq(w.left, -0.15f) && approxEq(w.right, 0.15f),
               "mix: pure CCW rotation");
    }

    // General case: v_L = v - b/2*omega, v_R = v + b/2*omega.
    {
        WheelSpeeds w = DifferentialDrive::mixWheelSpeedsPure(0.4f, 3.0f, b);
        report(approxEq(w.left, 0.4f - 0.075f * 3.0f) &&
               approxEq(w.right, 0.4f + 0.075f * 3.0f),
               "mix: general twist");
    }

    // Inverse consistency: wheels -> twist -> wheels.
    {
        WheelSpeeds w = DifferentialDrive::mixWheelSpeedsPure(0.4f, 3.0f, b);
        Twist2D t = DifferentialDrive::wheelsToTwist(w, b);
        WheelSpeeds w2 = DifferentialDrive::mixWheelSpeedsPure(t.v, t.omega, b);
        report(approxEq(w.left, w2.left) && approxEq(w.right, w2.right),
               "mix: inverse consistency");
    }
}

// ---------------------------------------------------------------------------
// CalibrationMetrics
// ---------------------------------------------------------------------------

static void testMetrics() {
    // Constant error 0.5 for 100 samples -> Ee = 0.25, eRMS = 0.5.
    {
        CalibrationMetrics m;
        m.beginWindow();
        for (int i = 0; i < 100; ++i) m.onFastLoopSample(0.5f, true, 0.0f, false);
        MetricsWindow w = m.endWindow(1.0f);
        report(approxEq(w.meanSquareError, 0.25f, 1e-3f) &&
               approxEq(w.rmsError, 0.5f, 1e-3f),
               "metrics: constant error MSE/RMS");
    }

    // Line lost half the time -> lostFraction 0.5.
    {
        CalibrationMetrics m;
        m.beginWindow();
        for (int i = 0; i < 100; ++i)
            m.onFastLoopSample(0.0f, (i % 2) == 0, 0.0f, false);
        MetricsWindow w = m.endWindow(1.0f);
        report(approxEq(w.lineLostFraction, 0.5f, 1e-3f),
               "metrics: line-lost fraction");
    }

    // Sinusoidal omega -> RMS = amplitude/sqrt(2); zero crossings: the
    // 1 Hz sine starts AT zero (first sample seeds the sign tracker, no
    // crossing) and crosses once more at t=0.5 s => rate = 1 /s.
    {
        CalibrationMetrics m;
        m.beginWindow();
        // omega = sin(2*pi*1*t), 200 samples over 1 s -> 1 Hz oscillation.
        for (int i = 0; i < 200; ++i) {
            const float t = i * 0.005f;
            const float om = sinf(6.2831853f * 1.0f * t);
            m.onFastLoopSample(0.0f, true, om, true);
        }
        MetricsWindow w = m.endWindow(1.0f);
        report(approxEq(w.omegaRms, 1.0f / 1.41421356f, 0.01f) &&
               w.zeroCrossingRate > 0.5f && w.zeroCrossingRate < 1.5f,
               "metrics: oscillation RMS + zero crossings");
    }
}

// ---------------------------------------------------------------------------
// PIDCalibrator (via a mock target; deterministic virtual clock)
// ---------------------------------------------------------------------------

/// Stands in for the PathController: records gain writes and fabricates
/// the observable state the calibrator reads.
class MockCalibrationTarget : public CalibrationTarget {
public:
    PIDGains gains = PIDGains(2.0f, 0.0f, 0.1f);
    float lineError = 0.0f;
    bool  lineLost  = false;
    int   gainWrites = 0;

    void setGains(const PIDGains& g) override { gains = g; ++gainWrites; }
    const PIDGains& getGains() const override { return gains; }
    float getLastLineError() const override { return lineError; }
    bool  isLineLost() const override { return lineLost; }
};

/// Drives the calibrator's slow loop + fast feed with a virtual clock.
static void pump(PIDCalibrator& cal, MockCalibrationTarget& tgt,
                 uint32_t& nowUs, uint32_t durationUs,
                 float lineError, bool lineValid) {
    const uint32_t stepUs = 5000;             // fast loop period
    uint32_t elapsed = 0;
    while (elapsed < durationUs) {
        tgt.lineError = lineError;
        tgt.lineLost = !lineValid;
        cal.onFastLoopSample(nowUs);
        // The supervisor runs when its period elapses; update() is called
        // with the virtual clock too.
        if (cal.shouldRun(nowUs)) cal.update(nowUs);
        nowUs += stepUs;
        elapsed += stepUs;
    }
}

static void testCalibrator() {
    // --- Candidate ACCEPT: lower kp on a noisy-high-kp baseline improves
    //     the objective when the error is driven by oscillation.
    {
        MockCalibrationTarget tgt;
        CalibrationConfig cfg;
        cfg.updatePeriodUs = 100000;      // run the supervisor often enough
        cfg.evaluationWindowS = 0.2f;    // short windows for the test
        cfg.timeoutS = 100.0f;
        PIDCalibrator cal(tgt, nullptr, cfg);

        // Scenario: with the baseline kp the (fabricated) error is large;
        // any accepted candidate reduces the fabricated error.
        // We emulate "better gains -> better tracking" by making the
        // error depend on the target's live gains.
        auto scenarioPump = [&](MockCalibrationTarget& t, PIDCalibrator& c,
                                uint32_t& now) {
            const uint32_t stepUs = 5000;
            for (uint32_t e = 0; e < (uint32_t)(cfg.evaluationWindowS * 1e6f); e += stepUs) {
                // Baseline kp=2 tracks badly (error 0.8); reduced kp tracks
                // better (error 0.2). Fabricate accordingly.
                t.lineError = (t.gains.kp > 1.5f) ? 0.8f : 0.2f;
                t.lineLost = false;
                c.onFastLoopSample(now);
                if (c.shouldRun(now)) c.update(now);
                now += stepUs;
            }
            // Let the supervisor close the window.
            uint32_t extra = 0;
            while (extra < 2 * cfg.updatePeriodUs) {
                t.lineError = (t.gains.kp > 1.5f) ? 0.8f : 0.2f;
                t.lineLost = false;
                c.onFastLoopSample(now);
                if (c.shouldRun(now)) c.update(now);
                now += stepUs;
                extra += stepUs;
            }
        };

        cal.enable();
        uint32_t now = micros() + 1000000;
        // Baseline window.
        scenarioPump(tgt, cal, now);
        // First candidate window.
        scenarioPump(tgt, cal, now);
        // Accept/reject happens inside update; give it a couple of cycles.
        scenarioPump(tgt, cal, now);
        scenarioPump(tgt, cal, now);

        report(tgt.gains.kp < 2.0f, "calibrator: improving candidate accepted");
        report(cal.getState() != CalibrationState::CAL_DISABLED &&
               cal.getCurrentGains().kp < 2.0f,
               "calibrator: still running after accept");
    }

    // --- Candidate REJECT: a candidate that worsens tracking is reverted.
    {
        MockCalibrationTarget tgt;
        CalibrationConfig cfg;
        cfg.updatePeriodUs = 100000;
        cfg.evaluationWindowS = 0.2f;
        cfg.timeoutS = 100.0f;
        PIDCalibrator cal(tgt, nullptr, cfg);

        cal.enable();
        uint32_t now = micros() + 1000000;

        // Baseline window: small error (good tracking).
        pump(cal, tgt, now, (uint32_t)(cfg.evaluationWindowS * 1e6f) + 2 * cfg.updatePeriodUs,
             0.1f, true);
        // Candidate window: WORSE error for any candidate (fabricated).
        pump(cal, tgt, now, (uint32_t)(cfg.evaluationWindowS * 1e6f) + 2 * cfg.updatePeriodUs,
             0.9f, true);
        pump(cal, tgt, now, 3 * cfg.updatePeriodUs, 0.1f, true);

        // The REJECTED candidate must never be committed: currentGains
        // stays at the baseline. (tgt.gains may briefly hold the NEXT
        // candidate under test — that is by design.)
        report(approxEq(cal.getCurrentGains().kp, 2.0f, 1e-3f),
               "calibrator: worsening candidate rejected/reverted");
    }

    // --- Failure: line lost -> revert to last-known-good + disable.
    {
        MockCalibrationTarget tgt;
        CalibrationConfig cfg;
        cfg.updatePeriodUs = 100000;
        cfg.evaluationWindowS = 0.2f;
        cfg.lineLossAbortS = 0.3f;
        cfg.timeoutS = 100.0f;
        PIDCalibrator cal(tgt, nullptr, cfg);

        cal.enable();
        uint32_t now = micros() + 1000000;
        // Some valid samples first so lastLineSeen is fresh.
        pump(cal, tgt, now, 100000, 0.2f, true);
        // Then lose the line for longer than the abort threshold.
        pump(cal, tgt, now, 1000000, 0.0f, false);

        report(cal.getState() == CalibrationState::CAL_DISABLED,
               "calibrator: line loss disables supervisor");
        report(approxEq(tgt.gains.kp, 2.0f, 1e-3f),
               "calibrator: reverts to last-known-good on failure");
    }

    // --- Timeout: supervisor disables itself and reverts.
    {
        MockCalibrationTarget tgt;
        CalibrationConfig cfg;
        cfg.updatePeriodUs = 100000;
        cfg.evaluationWindowS = 0.2f;
        cfg.timeoutS = 1.0f;   // very short for the test
        PIDCalibrator cal(tgt, nullptr, cfg);

        cal.enable();
        uint32_t now = micros() + 1000000;
        pump(cal, tgt, now, 2500000, 0.2f, true);  // > timeout

        report(cal.getState() == CalibrationState::CAL_DISABLED,
               "calibrator: timeout disables supervisor");
        report(approxEq(tgt.gains.kp, 2.0f, 1e-3f),
               "calibrator: timeout reverts to last-known-good");
    }

    // --- Hard bounds: clamped candidates never exceed the limits.
    {
        MockCalibrationTarget tgt;
        CalibrationConfig cfg;
        cfg.updatePeriodUs = 100000;
        cfg.evaluationWindowS = 0.2f;
        cfg.timeoutS = 100.0f;
        cfg.limits.kpMax = 3.0f;
        PIDCalibrator cal(tgt, nullptr, cfg);

        cal.enable();
        uint32_t now = micros() + 1000000;
        for (int round = 0; round < 20; ++round) {
            pump(cal, tgt, now, (uint32_t)(cfg.evaluationWindowS * 1e6f) + 2 * cfg.updatePeriodUs,
                 0.3f, true);
        }
        report(tgt.gains.kp <= 3.0f + 1e-4f && tgt.gains.kp >= cfg.limits.kpMin - 1e-4f,
               "calibrator: gains stay inside hard bounds");
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int runAll() {
    g_failures = 0;
    testPID();
    testPointKinematics();
    testMixing();
    testMetrics();
    testCalibrator();
#ifdef ARDUINO
    Serial.print(g_failures == 0 ? F("SELF-TEST: ALL PASS (")
                                 : F("SELF-TEST: FAILURES ("));
    Serial.print(g_failures);
    Serial.println(F(")"));
#else
    printf("SELF-TEST: %s (%d failures)\n",
           g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
#endif
    return g_failures;
}

}  // namespace selftest
