/**
 * Diagnostics.cpp — all Serial I/O lives here (and only here).
 * The fast control loop never prints; this class is called from loop()
 * and internally rate-limits everything (task §7, §21).
 */

#include "Diagnostics.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <cctype>

Diagnostics::Diagnostics(const RobotConfig& cfg)
    : cfg_(cfg),
      controller_(nullptr),
      rls08_(nullptr),
      calibrator_(nullptr),
      lastPrintUs_(0),
      lineLen_(0) {}

void Diagnostics::attach(PathController* controller, RLS08LineSensor* rls08,
                        PIDCalibrator* calibrator) {
    controller_ = controller;
    rls08_ = rls08;
    calibrator_ = calibrator;
}

void Diagnostics::begin() {
    Serial.begin(cfg_.diagnostics.serialBaud);
    // Small delay ONLY here (setup-time, one-off) so a serial monitor can
    // attach. This is NOT in any control path.
    delay(200);
    Serial.println();
    Serial.println(F("=== ESP32 Line Follower - diagnostics ready ==="));
    printHelp();
}

void Diagnostics::update(uint32_t nowUs) {
    // ---- Command parsing (non-blocking) --------------------------------
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (lineLen_ > 0) {
                line_[lineLen_] = '\0';
                handleCommand(line_);
                lineLen_ = 0;
            }
        } else if (lineLen_ < sizeof(line_) - 1) {
            line_[lineLen_++] = (char)c;
        }
    }

    // ---- Rate-limited heartbeat ----------------------------------------
    if ((uint32_t)(nowUs - lastPrintUs_) >= cfg_.diagnostics.printPeriodUs) {
        lastPrintUs_ = nowUs;
        printStatus();
    }
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

void Diagnostics::handleCommand(const char* cmd) {
    // Case-insensitive compare helper (avoid strcasecmp portability issues).
    auto ieq = [](const char* a, const char* b) {
        while (*a && *b) {
            if (tolower((int)*a) != tolower((int)*b)) return false;
            ++a; ++b;
        }
        return *a == '\0' && *b == '\0';
    };

    if (ieq(cmd, "gains"))            printGains();
    else if (ieq(cmd, "metrics"))     printMetrics();
    else if (ieq(cmd, "sensor"))       printSensor();
    else if (ieq(cmd, "controller"))  printController();
    else if (ieq(cmd, "timing"))      printTiming();
    else if (ieq(cmd, "cal on")) {
#if CALIBRATION_ENABLED
        if (calibrator_ != nullptr) {
            calibrator_->enable();
            Serial.println(F("calibration: ENABLED"));
        }
#else
        Serial.println(F("calibration: compiled out (CALIBRATION_ENABLED=false)"));
#endif
    }
    else if (ieq(cmd, "cal off")) {
#if CALIBRATION_ENABLED
        if (calibrator_ != nullptr) {
            calibrator_->disable();
            Serial.println(F("calibration: DISABLED, last-known-good restored"));
        }
#else
        Serial.println(F("calibration: compiled out"));
#endif
    }
    else if (ieq(cmd, "cal reset")) {
#if CALIBRATION_ENABLED
        if (calibrator_ != nullptr) {
            calibrator_->reset();
            Serial.println(F("calibration: RESET"));
        }
#endif
    }
    else if (ieq(cmd, "cal restore")) {
        if (calibrator_ != nullptr) {
            calibrator_->restoreLastKnownGood();
            Serial.println(F("restored last-known-good gains"));
        }
    }
    else printHelp();
}

// ---------------------------------------------------------------------------
// Printers
// ---------------------------------------------------------------------------

void Diagnostics::printGains() {
    if (controller_ == nullptr) return;
    const PIDGains& g = controller_->getGains();
    char buf[96];
    snprintf(buf, sizeof(buf), "gains: kp=%.4f ki=%.4f kd=%.4f integral=%s",
             (double)g.kp, (double)g.ki, (double)g.kd,
             controller_->pid().isIntegralEnabled() ? "on" : "off");
    Serial.println(buf);
}

void Diagnostics::printMetrics() {
#if CALIBRATION_ENABLED
    if (calibrator_ == nullptr) { Serial.println(F("metrics: n/a")); return; }
    const MetricsWindow& w = calibrator_->getLastWindow();
    char buf[160];
    snprintf(buf, sizeof(buf),
             "metrics: N=%lu Ee=%.4f eRMS=%.4f peak|e|=%.3f p2p=%.3f | "
             "wRMS=%.3f wP2P=%.3f zc=%.2f/s lost=%.2f imu=%d",
             (unsigned long)w.sampleCount,
             (double)w.meanSquareError, (double)w.rmsError,
             (double)w.peakAbsError, (double)w.peakToPeakError,
             (double)w.omegaRms, (double)w.omegaPeakToPeak,
             (double)w.zeroCrossingRate, (double)w.lineLostFraction,
             w.imuDataPresent ? 1 : 0);
    Serial.println(buf);
    snprintf(buf, sizeof(buf),
             "objective: J(cur)=%.4f J(cand)=%.4f state=%s round=%lu",
             (double)calibrator_->getLastObjective(),
             (double)calibrator_->getCandidateObjective(),
             calibrationStateName(calibrator_->getState()),
             (unsigned long)calibrator_->getRoundCount());
    Serial.println(buf);
#else
    Serial.println(F("metrics: calibration compiled out"));
#endif
}

void Diagnostics::printSensor() {
    if (rls08_ == nullptr) return;
    bool raw[RLS08LineSensor::NUM_CHANNELS];
    rls08_->readRaw(raw);
    char buf[64];
    snprintf(buf, sizeof(buf), "sensor raw: ");
    for (int i = 0; i < RLS08LineSensor::NUM_CHANNELS; ++i) {
        // Leftmost channel first, matches the physical array.
        char one[4];
        snprintf(one, sizeof(one), "%d", raw[i] ? 1 : 0);
        strncat(buf, one, sizeof(buf) - strlen(buf) - 1);
    }
    Serial.println(buf);
    const LineMeasurement m = rls08_->getMeasurement();
    snprintf(buf, sizeof(buf), "sensor: pos=%.3f conf=%.2f valid=%d",
             (double)m.position, (double)m.confidence, m.valid ? 1 : 0);
    Serial.println(buf);
}

void Diagnostics::printController() {
    if (controller_ == nullptr) return;
    char buf[96];
    snprintf(buf, sizeof(buf),
             "controller: e=%.3f lost=%d v=%.3f m/s omega=%.3f rad/s",
             (double)controller_->getLastLineError(),
             controller_->isLineLost() ? 1 : 0,
             (double)controller_->getLastOutputV(),
             (double)controller_->getLastOutputOmega());
    Serial.println(buf);
}

void Diagnostics::printTiming() {
    if (controller_ == nullptr) return;
    char buf[128];
    snprintf(buf, sizeof(buf),
             "timing: target=%lu us last=%lu min=%lu max=%lu avg=%lu n=%lu",
             (unsigned long)cfg_.controller.controlLoopPeriodUs,
             (unsigned long)controller_->getLastLoopPeriodUs(),
             (unsigned long)controller_->getMinLoopPeriodUs(),
             (unsigned long)controller_->getMaxLoopPeriodUs(),
             (unsigned long)controller_->getAvgLoopPeriodUs(),
             (unsigned long)controller_->getLoopCount());
    Serial.println(buf);
}

void Diagnostics::printHelp() {
    Serial.println(F("commands: gains | metrics | sensor | controller | "
                     "timing | cal on | cal off | cal reset | cal restore | help"));
}

void Diagnostics::printStatus() {
    // One compact heartbeat line, 1 Hz by default.
    if (controller_ == nullptr) return;
    char buf[128];
    snprintf(buf, sizeof(buf),
             "st: e=%+.2f v=%+.2f w=%+.2f lost=%d avgT=%luus",
             (double)controller_->getLastLineError(),
             (double)controller_->getLastOutputV(),
             (double)controller_->getLastOutputOmega(),
             controller_->isLineLost() ? 1 : 0,
             (unsigned long)controller_->getAvgLoopPeriodUs());
    Serial.println(buf);
}
