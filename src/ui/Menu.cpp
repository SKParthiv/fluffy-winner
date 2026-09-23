/**
 * Menu.cpp — see header.
 *
 * Implementation notes:
 *  - Redraw is rate-limited to ~5 Hz; the fast control loop is untouched.
 *  - All drawing is defensive: if the display is absent (begin() failed)
 *    the menu still processes buttons so SERIAL control keeps working.
 *  - The state machine is a flat enum + a couple of indices: explicit,
 *    easy to audit, no framework.
 */

#include "Menu.h"
#include "../system/PersistentConfig.h"
#include <stdio.h>

#if CALIBRATION_ENABLED
#include "../calibration/CalibrationState.h"
#endif

// Rows of the main menu (order matters — indices are persistent state).
static const int MAIN_AUTO = 0;
static const int MAIN_MANUAL = 1;
static const int MAIN_PID = 2;
static const int MAIN_SYSTEM = 3;
static const int MAIN_RESET = 4;
static const int MAIN_COUNT = 5;

// Rows of the PID/parameter menu.
static const int PARAM_KP = 0;
static const int PARAM_KI = 1;
static const int PARAM_KD = 2;
static const int PARAM_INTEGRAL = 3;
static const int PARAM_TARGET = 4;
static const int PARAM_VFWD = 5;
static const int PARAM_COUNT = 6;

Menu::Menu(Display& display, Buttons& buttons,
           PathController& controller, RLS08LineSensor& sensor,
           RobotConfig& cfg
#if CALIBRATION_ENABLED
           , PIDCalibrator* calibrator
#endif
    )
    : display_(display), buttons_(buttons), controller_(controller),
      sensor_(sensor), cfg_(cfg)
#if CALIBRATION_ENABLED
      , calibrator_(calibrator)
#endif
      ,
      screen_(Screen::MAIN_MENU),
      mainSel_(0), pidSel_(0), editIdx_(0),
      editValue_(0.0f), savedValue_(0.0f),
      resetSel_(false), lastDrawMs_(0) {}

void Menu::begin() {
    enterScreen(Screen::MAIN_MENU);
}

// ---------------------------------------------------------------------------
// Event handling
// ---------------------------------------------------------------------------

void Menu::update() {
    buttons_.update();

    // The hard-reset combo has priority from ANY screen. It actually
    // restarts the controller (not a config restore).
    for (ButtonEvent ev = buttons_.pollEvent(); ev != ButtonEvent::NONE;
         ev = buttons_.pollEvent()) {
        if (ev == ButtonEvent::RESET_REQUEST) {
            // Brief motor-safe state before rebooting: the OFF gate is the
            // safest place to leave the hardware during a reset.
            controller_.setEnabled(false);
            display_.beginFrame();
            display_.textBig(16, 30, "RESETTING...");
            display_.endFrame();
            delay(100);          // let the frame reach the panel
            ESP.restart();       // ESP32 software reset
        }
        handleEvent(ev);
    }

    // Rate-limited redraw (~5 Hz) keeps the loop free for control.
    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastDrawMs_) >= 200) {
        lastDrawMs_ = nowMs;
        draw();
    }
}

void Menu::handleEvent(ButtonEvent ev) {
    // UP/DOWN behave the same for PRESSED and REPEAT (value editing).
    const bool up = (ev == ButtonEvent::UP_PRESSED || ev == ButtonEvent::UP_REPEAT);
    const bool down = (ev == ButtonEvent::DOWN_PRESSED || ev == ButtonEvent::DOWN_REPEAT);

    switch (screen_) {
        case Screen::MAIN_MENU:
            if (up)   mainSel_ = (mainSel_ + MAIN_COUNT - 1) % MAIN_COUNT;
            if (down) mainSel_ = (mainSel_ + 1) % MAIN_COUNT;
            if (ev == ButtonEvent::SELECT_PRESSED) {
                switch (mainSel_) {
                    case MAIN_AUTO:    enterScreen(Screen::AUTO_MODE); break;
                    case MAIN_MANUAL:  enterScreen(Screen::MANUAL_MODE); break;
                    case MAIN_PID:      enterScreen(Screen::PID_MENU); break;
                    case MAIN_SYSTEM:  enterScreen(Screen::SYSTEM_TOGGLE); break;
                    case MAIN_RESET:   resetSel_ = false;
                                       enterScreen(Screen::RESET_CONFIRM); break;
                }
            }
            break;

        case Screen::AUTO_MODE:
#if CALIBRATION_ENABLED
            if (ev == ButtonEvent::SELECT_PRESSED && calibrator_ != nullptr) {
                if (calibrator_->isEnabled()) {
                    calibrator_->disable();   // stop + restore last-known-good
                } else {
                    calibrator_->enable();     // (re)start calibration
                }
            }
#endif
            if (ev == ButtonEvent::BACK_PRESSED) enterScreen(Screen::MAIN_MENU);
            break;

        case Screen::MANUAL_MODE:
            // Live sensor view; SELECT toggles the polarity flag so the
            // user can verify channel polarity interactively (a MANUAL
            // calibration aid — the change is runtime-only, never saved).
            if (ev == ButtonEvent::SELECT_PRESSED) {
                sensor_.setLineIsHigh(!sensor_.getLineIsHigh());
            }
            if (ev == ButtonEvent::BACK_PRESSED) enterScreen(Screen::MAIN_MENU);
            break;

        case Screen::PID_MENU:
            if (up)   pidSel_ = (pidSel_ + PARAM_COUNT - 1) % PARAM_COUNT;
            if (down) pidSel_ = (pidSel_ + 1) % PARAM_COUNT;
            if (ev == ButtonEvent::SELECT_PRESSED) {
                editIdx_ = pidSel_;
                editValue_ = readParam(editIdx_);
                savedValue_ = editValue_;
                enterScreen(Screen::VALUE_EDIT);
            }
            if (ev == ButtonEvent::BACK_PRESSED) enterScreen(Screen::MAIN_MENU);
            break;

        case Screen::VALUE_EDIT:
            if (up)   editValue_ += paramStep(editIdx_);
            if (down) editValue_ -= paramStep(editIdx_);
            if (ev == ButtonEvent::SELECT_PRESSED) {
                applyPendingValue();           // confirm -> apply + persist
                enterScreen(Screen::PID_MENU);
            }
            if (ev == ButtonEvent::BACK_PRESSED) {
                cancelPendingValue();           // cancel -> restore
                enterScreen(Screen::PID_MENU);
            }
            break;

        case Screen::SYSTEM_TOGGLE:
            if (up || down) {
                // UP/DOWN flips the system ON/OFF state; explicit and
                // visible on screen before anything changes.
                controller_.setEnabled(!controller_.isEnabled());
            }
            if (ev == ButtonEvent::SELECT_PRESSED ||
                ev == ButtonEvent::BACK_PRESSED) {
                enterScreen(Screen::MAIN_MENU);
            }
            break;

        case Screen::RESET_CONFIRM:
            if (up || down) resetSel_ = !resetSel_;
            if (ev == ButtonEvent::SELECT_PRESSED) {
                if (resetSel_) {
                    // Confirmed: erase saved config, stop motors, reboot so
                    // every module comes back with compiled-in defaults.
                    controller_.setEnabled(false);
                    PersistentConfig::clear();
                    display_.beginFrame();
                    display_.textBig(10, 30, "CONFIG CLEARED");
                    display_.endFrame();
                    delay(100);
                    ESP.restart();
                } else {
                    enterScreen(Screen::MAIN_MENU);   // Cancel
                }
            }
            if (ev == ButtonEvent::BACK_PRESSED) enterScreen(Screen::MAIN_MENU);
            break;
    }
}

void Menu::enterScreen(Screen s) {
    screen_ = s;
    draw();   // immediate feedback on transitions
}

// ---------------------------------------------------------------------------
// Parameter read/write (single source of truth: RobotConfig + controller)
// ---------------------------------------------------------------------------

const char* Menu::paramName(int idx) const {
    switch (idx) {
        case PARAM_KP:        return "Kp";
        case PARAM_KI:        return "Ki";
        case PARAM_KD:        return "Kd";
        case PARAM_INTEGRAL:  return "Integral";
        case PARAM_TARGET:    return "Target pos";
        case PARAM_VFWD:      return "Fwd speed";
    }
    return "?";
}

float Menu::readParam(int idx) const {
    switch (idx) {
        case PARAM_KP:       return cfg_.controller.gains.kp;
        case PARAM_KI:       return cfg_.controller.gains.ki;
        case PARAM_KD:       return cfg_.controller.gains.kd;
        case PARAM_INTEGRAL: return cfg_.controller.integralEnabled ? 1.0f : 0.0f;
        case PARAM_TARGET:   return cfg_.controller.targetPosition;
        case PARAM_VFWD:     return cfg_.controller.forwardVelocity;
    }
    return 0.0f;
}

void Menu::writeParam(int idx, float value) {
    switch (idx) {
        case PARAM_KP:       cfg_.controller.gains.kp = value; break;
        case PARAM_KI:       cfg_.controller.gains.ki = value; break;
        case PARAM_KD:       cfg_.controller.gains.kd = value; break;
        case PARAM_INTEGRAL: cfg_.controller.integralEnabled = (value > 0.5f); break;
        case PARAM_TARGET:   cfg_.controller.targetPosition = value; break;
        case PARAM_VFWD:     cfg_.controller.forwardVelocity = value; break;
    }
}

float Menu::paramStep(int idx) const {
    // Steps chosen so a handful of presses covers the useful range.
    switch (idx) {
        case PARAM_KP:       return 0.1f;
        case PARAM_KI:       return 0.05f;
        case PARAM_KD:       return 0.05f;
        case PARAM_INTEGRAL: return 1.0f;    // toggles 0/1
        case PARAM_TARGET:   return 0.05f;   // normalised position units
        case PARAM_VFWD:     return 0.05f;   // m/s
    }
    return 0.0f;
}

void Menu::applyPendingValue() {
    // Integral flag is a 0/1 toggle: clamp the edited value.
    if (editIdx_ == PARAM_INTEGRAL) {
        editValue_ = (editValue_ > 0.5f) ? 1.0f : 0.0f;
    }
    if (editIdx_ == PARAM_TARGET) {   // normalised position stays in [-1, 1]
        if (editValue_ >  1.0f) editValue_ =  1.0f;
        if (editValue_ < -1.0f) editValue_ = -1.0f;
    }
    if (editIdx_ == PARAM_VFWD) {     // cruise speed stays sane
        if (editValue_ < 0.0f) editValue_ = 0.0f;
        if (editValue_ > cfg_.controller.maxLinearVelocity)
            editValue_ = cfg_.controller.maxLinearVelocity;
    }

    writeParam(editIdx_, editValue_);

    // Push to the RUNNING controller immediately (live tuning), then
    // persist — but ONLY here, on the user's explicit confirmation.
    controller_.setGains(cfg_.controller.gains);
    controller_.setIntegralEnabled(cfg_.controller.integralEnabled);
    PersistentConfig::save(cfg_);
}

void Menu::cancelPendingValue() {
    writeParam(editIdx_, savedValue_);   // restore pre-edit value
    controller_.setGains(cfg_.controller.gains);
    controller_.setIntegralEnabled(cfg_.controller.integralEnabled);
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

void Menu::draw() {
    display_.beginFrame();
    switch (screen_) {
        case Screen::MAIN_MENU:     drawMainMenu(); break;
        case Screen::AUTO_MODE:     drawAutoMode(); break;
        case Screen::MANUAL_MODE:   drawManualMode(); break;
        case Screen::PID_MENU:      drawPidMenu(); break;
        case Screen::VALUE_EDIT:    drawValueEdit(); break;
        case Screen::SYSTEM_TOGGLE: drawSystemToggle(); break;
        case Screen::RESET_CONFIRM: drawResetConfirm(); break;
    }
    display_.endFrame();
}

void Menu::drawMainMenu() {
    static const char* rows[MAIN_COUNT] = {
        "AUTO MODE", "MANUAL MODE", "PID / PARAMS", "SYSTEM", "RESET"
    };
    display_.text(0, 12, "== FLUFFY WINNER ==");
    for (int i = 0; i < MAIN_COUNT; ++i) {
        const int y = 24 + i * 9;
        if (i == mainSel_) display_.textInverted(2, y, rows[i]);
        else               display_.text(2, y, rows[i]);
    }
}

void Menu::drawAutoMode() {
    display_.text(0, 12, "AUTO MODE");
    display_.hline(14);

#if CALIBRATION_ENABLED
    if (calibrator_ == nullptr) {
        display_.text(2, 26, "calibrator n/a");
        return;
    }
    const CalibrationState st = calibrator_->getState();
    char buf[24];

    display_.text(2, 26, st == CalibrationState::CAL_DISABLED
                        ? "status: OFF"
                        : "status: RUNNING");
    snprintf(buf, sizeof(buf), "state: %s", calibrationStateName(st));
    display_.text(2, 36, buf);

    const PIDGains& g = controller_.getGains();
    snprintf(buf, sizeof(buf), "kp=%.2f ki=%.2f kd=%.2f",
             (double)g.kp, (double)g.ki, (double)g.kd);
    display_.text(2, 46, buf);

    display_.text(2, 62, calibrator_->isEnabled()
                        ? "SEL: stop  BACK: menu"
                        : "SEL: start BACK: menu");
#else
    display_.text(2, 26, "calibration compiled");
    display_.text(2, 36, "out (see README)");
    display_.text(2, 52, "BACK: menu");
#endif
}

void Menu::drawManualMode() {
    display_.text(0, 12, "MANUAL MODE");
    display_.hline(14);

    // Raw channel states, Sensor 1 (rightmost) on the RIGHT of the row —
    // matching how the array sits on the robot when viewed from above.
    bool raw[RLS08LineSensor::NUM_CHANNELS];
    sensor_.readRaw(raw);
    char buf[26];

    // Print Sensor 8..1 left-to-right so the display mirrors the robot.
    int p = 0;
    buf[p++] = 'S';
    buf[p++] = '8';
    buf[p++] = ':';
    for (int i = RLS08LineSensor::NUM_CHANNELS - 1; i >= 0; --i) {
        buf[p++] = raw[i] ? '1' : (sensor_.assignedChannelCount() == 8 || i < sensor_.assignedChannelCount() ? '0' : '-');
    }
    buf[p] = '\0';
    display_.text(2, 26, buf);

    const LineMeasurement m = sensor_.getMeasurement();
    snprintf(buf, sizeof(buf), "pos=%+.2f  %s",
             (double)m.position, m.valid ? "OK" : "LOST");
    display_.text(2, 38, buf);
    snprintf(buf, sizeof(buf), "err=%+.2f",
             (double)(m.position - cfg_.controller.targetPosition));
    display_.text(2, 48, buf);

    snprintf(buf, sizeof(buf), "pol: HIGH=%sline",
             sensor_.getLineIsHigh() ? "" : "no ");
    display_.text(2, 62, buf);
}

void Menu::drawPidMenu() {
    display_.text(0, 12, "PID / PARAMETERS");
    display_.hline(14);
    char buf[24];
    for (int i = 0; i < PARAM_COUNT; ++i) {
        const int y = 24 + i * 8;
        if (i == PARAM_INTEGRAL) {
            snprintf(buf, sizeof(buf), "Integral: %s",
                     cfg_.controller.integralEnabled ? "ON" : "OFF");
        } else {
            snprintf(buf, sizeof(buf), "%s: %.3f", paramName(i),
                     (double)readParam(i));
        }
        if (i == pidSel_) display_.textInverted(2, y, buf);
        else              display_.text(2, y, buf);
    }
}

void Menu::drawValueEdit() {
    display_.text(0, 12, "EDIT PARAMETER");
    display_.hline(14);
    char buf[24];
    snprintf(buf, sizeof(buf), "%s", paramName(editIdx_));
    display_.text(2, 30, buf);
    snprintf(buf, sizeof(buf), "value: %.3f", (double)editValue_);
    display_.textBig(10, 52, buf);
    display_.text(2, 63, "UP/DOWN edit");
    display_.text(66, 63, "SEL ok BACK no");
}

void Menu::drawSystemToggle() {
    display_.text(0, 12, "SYSTEM");
    display_.hline(14);
    const bool on = controller_.isEnabled();
    display_.textBig(20, 40, on ? "SYSTEM: ON" : "SYSTEM: OFF");
    display_.text(2, 63, "UP/DOWN toggle  BACK: menu");
}

void Menu::drawResetConfirm() {
    display_.text(0, 12, "RESET CONFIG?");
    display_.hline(14);
    display_.text(2, 28, "Clears saved PID and");
    display_.text(2, 38, "calibration values,");
    display_.text(2, 48, "then reboots.");
    if (resetSel_) display_.textInverted(2, 62, "CONFIRM");
    else           display_.text(2, 62, "Cancel");
    display_.text(52, 62, "SEL: choose");
}
