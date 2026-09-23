/**
 * Menu.h
 * ======
 * OLED + 4-button menu system (UP / DOWN / SELECT / BACK).
 *
 * Structure (kept deliberately simple for a 128x64 panel):
 *
 *   MAIN MENU
 *   ├── AUTO MODE    — autonomous calibration status / start / restart
 *   ├── MANUAL MODE  — live sensor values + sensor polarity/order toggles
 *   ├── PID/PARAMS   — edit kp, ki, kd, integral on/off, target, v_fwd
 *   ├── SYSTEM       — line-following ON/OFF (motors safely stopped when OFF)
 *   └── RESET        — clear saved config (factory defaults on next boot)
 *
 * Hard reset (SELECT + BACK held ~3 s) works from ANY screen and actually
 * restarts the ESP32 (ESP.restart()) — it is NOT the same as RESET above,
 * which only clears saved configuration.
 *
 * Persistence rule: parameter edits are applied to the running controller
 * immediately (so they can be tried live) but are written to NVS ONLY when
 * the user confirms with SELECT on the edit screen. BACK cancels the edit
 * and restores the pre-edit value.
 *
 * Calibration rule: the menu never makes calibration a boot dependency.
 * With CALIBRATION_ENABLED=false the AUTO MODE screen simply says so.
 */

#ifndef MENU_H
#define MENU_H

#include <Arduino.h>
#include "Display.h"
#include "Buttons.h"
#include "../config/RobotConfig.h"
#include "../control/PathController.h"
#include "../sensors/RLS08LineSensor.h"

#if CALIBRATION_ENABLED
#include "../calibration/PIDCalibrator.h"
#endif

class Menu {
public:
    Menu(Display& display, Buttons& buttons,
         PathController& controller, RLS08LineSensor& sensor,
         RobotConfig& cfg
#if CALIBRATION_ENABLED
         , PIDCalibrator* calibrator
#endif
    );

    void begin();
    /// Call once per loop() pass: consumes button events, redraws at ~5 Hz.
    void update();

private:
    // ---- screens --------------------------------------------------------
    enum class Screen {
        MAIN_MENU,
        AUTO_MODE,
        MANUAL_MODE,
        PID_MENU,        ///< parameter selection list
        VALUE_EDIT,      ///< UP/DOWN changes the value, SELECT saves, BACK cancels
        SYSTEM_TOGGLE,
        RESET_CONFIRM
    };

    // ---- helpers -------------------------------------------------------
    void handleEvent(ButtonEvent ev);
    void draw();
    void drawMainMenu();
    void drawAutoMode();
    void drawManualMode();
    void drawPidMenu();
    void drawValueEdit();
    void drawSystemToggle();
    void drawResetConfirm();

    void enterScreen(Screen s);
    void applyPendingValue();
    void cancelPendingValue();

    const char* paramName(int idx) const;
    float readParam(int idx) const;
    void  writeParam(int idx, float value);
    /// Step size for UP/DOWN per parameter (kept practical for OLED edits).
    float paramStep(int idx) const;

    Display&    display_;
    Buttons&    buttons_;
    PathController& controller_;
    RLS08LineSensor& sensor_;
    RobotConfig& cfg_;
#if CALIBRATION_ENABLED
    PIDCalibrator* calibrator_;
#endif

    Screen  screen_;
    int     mainSel_;      ///< selected row in MAIN_MENU
    int     pidSel_;       ///< selected row in PID_MENU
    int     editIdx_;      ///< which parameter VALUE_EDIT is editing
    float   editValue_;    ///< live value being edited
    float   savedValue_;   ///< value to restore on BACK (cancel)
    bool    resetSel_;     ///< RESET_CONFIRM: false=Cancel, true=Confirm
    uint32_t lastDrawMs_;
};

#endif // MENU_H
