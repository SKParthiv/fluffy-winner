/**
 * Buttons.h
 * =========
 * Four physical buttons: UP / DOWN / SELECT / BACK.
 *
 * Wiring assumption (TODO: VERIFY): buttons connect the GPIO to GND and
 * rely on the ESP32 internal pull-up, so PRESSED = LOW. If the physical
 * buttons are wired to 3.3 V with a pull-down, change activeLevel below.
 *
 * Features:
 *   - Debounce: a level must be stable for DEBOUNCE_MS to count.
 *   - Edge events (pressed once) for menu navigation.
 *   - Auto-repeat on UP/DOWN after ~600 ms held (400 ms period) so
 *     long value edits do not require 200 button presses.
 *   - SELECT + BACK held together ~3 s => hard-reset event (the MENU
 *     layer performs the actual ESP.restart(); this class only reports).
 *
 * All pins are TODO placeholders until the wiring is confirmed; with
 * unassigned pins the buttons simply never fire — the robot runs normally.
 */

#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>
#include "../config/PinConfig.h"

enum class ButtonEvent {
    NONE,
    UP_PRESSED,
    DOWN_PRESSED,
    SELECT_PRESSED,
    BACK_PRESSED,
    UP_REPEAT,      ///< auto-repeat while UP held
    DOWN_REPEAT,    ///< auto-repeat while DOWN held
    RESET_REQUEST   ///< SELECT + BACK held ~3 s (hard reset combo)
};

class Buttons {
public:
    explicit Buttons(const ButtonPinConfig& pins);

    /// Configure GPIOs (pull-ups). Call once in setup().
    void begin();

    /// Poll once per loop() pass. Non-blocking.
    void update();

    /// Consume one event (call until NONE in each UI tick).
    ButtonEvent pollEvent();

    /// True while SELECT and BACK are both held (for live UI feedback).
    bool resetComboActive() const;

    static const uint16_t DEBOUNCE_MS = 25;
    static const uint16_t REPEAT_DELAY_MS = 600;  ///< before first repeat
    static const uint16_t REPEAT_PERIOD_MS = 400; ///< between repeats
    static const uint16_t RESET_HOLD_MS = 3000;   ///< SELECT+BACK combo

private:
    struct Btn {
        uint8_t pin;
        bool stableLevel;        ///< debounced level (true = released)
        bool lastRawLevel;
        uint32_t lastChangeMs;
        uint32_t pressedSinceMs;
        uint32_t lastRepeatMs;
        bool repeatArmed;
    };

    void serviceButton(Btn& b, uint32_t nowMs);
    bool readLevel(const Btn& b) const;

    ButtonPinConfig pins_;
    Btn up_, down_, select_, back_;
    bool resetComboFired_;
    bool resetComboActive_;
};

#endif // BUTTONS_H
