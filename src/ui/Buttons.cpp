/**
 * Buttons.cpp — see header.
 *
 * Debounce model: sample the raw level every update(); a level change is
 * only accepted after it persists DEBOUNCE_MS. Events are emitted on the
 * accepted press EDGE (no repeat on SELECT/BACK — a held SELECT must not
 * re-trigger menu entries; UP/DOWN get explicit auto-repeat for value
 * editing).
 *
 * Reset combo: SELECT and BACK are edge-triggered individually (so normal
 * menu use never fires it), but when BOTH stay held for RESET_HOLD_MS a
 * single RESET_REQUEST event is produced. Requiring both buttons plus the
 * 3 s hold makes accidental triggers during normal operation essentially
 * impossible.
 */

#include "Buttons.h"

Buttons::Buttons(const ButtonPinConfig& pins)
    : pins_(pins), resetComboFired_(false), resetComboActive_(false) {
    up_     = {pins.up,     true, true, 0, 0, 0, false};
    down_   = {pins.down,   true, true, 0, 0, 0, false};
    select_ = {pins.select, true, true, 0, 0, 0, false};
    back_   = {pins.back,   true, true, 0, 0, 0, false};
}

void Buttons::begin() {
    // TODO: VERIFY button wiring. Assumed: button to GND, internal
    // pull-up, PRESSED = LOW. With unassigned (TODO) pins we never touch
    // the GPIO and the buttons stay inert.
    Btn* all[4] = {&up_, &down_, &select_, &back_};
    for (Btn* b : all) {
        if (pinAssigned(b->pin)) {
            pinMode(b->pin, INPUT_PULLUP);
        }
        b->stableLevel = true;   // released
        b->lastRawLevel = true;
        b->lastChangeMs = millis();
        b->pressedSinceMs = 0;
        b->lastRepeatMs = 0;
        b->repeatArmed = false;
    }
}

bool Buttons::readLevel(const Btn& b) const {
    // Unassigned pin => always "released"; never digitalRead a TODO pin.
    if (!pinAssigned(b.pin)) return true;
    return digitalRead(b.pin) == HIGH;   // true = released (pull-up)
}

void Buttons::serviceButton(Btn& b, uint32_t nowMs) {
    const bool raw = readLevel(b);
    if (raw != b.lastRawLevel) {
        b.lastRawLevel = raw;
        b.lastChangeMs = nowMs;      // restart the debounce window
    }
    // Accept a new stable level only after DEBOUNCE_MS of consistency.
    if (raw != b.stableLevel &&
        (uint32_t)(nowMs - b.lastChangeMs) >= DEBOUNCE_MS) {
        b.stableLevel = raw;
        if (!b.stableLevel) {        // press edge accepted
            b.pressedSinceMs = nowMs;
            b.lastRepeatMs = nowMs;
            b.repeatArmed = false;
        } else {
            b.pressedSinceMs = 0;
        }
    }
}

void Buttons::update() {
    const uint32_t nowMs = millis();
    serviceButton(up_, nowMs);
    serviceButton(down_, nowMs);
    serviceButton(select_, nowMs);
    serviceButton(back_, nowMs);

    // ---- SELECT + BACK hard-reset combo ------------------------------
    const bool comboHeld = (!select_.stableLevel && !back_.stableLevel);
    resetComboActive_ = comboHeld;
    if (comboHeld) {
        const uint32_t heldFor = nowMs - select_.pressedSinceMs;
        if (!resetComboFired_ && heldFor >= RESET_HOLD_MS) {
            resetComboFired_ = true;   // fire once per hold
        }
    } else {
        resetComboFired_ = false;      // re-arm after release
    }
}

ButtonEvent Buttons::pollEvent() {
    const uint32_t nowMs = millis();

    // The reset combo has absolute priority.
    if (resetComboFired_) {
        resetComboFired_ = false;
        return ButtonEvent::RESET_REQUEST;
    }

    // Press edges (accepted by serviceButton's debounce).
    if (!up_.stableLevel && up_.pressedSinceMs != 0 &&
        (uint32_t)(nowMs - up_.pressedSinceMs) < DEBOUNCE_MS) {
        up_.pressedSinceMs = 0;
        return ButtonEvent::UP_PRESSED;
    }
    if (!down_.stableLevel && down_.pressedSinceMs != 0 &&
        (uint32_t)(nowMs - down_.pressedSinceMs) < DEBOUNCE_MS) {
        down_.pressedSinceMs = 0;
        return ButtonEvent::DOWN_PRESSED;
    }
    if (!select_.stableLevel && select_.pressedSinceMs != 0 &&
        (uint32_t)(nowMs - select_.pressedSinceMs) < DEBOUNCE_MS) {
        select_.pressedSinceMs = 0;
        return ButtonEvent::SELECT_PRESSED;
    }
    if (!back_.stableLevel && back_.pressedSinceMs != 0 &&
        (uint32_t)(nowMs - back_.pressedSinceMs) < DEBOUNCE_MS) {
        back_.pressedSinceMs = 0;
        return ButtonEvent::BACK_PRESSED;
    }

    // Auto-repeat for held UP/DOWN (value editing).
    if (!up_.stableLevel && up_.pressedSinceMs != 0) {
        if ((uint32_t)(nowMs - up_.pressedSinceMs) >= REPEAT_DELAY_MS &&
            (uint32_t)(nowMs - up_.lastRepeatMs) >= REPEAT_PERIOD_MS) {
            up_.lastRepeatMs = nowMs;
            up_.repeatArmed = true;
            return ButtonEvent::UP_REPEAT;
        }
    }
    if (!down_.stableLevel && down_.pressedSinceMs != 0) {
        if ((uint32_t)(nowMs - down_.pressedSinceMs) >= REPEAT_DELAY_MS &&
            (uint32_t)(nowMs - down_.lastRepeatMs) >= REPEAT_PERIOD_MS) {
            down_.lastRepeatMs = nowMs;
            down_.repeatArmed = true;
            return ButtonEvent::DOWN_REPEAT;
        }
    }

    return ButtonEvent::NONE;
}

bool Buttons::resetComboActive() const {
    return resetComboActive_;
}
