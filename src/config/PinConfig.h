/**
 * PinConfig.h
 * ===========
 * ALL ESP32 pin assignments in one place (task: "do not scatter GPIO numbers
 * through the code").
 *
 * CONFIRMED values are marked "CONFIRMED". Everything not confirmed is
 * PIN_UNASSIGNED (0xFF) with a "TODO: VERIFY" marker — unconfirmed hardware
 * facts are NEVER invented; the drivers fail safe on unassigned pins.
 *
 * Sensor orientation (CONFIRMED):
 *   Sensor 1 is the RIGHTMOST channel when viewed from the robot's normal
 *   forward-facing orientation. Sensor 8 would be the leftmost.
 *   (See RLS08LineSensor: channel index 0 == Sensor 1 == rightmost, with
 *   position weight +1 = right edge of the array.)
 *
 * Electrical warning (see README "Hardware & Validation TODO"):
 *   The RLS08 is expected to be powered at 5 V. ESP32 GPIO/ADC inputs are
 *   NOT 5 V tolerant. The AOUT voltage of every sensor channel MUST be
 *   measured before connecting to the ESP32; a divider/protection may be
 *   required. Do not assume a 5 V-powered sensor swings only to 3.3 V.
 *
 * General ESP32 constraints:
 *   - GPIO 34-39 are input-only (fine for sensors/buttons, NOT for PWM).
 *   - Avoid GPIO 6-11 (flash). Strapping pins (0, 2, 5, 12, 15) need care
 *     at boot; none of the CONFIRMED pins below are strapping pins.
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <stdint.h>

/// Sentinel for "not yet assigned — do not touch hardware on this pin".
#define PIN_UNASSIGNED 0xFF

/// True when a pin has a real assignment (not the TODO sentinel).
inline bool pinAssigned(uint8_t pin) { return pin != PIN_UNASSIGNED; }

// ---------------------------------------------------------------------------
// L298N dual H-bridge — CONFIRMED wiring (task statement)
// ---------------------------------------------------------------------------

struct L298NPinConfig {
    // Left motor channel (OUT1/OUT2)
    uint8_t ena;      ///< PWM pin  (ENA on the L298N board)
    uint8_t in1;      ///< direction pin IN1
    uint8_t in2;      ///< direction pin IN2

    // Right motor channel (OUT3/OUT4)
    uint8_t enb;      ///< PWM pin  (ENB)
    uint8_t in3;      ///< direction pin IN3
    uint8_t in4;      ///< direction pin IN4
};

inline L298NPinConfig defaultL298NPins() {
    L298NPinConfig p;
    // CONFIRMED:
    p.ena = 4;   p.in1 = 18;  p.in2 = 19;   // left motor
    p.enb = 23;  p.in3 = 21;  p.in4 = 22;   // right motor
    // TODO(hardware): verify left/right motor mapping (which physical motor
    // is on OUT1/OUT2 vs OUT3/OUT4) and motor polarity with test_motors.ino.
    return p;
}

// ---------------------------------------------------------------------------
// Smartflex RLS08 8-channel line sensor
//
// CONFIRMED (task statement): Sensor 1 = RIGHTMOST channel.
//   sensorPins[0] = Sensor 1 (rightmost) ... sensorPins[7] = Sensor 8.
// CONFIRMED pins: Sensors 1-6. Sensors 7 & 8 are TODO: VERIFY.
// ---------------------------------------------------------------------------

struct RLS08PinConfig {
    /// sensorPins[i] = GPIO of Sensor (i+1). Index 0 = Sensor 1 = RIGHTMOST.
    uint8_t sensorPins[8];
};

inline RLS08PinConfig defaultRLS08Pins() {
    RLS08PinConfig p;
    // CONFIRMED (task statement):
    p.sensorPins[0] = 32;  // Sensor 1 (rightmost)
    p.sensorPins[1] = 33;  // Sensor 2
    p.sensorPins[2] = 25;  // Sensor 3
    p.sensorPins[3] = 26;  // Sensor 4
    p.sensorPins[4] = 27;  // Sensor 5
    p.sensorPins[5] = 14;  // Sensor 6
    // TODO: VERIFY — remaining sensor GPIO assignments are NOT confirmed:
    p.sensorPins[6] = PIN_UNASSIGNED;  // Sensor 7 — TODO: VERIFY
    p.sensorPins[7] = PIN_UNASSIGNED;  // Sensor 8 — TODO: VERIFY
    // TODO: VERIFY sensor channel polarity (HIGH = line or LOW = line).
    // TODO: VERIFY analog vs digital operating mode of your RLS08 variant.
    // TODO: VERIFY AOUT max voltage before connecting (5 V warning above).
    return p;
}

// ---------------------------------------------------------------------------
// OLED display (I2C) — TODO: VERIFY everything except the intent to use I2C
// ---------------------------------------------------------------------------

struct OLEDPinConfig {
    uint8_t sda;      ///< I2C data
    uint8_t scl;      ///< I2C clock
    uint8_t address;  ///< 7-bit I2C address
};

inline OLEDPinConfig defaultOLEDPins() {
    OLEDPinConfig p;
    // TODO: VERIFY SDA/SCL pins and the OLED I2C address for your wiring.
    // ESP32 default I2C pins (21/22) are NOT available here — GPIO 21 and 22
    // are already CONFIRMED as L298N IN3/IN4. Choose other pins and confirm.
    p.sda = PIN_UNASSIGNED;  // TODO: VERIFY
    p.scl = PIN_UNASSIGNED;  // TODO: VERIFY
    p.address = 0x3C;        // TODO: VERIFY (common default; some boards 0x3D)
    return p;
}

// ---------------------------------------------------------------------------
// Four-button UI (UP / DOWN / SELECT / BACK) — TODO: VERIFY all pins
// ---------------------------------------------------------------------------

struct ButtonPinConfig {
    uint8_t up;
    uint8_t down;
    uint8_t select;
    uint8_t back;
};

inline ButtonPinConfig defaultButtonPins() {
    ButtonPinConfig p;
    // TODO: VERIFY — button GPIO assignments are NOT confirmed. Do NOT
    // invent them. Assign real pins here once the wiring is decided, then
    // verify pull-up/pull-down behaviour and electrical levels.
    p.up     = PIN_UNASSIGNED;  // TODO: VERIFY
    p.down   = PIN_UNASSIGNED;  // TODO: VERIFY
    p.select = PIN_UNASSIGNED;  // TODO: VERIFY
    p.back   = PIN_UNASSIGNED;  // TODO: VERIFY
    // TODO: VERIFY whether buttons pull the pin LOW (internal pull-up,
    // pressed = LOW — assumed by Buttons.cpp) or HIGH (external pull-down).
    return p;
}

#endif // PIN_CONFIG_H
