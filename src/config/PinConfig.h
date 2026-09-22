/**
 * PinConfig.h
 * ===========
 * ALL ESP32 pin assignments in one place.
 *
 * WHY: pin mapping is pure hardware detail; the task requires it not to be
 * scattered through drivers, and unknown mappings must be obvious TODOs.
 *
 * TODO(hardware) — VERIFY EVERY PIN BELOW against your actual wiring:
 *   - L298N ENA/ENB must be connected to ESP32 pins that support LEDC
 *     PWM output (any output-capable GPIO on the ESP32 does).
 *   - The RLS08 is an 8-channel IR array with one DIGITAL output per
 *     channel (D1..D8, HIGH = line detected under that channel —
 *     based on community ESP32/AVR wiring, see README "Hardware notes").
 *     TODO(hardware): confirm your board's channel order and polarity;
 *     some variants expose analog or inverted outputs.
 *   - Do NOT use input-only pins (GPIO 34-39) for PWM outputs.
 *   - Avoid GPIO 6-11 (flash) and be careful with strapping pins
 *     (0, 2, 5, 12, 15) at boot.
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// L298N dual H-bridge
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

// ---------------------------------------------------------------------------
// Smartflex RLS08 8-channel line sensor
// ---------------------------------------------------------------------------

struct RLS08PinConfig {
    /// Digital inputs D1..D8, leftmost channel first (channel 0 = left).
    /// TODO(hardware): verify physical left/right ordering on your robot!
    uint8_t channelPins[8];
};

// ---------------------------------------------------------------------------
// Master pin map — placeholders, MUST be verified before hardware use.
// The values below mirror a common ESP32 dev-board wiring and compile
// fine, but nothing here is guaranteed for your build.
// ---------------------------------------------------------------------------

inline L298NPinConfig defaultL298NPins() {
    // TODO(hardware): verify. Chosen to avoid strapping/flash pins.
    L298NPinConfig p;
    p.ena = 25;  p.in1 = 26;  p.in2 = 27;   // left motor
    p.enb = 14;  p.in3 = 12;  p.in4 = 13;   // right motor
    return p;
}

inline RLS08PinConfig defaultRLS08Pins() {
    // TODO(hardware): verify channel order and polarity.
    RLS08PinConfig p;
    const uint8_t pins[8] = {4, 5, 18, 19, 21, 22, 23, 32};
    for (int i = 0; i < 8; ++i) p.channelPins[i] = pins[i];
    return p;
}

#endif // PIN_CONFIG_H
