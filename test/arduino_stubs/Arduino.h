/**
 * arduino_stubs/Arduino.h
 * =======================
 * Minimal Arduino API surface for HOST compilation of the modules
 * (unit testing without hardware). Only what this codebase uses.
 * On the ESP32 the real Arduino.h is used instead.
 */
#ifndef ARDUINO_STUBS_H
#define ARDUINO_STUBS_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cctype>
#include <cmath>
#include <iostream>

// ---- time ---------------------------------------------------------------
uint32_t micros();
uint32_t millis();

// ---- digital / PWM ------------------------------------------------------
#define HIGH 1
#define LOW  0
#define OUTPUT 0x01
#define INPUT  0x00

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int  digitalRead(uint8_t) { return 0; }

// ESP32 Arduino core 3.x LEDC API stubs.
inline void ledcAttach(uint8_t, uint32_t, uint8_t) {}
inline void ledcWrite(uint8_t, uint32_t) {}

// ---- serial --------------------------------------------------------------
// A tiny Print stub so F() and println/print work on the host.
class StubSerial {
public:
    void begin(unsigned long) {}
    int available() { return 0; }
    int read() { return -1; }
    template <typename T> void print(const T& v) { std::cout << v; }
    template <typename T> void println(const T& v) { std::cout << v << "\n"; }
    void println() { std::cout << "\n"; }
};
extern StubSerial Serial;

// F() macro: on AVR it moves literals to flash; on host/ESP32 it is a no-op.
#define F(x) (x)

// delay(): stubbed out — the control code NEVER calls it (only the
// one-off setup-time serial settle in Diagnostics does).
inline void delay(unsigned long) {}

#endif // ARDUINO_STUBS_H
