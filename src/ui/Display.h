/**
 * Display.h
 * =========
 * Thin wrapper around the U8g2 OLED library.
 *
 * WHY a wrapper: the OLED controller (SSD1306 vs SH1106) and the I2C
 * wiring are NOT yet confirmed hardware facts. The ONE constructor line
 * to change is clearly marked below in Display.cpp (search OLED_TODO).
 * Nothing else in the codebase touches U8g2.
 *
 * Failsafe behaviour: if the OLED is absent or the pins are unassigned,
 * begin() returns false and every draw call becomes a no-op — the robot
 * (and the serial diagnostics) keep working without a display.
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <U8g2lib.h>
#include "../config/PinConfig.h"

class Display {
public:
    explicit Display(const OLEDPinConfig& pins);

    /// Initialise I2C + OLED. Returns false when unusable (absent or
    /// unassigned pins) — the caller must treat that as non-fatal.
    bool begin();

    bool isWorking() const { return working_; }

    /// Start a frame (call before draw calls).
    void beginFrame();
    /// Finish and push the frame to the panel.
    void endFrame();

    // --- Small-font text primitives (128x64: 21 cols x 8 rows) ---
    void text(int x, int y, const char* s);           ///< normal font
    void textInverted(int x, int y, const char* s);   ///< highlight bar
    void textBig(int x, int y, const char* s);        ///< title font

    /// Draw a horizontal rule.
    void hline(int y);

    /// Inverted-video state for textInverted.
    void setInverted(bool inv) { inverted_ = inv; }

    /// Panel geometry in pixels (both common controllers are 128x64;
    /// TODO: VERIFY the panel size with the actual module).
    static const int WIDTH = 128;
    static const int HEIGHT = 64;

private:
    OLEDPinConfig pins_;
    bool working_;
    bool inverted_;
    U8G2_SH1106_128X64_NONAME_F_HW_I2C* u8g2_;  ///< see OLED_TODO in .cpp
};

#endif // DISPLAY_H
