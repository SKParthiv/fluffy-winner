/**
 * Display.cpp — see header.
 *
 * OLED_TODO — THE ONE PLACE TO CHANGE THE OLED CONSTRUCTOR:
 * ---------------------------------------------------------------------------
 * The controller is NOT yet confirmed. The default below assumes SH1106
 * (many cheap 1.3" modules); if your module is the more common 0.96"
 * SSD1306, replace the constructor with:
 *
 *     U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE)
 *
 * ...and delete the SH1106 line. Nothing else changes. Confirm the
 * controller from the module's silkscreen/datasheet — do not guess.
 * ---------------------------------------------------------------------------
 */

#include "Display.h"
#include <Wire.h>

Display::Display(const OLEDPinConfig& pins)
    : pins_(pins), working_(false), inverted_(false), u8g2_(nullptr) {}

bool Display::begin() {
    // Failsafe: unassigned (TODO) I2C pins mean the OLED wiring is not
    // confirmed yet. Do not touch random GPIOs; run headless instead.
    if (!pinAssigned(pins_.sda) || !pinAssigned(pins_.scl)) {
        return false;
    }

    // OLED_TODO: constructor line — change SH1106 <-> SSD1306 here only.
    static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2_sh1106(
        U8G2_R0, U8X8_PIN_NONE);
    u8g2_ = &u8g2_sh1106;

    // The ESP32 Arduino core lets us pick arbitrary I2C pins; the OLED
    // address comes from the pin config (TODO: VERIFY 0x3C vs 0x3D).
    u8g2_->setI2CAddress(pins_.address << 1);  // U8g2 wants the 8-bit form
    Wire.begin(pins_.sda, pins_.scl);

    if (!u8g2_->begin()) {
        u8g2_ = nullptr;      // panel absent or init failed: run headless,
        return false;         // never block or crash the robot for a display
    }
    u8g2_->setFont(u8g2_font_6x12_tr);      // normal text
    u8g2_->setFontRefHeightExtendedText();
    working_ = true;
    return true;
}

void Display::beginFrame() {
    if (!working_) return;
    u8g2_->clearBuffer();
}

void Display::endFrame() {
    if (!working_) return;
    u8g2_->sendBuffer();
}

void Display::text(int x, int y, const char* s) {
    if (!working_ || s == nullptr) return;
    if (inverted_) {
        const int w = u8g2_->getStrWidth(s);
        u8g2_->setDrawColor(1);
        u8g2_->drawBox(x - 1, y - 11, w + 2, 13);
        u8g2_->setDrawColor(0);
        u8g2_->drawStr(x, y, s);
        u8g2_->setDrawColor(1);
    } else {
        u8g2_->setDrawColor(1);
        u8g2_->drawStr(x, y, s);
    }
}

void Display::textInverted(int x, int y, const char* s) {
    if (!working_) return;
    const bool prev = inverted_;
    inverted_ = true;
    text(x, y, s);
    inverted_ = prev;
}

void Display::textBig(int x, int y, const char* s) {
    if (!working_ || s == nullptr) return;
    u8g2_->setFont(u8g2_font_10x20_tr);
    u8g2_->setDrawColor(1);
    u8g2_->drawStr(x, y, s);
    u8g2_->setFont(u8g2_font_6x12_tr);
}

void Display::hline(int y) {
    if (!working_) return;
    u8g2_->setDrawColor(1);
    u8g2_->drawHLine(0, y, WIDTH);
}
