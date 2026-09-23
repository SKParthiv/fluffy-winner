/**
 * backup_with_interface.ino — minimal line follower + OLED/button interface
 * =====================================================================
 * Same guaranteed-to-work core as backup.ino, plus the U8g2 OLED and the
 * four buttons, ALL IN ONE FILE. If the modular system (main.ino) is too
 * much for the hardware, this is the fallback that still gives you an
 * interface.
 *
 * The interface is deliberately minimal:
 *   - RUN screen: live sensor bits, position, error, kp/ki/kd, ON/OFF state
 *   - UP/DOWN: adjust the selected parameter (kp, ki, kd, speed)
 *   - SELECT: move to the next parameter
 *   - BACK: START/STOP the robot (safety: boots OFF, like the main system)
 *
 * Uses ONLY the CONFIRMED wiring:
 *   L298N:  ENA=4  IN1=18  IN2=19   (left motor,  OUT1/OUT2)
 *           ENB=23 IN3=21  IN4=22   (right motor, OUT3/OUT4)
 *   RLS08:  Sensor 1..6 = GPIO 32, 33, 25, 26, 27, 14
 *           (Sensor 1 = RIGHTMOST channel; Sensors 7/8 unconnected)
 *
 * OLED and button pins are NOT confirmed yet — they are TODO values
 * below. The sketch runs fine with the OLED absent (headless); assign
 * real pins before expecting the interface to work.
 *
 * OLED controller: SH1106 assumed (same as the main system). If yours is
 * the common 0.96" SSD1306, swap the ONE constructor line marked
 * OLED_TODO below.
 *
 * Verified: compiles for ESP32 (arduino-cli, core 3.3.11, U8g2 2.36.19).
 * NOT verified on physical hardware — run on a stand first.
 */

#include <U8g2lib.h>
#include <Wire.h>

// ============================ HARDCODED/ADJUSTABLE CONSTANTS ==============

float kp = 120.0f;    // adjustable via interface (start: 80..150)
float ki = 0.0f;      // adjustable via interface (keep 0 until P works)
float kd = 30.0f;     // adjustable via interface (damps oscillation)

const float TARGET = 0.0f;    // desired line position (-1 right .. +1 left)
int   baseSpeed = 90;         // adjustable via interface (0..255, start LOW)

const int   MAX_SPEED = 200;  // clamp so PID can never saturate fully
const float STEER_SIGN = 1.0f; // flip to -1.0f if robot steers AWAY from line
const bool  LINE_IS_HIGH = true;  // false if sensors read LOW on the line
const unsigned long LOST_TIMEOUT_MS = 300;

// ============================ PIN DEFINITIONS ==============================
// CONFIRMED wiring only (see src/config/PinConfig.h for the full map).

const int SENSOR_PINS[6] = {32, 33, 25, 26, 27, 14};  // S1(right)..S6(left)
const int NUM_SENSORS = 6;

const int ENA = 4, IN1 = 18, IN2 = 19;   // left motor
const int ENB = 23, IN3 = 21, IN4 = 22;  // right motor

// ---- UI pins: TODO — assign real pins before use --------------------------
// (these mirror src/config/PinConfig.h; unassigned = interface inert)
const int PIN_UNASSIGNED = 255;
const int BTN_UP     = PIN_UNASSIGNED;  // TODO: assign
const int BTN_DOWN   = PIN_UNASSIGNED;  // TODO: assign
const int BTN_SELECT = PIN_UNASSIGNED;  // TODO: assign
const int BTN_BACK   = PIN_UNASSIGNED;  // TODO: assign

const int OLED_SDA = PIN_UNASSIGNED;    // TODO: assign (NOT 21/22 — L298N)
const int OLED_SCL = PIN_UNASSIGNED;    // TODO: assign (NOT 21/22 — L298N)
const int OLED_ADDR = 0x3C;             // TODO: verify (some modules 0x3D)

// ============================ UI STATE =====================================

// OLED_TODO — THE ONE PLACE TO CHANGE THE OLED CONSTRUCTOR (SH1106 assumed;
// for the common 0.96" SSD1306 use U8G2_SSD1306_128X64_NONAME_F_HW_I2C):
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

bool oledWorking = false;

// Parameter selection: 0=kp 1=ki 2=kd 3=speed
int selectedParam = 0;
const int NUM_PARAMS = 4;
const char* PARAM_NAMES[NUM_PARAMS] = {"kp", "ki", "kd", "spd"};

// Robot state: boots OFF (safety, same rule as the main system).
bool robotEnabled = false;

// ============================ RUNTIME STATE ================================

float integral = 0.0f;
float lastError = 0.0f;
unsigned long lineLostSinceMs = 0;

// Live values for the display
float livePosition = 0.0f;
float liveError = 0.0f;
bool  liveLineFound = false;
bool  liveBits[NUM_SENSORS];

// ============================ BUTTON HELPERS ==============================
// Simple debounced edge detection. Buttons assumed pull-up, pressed = LOW
// (same assumption as the main system — verify against wiring).

struct Button {
    int pin;
    bool lastStable;
    unsigned long lastChangeMs;
};

Button buttons[4];  // UP, DOWN, SELECT, BACK
const int BTN_COUNT = 4;
const unsigned long DEBOUNCE_MS = 25;

bool buttonPressed(int idx) {           // true on the press EDGE only
    if (buttons[idx].pin == PIN_UNASSIGNED) return false;
    const bool now = (digitalRead(buttons[idx].pin) == LOW);
    bool edge = false;
    if (now != buttons[idx].lastStable) {
        if (millis() - buttons[idx].lastChangeMs > DEBOUNCE_MS) {
            buttons[idx].lastStable = now;
            buttons[idx].lastChangeMs = millis();
            edge = now;                 // edge only on press (not release)
        }
    } else {
        buttons[idx].lastChangeMs = millis();
    }
    return edge;
}

bool buttonHeld(int idx) {              // true while held down
    if (buttons[idx].pin == PIN_UNASSIGNED) return false;
    return digitalRead(buttons[idx].pin) == LOW;
}

// ============================ MOTOR HELPERS ================================

void motorLeft(int pwm) {               // pwm: -255..255, + = forward
    if (pwm >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
    else          { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); pwm = -pwm; }
    analogWrite(ENA, pwm);
}

void motorRight(int pwm) {
    if (pwm >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
    else          { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); pwm = -pwm; }
    analogWrite(ENB, pwm);
}

void motorsStop() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);    analogWrite(ENB, 0);
}

// ============================ SENSOR =======================================

// Weighted line position: +1 = line under Sensor 1 (RIGHTMOST),
// -1 = line under Sensor 6 (leftmost). Same convention as backup.ino and
// the main system (e > 0 = line RIGHT of robot centre).
bool readLinePosition(float& position) {
    float weightedSum = 0.0f;
    int   activeCount = 0;

    for (int i = 0; i < NUM_SENSORS; ++i) {
        const bool seesLine = (digitalRead(SENSOR_PINS[i]) == HIGH) == LINE_IS_HIGH;
        liveBits[i] = seesLine;
        if (seesLine) {
            const float weight = 1.0f - (2.0f * i) / (float)(NUM_SENSORS - 1);
            weightedSum += weight;
            ++activeCount;
        }
    }

    if (activeCount == 0) return false;
    position = weightedSum / (float)activeCount;
    return true;
}

// ============================ INTERFACE ====================================

void oledInit() {
    if (OLED_SDA == PIN_UNASSIGNED || OLED_SCL == PIN_UNASSIGNED) return;
    oled.setI2CAddress(OLED_ADDR << 1);   // U8g2 wants the 8-bit form
    Wire.begin(OLED_SDA, OLED_SCL);
    if (oled.begin()) {
        oled.setFont(u8g2_font_6x12_tr);
        oledWorking = true;
    }
}

void drawInterface() {
    if (!oledWorking) return;

    oled.clearBuffer();

    // Line 1: state + line status
    oled.drawStr(0, 12, robotEnabled ? "RUN  " : "STOP ");
    oled.drawStr(40, 12, liveLineFound ? "LINE" : "LOST");

    // Line 2: sensor bits, S6..S1 left-to-right (matches physical array)
    char bits[16];
    for (int i = 0; i < NUM_SENSORS; ++i)
        bits[i] = liveBits[NUM_SENSORS - 1 - i] ? '1' : '0';
    bits[NUM_SENSORS] = 0;
    oled.drawStr(0, 26, bits);

    // Line 3: position / error
    char line3[24];
    snprintf(line3, sizeof(line3), "pos%+.2f err%+.2f", livePosition, liveError);
    oled.drawStr(0, 40, line3);

    // Lines 4-5: parameters, selected one highlighted (inverted box)
    char buf[24];
    for (int p = 0; p < NUM_PARAMS; ++p) {
        const int y = 52 + (p / 2) * 12;
        const int x = (p % 2) * 64;
        if (p == 0) snprintf(buf, sizeof(buf), "kp%4.0f", kp);
        else if (p == 1) snprintf(buf, sizeof(buf), "ki%4.1f", ki);
        else if (p == 2) snprintf(buf, sizeof(buf), "kd%4.0f", kd);
        else snprintf(buf, sizeof(buf), "spd%3d", baseSpeed);
        if (p == selectedParam) {
            oled.drawBox(x - 1, y - 11, 62, 13);
            oled.setDrawColor(0);
            oled.drawStr(x, y, buf);
            oled.setDrawColor(1);
        } else {
            oled.drawStr(x, y, buf);
        }
    }

    oled.sendBuffer();
}

void adjustParam(int dir) {             // dir: +1 or -1
    switch (selectedParam) {
        case 0: kp = constrain(kp + dir * 10.0f, 0.0f, 500.0f); break;
        case 1: ki = constrain(ki + dir * 0.5f, 0.0f, 50.0f); break;
        case 2: kd = constrain(kd + dir * 5.0f, 0.0f, 200.0f); break;
        case 3: baseSpeed = constrain(baseSpeed + dir * 10, 0, MAX_SPEED); break;
    }
}

void handleButtons() {
    // UP/DOWN: adjust (with auto-repeat while held)
    static unsigned long lastRepeatMs = 0;
    if (buttonPressed(0)) { adjustParam(+1); lastRepeatMs = millis(); }
    else if (buttonPressed(1)) { adjustParam(-1); lastRepeatMs = millis(); }
    else if ((buttonHeld(0) || buttonHeld(1)) &&
             millis() - lastRepeatMs > 400) {
        adjustParam(buttonHeld(0) ? +1 : -1);
        lastRepeatMs = millis();
    }

    // SELECT: next parameter
    if (buttonPressed(2)) {
        selectedParam = (selectedParam + 1) % NUM_PARAMS;
    }

    // BACK: toggle RUN/STOP
    if (buttonPressed(3)) {
        robotEnabled = !robotEnabled;
        if (!robotEnabled) {
            motorsStop();
            integral = 0.0f;   // no stale integral on restart
            lastError = 0.0f;
        }
    }
}

// ============================ SETUP / LOOP =================================

void setup() {
    for (int i = 0; i < NUM_SENSORS; ++i) pinMode(SENSOR_PINS[i], INPUT);

    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    motorsStop();

    const int btnPins[BTN_COUNT] = {BTN_UP, BTN_DOWN, BTN_SELECT, BTN_BACK};
    for (int i = 0; i < BTN_COUNT; ++i) {
        buttons[i].pin = btnPins[i];
        buttons[i].lastStable = false;
        buttons[i].lastChangeMs = 0;
        if (btnPins[i] != PIN_UNASSIGNED) pinMode(btnPins[i], INPUT_PULLUP);
    }

    oledInit();

    Serial.begin(115200);
    Serial.println("backup_with_interface.ino: boots OFF, BACK toggles RUN/STOP");
}

void loop() {
    handleButtons();

    // ---- Read sensor (always, so the display stays live) ---------------
    float position;
    liveLineFound = readLinePosition(position);
    livePosition = position;
    liveError = position - TARGET;

    if (!robotEnabled) {
        motorsStop();
        drawInterface();
        delay(10);
        return;
    }

    if (!liveLineFound) {
        if (lineLostSinceMs == 0) lineLostSinceMs = millis();
        if (millis() - lineLostSinceMs > LOST_TIMEOUT_MS) {
            motorsStop();
            integral = 0.0f;
        }
        lastError = 0.0f;
        drawInterface();
        delay(5);
        return;
    }
    lineLostSinceMs = 0;

    // ---- PID ----------------------------------------------------------
    const float error = liveError;
    const float dt = 0.005f;
    integral += ki * error * dt;
    integral = constrain(integral, -100.0f, 100.0f);
    const float derivative = (error - lastError) / dt;
    lastError = error;

    const float steer = STEER_SIGN * (kp * error + integral + kd * derivative);

    const int left  = constrain((int)(baseSpeed + steer), -MAX_SPEED, MAX_SPEED);
    const int right = constrain((int)(baseSpeed - steer), -MAX_SPEED, MAX_SPEED);
    motorLeft(left);
    motorRight(right);

    drawInterface();     // ~200 Hz control; display refresh is fast enough
    delay(5);
}
