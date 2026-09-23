/**
 * test_motors.ino
 * ===============
 * FOCUSED motor/L298N test sketch — completely independent of the sensor
 * system and the line-following controller.
 *
 * WHAT IT VERIFIES (run with Serial Monitor at 115200 baud):
 *   1. Left motor forward        5. Both motors forward
 *   2. Left motor reverse        6. Both motors reverse
 *   3. Right motor forward       7. Stop (coast)
 *   4. Right motor reverse       8. PWM sweep 0 -> max -> 0 (both)
 *
 * Speeds are deliberately CONSERVATIVE (25% duty for direction tests) —
 * put the robot on a stand/block so the wheels are free.
 *
 * HOW TO DIAGNOSE:
 *   - A motor spins the WRONG WAY          -> swap that motor's two L298N
 *     OUT wires, or set invert=true for it in main.ino's motor config.
 *   - The LEFT command moves the RIGHT motor (or vice versa)
 *     -> left/right mapping is swapped: swap the ENA/IN1/IN2 vs
 *     ENB/IN3/IN4 channel assignment in PinConfig.h.
 *   - A motor never spins                  -> check ENA/ENB jumper/pin,
 *     motor supply voltage, common ground (see README TODO).
 *   - A motor only runs at high PWM        -> deadband; note the duty at
 *     which motion starts (future per-motor deadband compensation).
 *
 * Pin assignments come from src/config/PinConfig.h — the SAME single
 * source of truth as the main sketch.
 */

#include <Arduino.h>
#include "src/config/PinConfig.h"
#include "src/config/RobotConfig.h"   ///< MotionConfig (PWM freq/res/max duty)

static L298NPinConfig pins = defaultL298NPins();
static MotionConfig motion;   // PWM frequency/resolution/max duty defaults

// Conservative test duty: enough to spin an N20 on a bench, safe if a
// wheel touches the ground by accident.
static const uint16_t TEST_DUTY_FRACTION = 4;  // duty = maxPwm / 4 (25%)

void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.println();
    Serial.println("=== L298N / N20 motor test ===");
    Serial.println("BLOCK THE WHEELS OFF THE GROUND FIRST.");
    Serial.println();

    pinMode(pins.in1, OUTPUT);
    pinMode(pins.in2, OUTPUT);
    pinMode(pins.in3, OUTPUT);
    pinMode(pins.in4, OUTPUT);
    allStop();

    // ESP32 Arduino core 3.x LEDC API (same as the main sketch's Motor.cpp).
    ledcAttach(pins.ena, motion.pwmFrequencyHz, motion.pwmBits);
    ledcAttach(pins.enb, motion.pwmFrequencyHz, motion.pwmBits);
    ledcWrite(pins.ena, 0);
    ledcWrite(pins.enb, 0);

    printPinMap();
    Serial.println("Starting test sequence in 3 s...");
    delay(3000);
}

void loop() {
    // ---- Direction tests (conservative duty) ---------------------------
    Serial.println("[1/8] LEFT forward");
    driveLeft(true);
    pauseAndStop(2000);

    Serial.println("[2/8] LEFT reverse");
    driveLeft(false);
    pauseAndStop(2000);

    Serial.println("[3/8] RIGHT forward");
    driveRight(true);
    pauseAndStop(2000);

    Serial.println("[4/8] RIGHT reverse");
    driveRight(false);
    pauseAndStop(2000);

    Serial.println("[5/8] BOTH forward");
    driveBoth(true);
    pauseAndStop(2000);

    Serial.println("[6/8] BOTH reverse");
    driveBoth(false);
    pauseAndStop(2000);

    Serial.println("[7/8] STOP (coast) - wheels should roll free");
    allStop();
    pauseAndStop(2000);

    // ---- PWM range sweep ------------------------------------------------
    Serial.println("[8/8] PWM sweep both motors 0 -> max -> 0");
    pwmSweep();

    Serial.println();
    Serial.println("Sequence complete. Restarting in 5 s (or reset).");
    Serial.println("Fix any direction/mapping issues, see header notes.");
    delay(5000);
}

// ---------------------------------------------------------------------------
// Low-level L298N channel control (deliberately explicit — this sketch is
// a wiring-verification tool, not a reusable driver)
// ---------------------------------------------------------------------------

void setLeftBridge(bool forward) {
    digitalWrite(pins.in1, forward ? HIGH : LOW);
    digitalWrite(pins.in2, forward ? LOW  : HIGH);
}

void setRightBridge(bool forward) {
    digitalWrite(pins.in3, forward ? HIGH : LOW);
    digitalWrite(pins.in4, forward ? LOW  : HIGH);
}

void coastBridge() {
    // Both inputs LOW = coast (same as the main sketch's default stop).
    digitalWrite(pins.in1, LOW);
    digitalWrite(pins.in2, LOW);
    digitalWrite(pins.in3, LOW);
    digitalWrite(pins.in4, LOW);
}

void driveLeft(bool forward) {
    setLeftBridge(forward);
    ledcWrite(pins.ena, motion.maxPwm / TEST_DUTY_FRACTION);
    ledcWrite(pins.enb, 0);   // right stays stopped
}

void driveRight(bool forward) {
    setRightBridge(forward);
    ledcWrite(pins.enb, motion.maxPwm / TEST_DUTY_FRACTION);
    ledcWrite(pins.ena, 0);   // left stays stopped
}

void driveBoth(bool forward) {
    setLeftBridge(forward);
    setRightBridge(forward);
    const uint16_t duty = motion.maxPwm / TEST_DUTY_FRACTION;
    ledcWrite(pins.ena, duty);
    ledcWrite(pins.enb, duty);
}

void allStop() {
    coastBridge();
    ledcWrite(pins.ena, 0);
    ledcWrite(pins.enb, 0);
}

void pauseAndStop(uint32_t runMs) {
    delay(runMs);
    allStop();
    delay(500);   // clear gap between tests
}

void pwmSweep() {
    setLeftBridge(true);
    setRightBridge(true);
    // Ramp up and down over ~4 s; note the duty where each motor starts
    // turning (deadband measurement for future compensation).
    const int steps = 50;
    const uint16_t maxDuty = motion.maxPwm;
    for (int i = 0; i <= steps; ++i) {
        const uint16_t duty = (uint16_t)((long)maxDuty * i / steps);
        ledcWrite(pins.ena, duty);
        ledcWrite(pins.enb, duty);
        delay(40);
    }
    for (int i = steps; i >= 0; --i) {
        const uint16_t duty = (uint16_t)((long)maxDuty * i / steps);
        ledcWrite(pins.ena, duty);
        ledcWrite(pins.enb, duty);
        delay(40);
    }
    allStop();
}

void printPinMap() {
    Serial.println("L298N -> ESP32 (from PinConfig.h, CONFIRMED):");
    Serial.print("  ENA="); Serial.print(pins.ena);
    Serial.print(" IN1="); Serial.print(pins.in1);
    Serial.print(" IN2="); Serial.println(pins.in2);
    Serial.print("  ENB="); Serial.print(pins.enb);
    Serial.print(" IN3="); Serial.print(pins.in3);
    Serial.print(" IN4="); Serial.println(pins.in4);
    Serial.println();
    Serial.println("Assumes: OUT1/OUT2 -> LEFT motor, OUT3/OUT4 -> RIGHT.");
    Serial.println("If left/right is swapped, fix it in PinConfig.h.");
    Serial.println();
}
