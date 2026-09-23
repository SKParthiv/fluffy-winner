/**
 * backup.ino — MINIMAL line follower (guaranteed-fallback firmware)
 * ================================================================
 * One file. No menu, no OLED, no buttons, no persistence, no libraries
 * beyond the Arduino core. Direct upload, hardcoded constants.
 *
 * PURPOSE: this is the "for sure to work" backup. If the modular
 * system (main.ino) misbehaves on real hardware, upload this and the
 * robot still follows a line. It uses ONLY the CONFIRMED wiring:
 *
 *   L298N:  ENA=4  IN1=18  IN2=19   (left motor,  OUT1/OUT2)
 *           ENB=23 IN3=21  IN4=22   (right motor, OUT3/OUT4)
 *   RLS08:  Sensor 1..6 = GPIO 32, 33, 25, 26, 27, 14
 *           (Sensor 1 = RIGHTMOST channel; Sensors 7/8 unconnected)
 *
 * HOW IT WORKS (deliberately textbook):
 *   1. Read the 6 sensor bits (HIGH = line, assumed).
 *   2. Weighted position: rightmost sensor = +1 ... leftmost = -1.
 *   3. error = position - TARGET (TARGET hardcoded below).
 *   4. PID on the error -> steering correction.
 *   5. Mix: left = BASE + steer, right = BASE - steer (clamped 0..255).
 *
 * If the robot steers AWAY from the line, the wiring polarity/order is
 * wrong — flip STEER_SIGN below. Do not change anything else.
 *
 * Verified: compiles for ESP32 (arduino-cli, core 3.3.11).
 * NOT verified on physical hardware — run on a stand first.
 */

// ============================ HARDCODED CONSTANTS ==========================
// Tune these ONLY after the sensor polarity/order is verified with
// test_sensor.ino and motors with test_motors.ino.

const float KP = 120.0f;   // proportional gain (start: 80..150)
const float KI = 0.0f;     // integral gain (keep 0 until P works)
const float KD = 30.0f;    // derivative gain (damps oscillation)

const float TARGET = 0.0f;    // desired line position (-1 right .. +1 left)
const int   BASE_SPEED = 90;  // cruise PWM (0..255). Start LOW (60-100).
const int   MAX_SPEED  = 200; // clamp so PID can never saturate fully

const float STEER_SIGN = 1.0f;  // flip to -1.0f if robot steers AWAY from line

const bool  LINE_IS_HIGH = true;   // false if sensors read LOW on the line
const unsigned long LOST_TIMEOUT_MS = 300;  // stop after line lost this long

// ============================ PIN DEFINITIONS ==============================
// CONFIRMED wiring only (see src/config/PinConfig.h for the full map).

const int SENSOR_PINS[6] = {32, 33, 25, 26, 27, 14};  // S1(right)..S6(left)
const int NUM_SENSORS = 6;

const int ENA = 4, IN1 = 18, IN2 = 19;   // left motor
const int ENB = 23, IN3 = 21, IN4 = 22;  // right motor

// ============================ RUNTIME STATE ================================

float integral = 0.0f;
float lastError = 0.0f;
unsigned long lineLostSinceMs = 0;

// ============================ MOTOR HELPERS ================================
// Plain analogWrite PWM (ESP32 core 3.x maps it to LEDC automatically).

void motorLeft(int pwm) {           // pwm: -255..255, + = forward
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
// -1 = line under Sensor 6 (leftmost), 0 = centred. Same sign convention
// as the main system (e > 0 = line RIGHT of robot centre).
// Returns false when NO sensor sees the line (line lost).

bool readLinePosition(float& position) {
    float weightedSum = 0.0f;
    int   activeCount = 0;

    for (int i = 0; i < NUM_SENSORS; ++i) {
        const bool seesLine = (digitalRead(SENSOR_PINS[i]) == HIGH) == LINE_IS_HIGH;
        if (seesLine) {
            // Sensor 1 (index 0) = rightmost = weight +1.
            // Weight steps: +1.0, +0.6, +0.2, -0.2, -0.6, -1.0
            const float weight = 1.0f - (2.0f * i) / (float)(NUM_SENSORS - 1);
            weightedSum += weight;
            ++activeCount;
        }
    }

    if (activeCount == 0) return false;
    position = weightedSum / (float)activeCount;
    return true;
}

// ============================ SETUP / LOOP =================================

void setup() {
    for (int i = 0; i < NUM_SENSORS; ++i) pinMode(SENSOR_PINS[i], INPUT);

    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    motorsStop();

    Serial.begin(115200);
    Serial.println("backup.ino: minimal line follower (system ON at boot)");
}

void loop() {
    const unsigned long nowMs = millis();

    float position;
    const bool lineFound = readLinePosition(position);

    if (!lineFound) {
        // Line lost: stop safely after a short grace period (junctions
        // can momentarily blank all sensors).
        if (lineLostSinceMs == 0) lineLostSinceMs = nowMs;
        if (nowMs - lineLostSinceMs > LOST_TIMEOUT_MS) {
            motorsStop();
            integral = 0.0f;   // no stale integral when line returns
        }
        lastError = 0.0f;
        delay(2);
        return;
    }
    lineLostSinceMs = 0;

    // ---- PID ----------------------------------------------------------
    const float error = position - TARGET;

    // Fixed 5 ms sample period (delay(5) at the loop bottom enforces it).
    const float dt = 0.005f;
    integral += KI * error * dt;
    integral = constrain(integral, -100.0f, 100.0f);   // anti-windup
    const float derivative = (error - lastError) / dt;
    lastError = error;

    const float steer = STEER_SIGN * (KP * error + integral + KD * derivative);

    // ---- Mix and drive --------------------------------------------------
    const int left  = constrain((int)(BASE_SPEED + steer), -MAX_SPEED, MAX_SPEED);
    const int right = constrain((int)(BASE_SPEED - steer), -MAX_SPEED, MAX_SPEED);
    motorLeft(left);
    motorRight(right);

    delay(5);   // ~200 Hz control rate, matching the main system
}
