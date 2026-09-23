/**
 * test_sensor.ino
 * ===============
 * FOCUSED sensor-system test sketch — completely independent of the
 * line-following control system (no motors, no PID, no OLED menu).
 *
 * WHAT IT VERIFIES (run with Serial Monitor at 115200 baud):
 *   1. Individual sensor readings (all 8 channels)
 *   2. Analog/digital behaviour of each channel (raw GPIO levels)
 *   3. Sensor polarity (which level means "line")
 *   4. Sensor ordering/orientation (Sensor 1 must react first when a
 *      line is moved in from the RIGHT side of the robot)
 *   5. Line detection (active channel count)
 *   6. Calculated line position / error (same weighted-average math as
 *      the main sketch, duplicated here on purpose so a math bug in the
 *      main sketch cannot hide)
 *
 * HOW TO USE:
 *   - Slide a dark line (or your hand) across the sensor array and watch
 *     the channel bits. Sensor 1 is printed on the RIGHT of the bit row,
 *     matching the robot's forward view.
 *   - If the bits are INVERTED (1s everywhere on white, 0s on the line),
 *     the polarity flag is wrong: note it and fix lineIsHigh in the main
 *     configuration (or verify the sensor's actual output behaviour).
 *   - If the wrong end reacts first, the channel order is mirrored:
 *     note it and fix reverseOrder.
 *
 * Pin assignments come from src/config/PinConfig.h — the SAME single
 * source of truth as the main sketch, so the test can never drift from
 * the real wiring. Unassigned (TODO) channels print '-'.
 */

#include <Arduino.h>
#include "src/config/PinConfig.h"

static const int NUM_CHANNELS = 8;
static RLS08PinConfig pins = defaultRLS08Pins();

// Same polarity flag as the main sketch. Flip BOTH here and in the main
// configuration if the physical test proves the polarity inverted.
static bool lineIsHigh = true;

void setup() {
    Serial.begin(115200);
    delay(200);  // let the monitor attach (setup-time only)

    Serial.println();
    Serial.println("=== RLS08 sensor test ===");
    Serial.println("Slide a dark line across the array.");
    Serial.println("Bit row prints Sensor 8..1 left-to-right");
    Serial.println("(Sensor 1 = RIGHTMOST, matching the robot's view).");
    Serial.println();

    for (int i = 0; i < NUM_CHANNELS; ++i) {
        if (pinAssigned(pins.sensorPins[i])) {
            // The RLS08 is assumed digital here. TODO(hardware): verify
            // whether your variant is analog — if so, use analogRead and
            // a threshold instead (see the note at the bottom of output).
            pinMode(pins.sensorPins[i], INPUT);
        }
    }
    printPinMap();
}

void loop() {
    static uint32_t lastPrintMs = 0;
    const uint32_t now = millis();

    // 5 Hz print rate: fast enough to follow a hand, slow enough to read.
    if (now - lastPrintMs < 200) return;
    lastPrintMs = now;

    // ---- 1/2/3: read raw GPIO levels and interpret polarity -----------
    bool level[NUM_CHANNELS];   // raw HIGH/LOW from the pin
    bool line[NUM_CHANNELS];   // after polarity interpretation
    int activeCount = 0;
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        if (!pinAssigned(pins.sensorPins[i])) {
            level[i] = false;
            line[i] = false;
            continue;
        }
        level[i] = (digitalRead(pins.sensorPins[i]) == HIGH);
        line[i] = (level[i] == lineIsHigh);
        if (line[i]) ++activeCount;
    }

    // ---- Bit row: Sensor 8..1 printed left-to-right --------------------
    char bits[NUM_CHANNELS + 1];
    char lvl[NUM_CHANNELS + 1];
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        const int sensor = NUM_CHANNELS - 1 - i;   // print Sensor 8 first
        bits[i] = line[sensor] ? '1'
                               : (pinAssigned(pins.sensorPins[sensor]) ? '0' : '-');
        lvl[i]  = level[sensor] ? 'H'
                               : (pinAssigned(pins.sensorPins[sensor]) ? 'L' : '-');
    }
    bits[NUM_CHANNELS] = '\0';
    lvl[NUM_CHANNELS] = '\0';

    // ---- 5/6: line detection + weighted position ----------------------
    float position = 0.0f;
    bool lineDetected = (activeCount > 0);
    bool junction = (activeCount == NUM_CHANNELS);
    if (lineDetected && !junction) {
        // Same weights as the main sketch: Sensor 1 (index 0, rightmost)
        // -> +1 ... Sensor 8 (index 7, leftmost) -> -1.
        const float half = (NUM_CHANNELS - 1) / 2.0f;
        float sum = 0.0f;
        for (int i = 0; i < NUM_CHANNELS; ++i) {
            if (line[i]) sum += (half - (float)i) / half;
        }
        position = sum / (float)activeCount;
    }

    Serial.print("line: ");
    Serial.print(bits);
    Serial.print("  raw: ");
    Serial.print(lvl);
    Serial.print("  active=");
    Serial.print(activeCount);

    if (junction) {
        Serial.print("  JUNCTION (all 8 active)");
    } else if (lineDetected) {
        // Error vs a centred target (target = 0 here; the main sketch's
        // target position is a config value).
        Serial.print("  pos=");
        Serial.print(position, 3);
        Serial.print("  err=");
        Serial.print(position, 3);
    } else {
        Serial.print("  NO LINE");
    }
    Serial.println();
}

void printPinMap() {
    Serial.println("Channel -> GPIO map (from PinConfig.h):");
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        Serial.print("  Sensor ");
        Serial.print(i + 1);
        Serial.print(" (");
        Serial.print(i == 0 ? "RIGHTMOST" :
                    i == NUM_CHANNELS - 1 ? "leftmost " : "         ");
        Serial.print("): GPIO ");
        if (pinAssigned(pins.sensorPins[i])) {
            Serial.println(pins.sensorPins[i]);
        } else {
            Serial.println("TODO: VERIFY (unassigned)");
        }
    }
    Serial.println();
    Serial.println("If your variant has ANALOG outputs, this digital test");
    Serial.println("will read garbage - verify AOUT mode first (README).");
    Serial.println();
}
