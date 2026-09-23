# fluffy-winner — ESP32 Line-Follower Control System

A modular Arduino/ESP32 codebase for a two-wheeled differential-drive
line-following robot, with a **mathematically defined path-tracking
controller**, a **completely optional autonomous PID-calibration
supervisor**, and a **local OLED + 4-button configuration interface**.

The most important architectural rule of this project:

> **`CALIBRATION_ENABLED = false` must always be possible, and the robot
> must operate normally.** The calibration module is a slow supervisory
> optimizer that only observes the robot and proposes PID gains. It is
> never part of the real-time control path.

Boot safety rule:

> **The system boots in the OFF state.** Motors are commanded to a safe
> stop every cycle until the user explicitly turns the system ON via the
> OLED menu (SYSTEM). The robot never starts driving at power-up.

---

## Table of contents

1. [Hardware & Validation TODO — START HERE](#hardware--validation-todo--start-here)
2. [Step-by-step bring-up and usage](#step-by-step-bring-up-and-usage)
3. [Hardware](#hardware)
4. [Architecture](#architecture)
5. [Control-flow diagram](#control-flow-diagram)
6. [The mathematics](#the-mathematics)
7. [Enabling / disabling calibration](#enabling--disabling-calibration)
8. [Repository layout](#repository-layout)
9. [Building](#building)
10. [Testing](#testing)
11. [Serial diagnostics commands](#serial-diagnostics-commands)
12. [Coordinate frames, sign conventions and units](#coordinate-frames-sign-conventions-and-units)
13. [Local UI (OLED + 4 buttons)](#local-ui-oled--4-buttons)

---

# Hardware & Validation TODO — START HERE

Everything in this section **still requires physical verification**.
Nothing here has been tested on hardware by the coding environment —
compile success is not hardware validation. Confirmed facts are marked
CONFIRMED; everything else is an open item. Work through this list in
order using the [step-by-step bring-up](#step-by-step-bring-up-and-usage)
procedure below.

## ⚠️ Electrical constraint — read before connecting the sensor

The **ESP32 GPIO/ADC inputs are NOT 5 V tolerant**. The RLS08 is
expected to be powered at **5 V**, and its actual AOUT voltage **must be
measured before connecting any channel to the ESP32**. Do **not** assume
that a 5 V-powered sensor only swings to 3.3 V. If AOUT can exceed
~3.3 V, a voltage divider (or other protection) is **required** on every
channel.

## Sensor (RLS08, 8-channel)

* [ ] Verify RLS08 sensor power voltage
* [ ] Verify AOUT maximum voltage before connecting to ESP32 (5 V warning above)
* [ ] Determine whether voltage divider/protection is required
* [ ] Verify sensor channel polarity (which level = line)
* [ ] Verify sensor ordering
* [ ] Verify Sensor 1 is the rightmost channel (CONFIRMED in software: index 0 = Sensor 1 = rightmost, weight +1)
* [ ] Verify remaining sensor pin assignments (Sensors 7 & 8 are `PIN_UNASSIGNED` — TODO)
* [ ] Verify ADC channel compatibility (if the variant is analog)
* [ ] Verify analog/digital operating mode (driver currently assumes digital, HIGH = line)
* [ ] Verify line/background response
* [ ] Verify calibration behavior

Confirmed sensor GPIO assignments (from `src/config/PinConfig.h`):

```text
Sensor 1 (rightmost) = GPIO 32   (CONFIRMED)
Sensor 2             = GPIO 33   (CONFIRMED)
Sensor 3             = GPIO 25   (CONFIRMED)
Sensor 4             = GPIO 26   (CONFIRMED)
Sensor 5             = GPIO 27   (CONFIRMED)
Sensor 6             = GPIO 14   (CONFIRMED)
Sensor 7             = TODO: VERIFY (PIN_UNASSIGNED)
Sensor 8             = TODO: VERIFY (PIN_UNASSIGNED)
```

Use `test_sensor/test_sensor.ino` for all sensor verification — it is
independent of the line-following system.

**Architecture reference:** the sensor feeds the FAST layer
([Architecture](#architecture),
[Control-flow diagram](#control-flow-diagram)) as the "Line/Path State
Estimator" input. The driver is `src/sensors/RLS08LineSensor.cpp`; the
hardware-independent interface it implements is `src/sensors/LineSensor.h`.
If your variant differs (polarity, order, analog), you change **one file**
or flip a config flag — the controller never knows. The sign convention
the controller expects (e > 0 = line RIGHT) is defined in
[Coordinate frames](#coordinate-frames-sign-conventions-and-units).

## OLED

* [ ] Confirm OLED controller (the code defaults to **SH1106**; switch to SSD1306 in ONE place: `src/ui/Display.cpp`, search `OLED_TODO`)
* [ ] Confirm SSD1306 vs SH1106 if necessary
* [ ] Confirm OLED supply voltage
* [ ] Confirm SDA pin (currently `PIN_UNASSIGNED` — TODO)
* [ ] Confirm SCL pin (currently `PIN_UNASSIGNED` — TODO)
* [ ] Confirm OLED I2C address (default 0x3C; some modules use 0x3D)

Note: the common ESP32 I2C pins GPIO 21/22 are **already used** by the
L298N (IN3/IN4), so the OLED needs other pins. The OLED is optional at
runtime: if it is absent or its pins are unassigned, the robot runs
headless with full serial diagnostics.

**Architecture reference:** the OLED is part of the **UI loop** in
`main.ino` (see [Local UI](#local-ui-oled--4-buttons)). It sits *outside*
the FAST control layer — the 200 Hz loop never waits for the display;
the UI is rate-limited to ~5 Hz and never blocks control.

## Buttons (UP / DOWN / SELECT / BACK)

* [ ] Assign UP GPIO (currently `PIN_UNASSIGNED` — TODO)
* [ ] Assign DOWN GPIO (currently `PIN_UNASSIGNED` — TODO)
* [ ] Assign SELECT GPIO (currently `PIN_UNASSIGNED` — TODO)
* [ ] Assign BACK GPIO (currently `PIN_UNASSIGNED` — TODO)
* [ ] Verify pull-up/pull-down configuration (code assumes internal pull-up, pressed = LOW)
* [ ] Verify button electrical behavior

With unassigned button pins the UI is inert (no accidental resets); the
robot still runs and the serial interface still works.

**Architecture reference:** buttons drive the same UI loop as the OLED
([Local UI](#local-ui-oled--4-buttons), `src/ui/Buttons.cpp`,
`src/ui/Menu.cpp`). The ON/OFF gate they control lives in
`PathController::setEnabled()` — see the boot safety rule at the top and
[Architecture](#architecture).

## Motors / L298N

* [ ] Verify motor polarity
* [ ] Verify left/right motor mapping (code assumes OUT1/OUT2 = LEFT, OUT3/OUT4 = RIGHT)
* [ ] Verify L298N ENA/ENB behavior
* [ ] Verify PWM behavior
* [ ] Verify motor supply voltage
* [ ] Verify common ground (ESP32, L298N logic, motor supply)
* [ ] Verify motor direction against software

Confirmed motor-driver GPIO configuration:

```text
ENA = GPIO 4    (CONFIRMED)
IN1 = GPIO 18   (CONFIRMED)
IN2 = GPIO 19   (CONFIRMED)
IN3 = GPIO 21   (CONFIRMED)
IN4 = GPIO 22   (CONFIRMED)
ENB = GPIO 23   (CONFIRMED)
```

Use `test_motors/test_motors.ino` for all motor verification —
conservative duty cycles, independent of the sensor system.

**Architecture reference:** motors are the last stage of the FAST layer
([Architecture](#architecture)): `DifferentialDrive` mixes `(v, ω)` into
wheel speeds (`v_L = v − (b/2)·ω`, see
[The mathematics §4](#the-mathematics)), and the `Motor` class is the
only place PWM duty exists. If a motor runs backwards, flip `invert` in
its config in `main.ino` — never the math (sign conventions:
[Coordinate frames](#coordinate-frames-sign-conventions-and-units)).

## IMU (MPU6050) — FUTURE, deliberately not implemented

* [ ] Future: MPU6050 integration
* [ ] Future: MPU6050 testing
* [ ] Future: IMU calibration

The IMU is **completely inactive** in this milestone: no sensing, no
fusion, no calibration, no test code. Nothing fails or blocks because
the MPU6050 is absent. Do not implement it now.

**Architecture reference:** the IMU would only ever feed the OPTIONAL
SLOW calibration layer ([Architecture](#architecture),
[Control-flow diagram](#control-flow-diagram) — "IMU (gyro z)"), which is
compiled out with `CALIBRATION_ENABLED = false`. It is never part of the
FAST control path. The abstract interface it would implement is
`src/sensors/IMUInterface.h`.

## Other unknowns (software parameters to measure)

| Item | Status | Where |
|------|--------|-------|
| Motor gearbox ratio | Unknown — irrelevant for open-loop PWM; needed for future odometry | TODO in `DifferentialDrive.cpp` |
| Wheel track | Placeholder 0.15 m — **measure** | `RobotConfig.h` |
| Actual motor voltage range | ~6–7 V stated; L298N drops 1.5–2.5 V — full-duty wheel speed **measure** | `DifferentialDrive` `maxWheelSpeed_` |
| Motor deadband | Not implemented (L298N has no feedback) — add per-motor once measured | TODO in `Motor.h` |

Search the code for `TODO(hardware)` to find every place that needs
verification before running on the physical robot.

---

# Step-by-step bring-up and usage

Follow these steps **in order**. Each hardware step maps to a checklist
item in the [Hardware & Validation TODO](#hardware--validation-todo--start-here)
above. Do not skip the electrical checks in Step 1.

**Prerequisites**

- Arduino IDE or arduino-cli with the **ESP32 Arduino core 3.x**
  installed.
- The **U8g2** library (Library Manager → "U8g2" by Oliver Kraus).
- Serial monitor at **115200 baud**.
- All three sketches compile from this repository as-is:
  `main.ino` (root), `test_sensor/test_sensor.ino`,
  `test_motors/test_motors.ino`.

## Step 1 — Electrical checks (power OFF, nothing connected to ESP32 yet)

1. Power the RLS08 and **measure its AOUT voltage** on a channel seeing
   the line and one seeing background (5 V warning above).
2. If AOUT can exceed ~3.3 V, add a voltage divider to **every channel**
   before connecting anything.
3. Confirm ESP32, L298N logic, and motor supply share a **common ground**.

## Step 2 — Sensor verification (`test_sensor.ino`)

1. Open `test_sensor/test_sensor.ino` and upload it. It is fully
   independent of the line-following system — nothing else runs.
2. Open the serial monitor (115200). The sketch prints the pin map, raw
   levels, interpreted bits (S8..S1, left-to-right), active count, and
   the weighted position/error.
3. **Polarity:** place the line under one sensor. If the "interpreted"
   bit is 0 while raw is HIGH, polarity is inverted — note it.
4. **Order:** sweep the line from the robot's right to left and check
   the bit that lights up moves S1 → S8 (Sensor 1 = rightmost).
5. **Junction:** place the robot on a cross/junction and confirm the
   junction detection line behaves as expected.
6. Record results, then update `src/config/PinConfig.h` (Sensors 7 & 8
   pins) and the `lineIsHigh` / `reverseOrder` flags in `main.ino`
   (`rls08Config()`) accordingly.

## Step 3 — Motor verification (`test_motors.ino`) — wheels OFF the ground

1. **Put the robot on a stand so the wheels cannot touch the ground.**
2. Open `test_motors/test_motors.ino` and upload it. It runs an 8-step
   sequence (left fwd/rev, right fwd/rev, both, stop, PWM sweep) at a
   conservative 25% duty.
3. Watch each step against the printed labels and verify:
   - the motor labelled LEFT is the physical left motor (OUT1/OUT2),
   - "forward" spins both wheels toward the robot's front,
   - the PWM sweep changes speed smoothly.
4. If a motor runs backwards, set `invert = true` in its config in
   `main.ino` (`leftMotorConfig()` / `rightMotorConfig()`). If left and
   right are swapped, swap the pin groups in `src/config/PinConfig.h`.

## Step 4 — Assign UI pins and flash the main firmware

1. Choose free GPIOs for the OLED (SDA/SCL — **not** 21/22, they are the
   L298N IN3/IN4) and the four buttons, and fill them in
   `src/config/PinConfig.h` (`defaultOLEDPins()`, `defaultButtonPins()`).
2. If your OLED is an SSD1306 rather than SH1106, switch the constructor
   in `src/ui/Display.cpp` (search `OLED_TODO` — one line).
3. Upload `main.ino`. Expected boot behaviour:
   - motors stay stopped (system boots OFF),
   - the OLED shows the main menu (AUTO MODE / MANUAL MODE / PID / PARAMS /
     SYSTEM / RESET),
   - the serial heartbeat runs at 115200 baud.
4. If the OLED stays blank, check the I2C address (0x3C vs 0x3D) and
   wiring; the robot keeps running headless either way.

## Step 5 — First line-following run

1. Place the robot **on the line** before turning anything on.
2. In MANUAL MODE, confirm the live sensor view: bits S8..S1, position
   and error change as you slide the robot over the line. The error
   sign must follow the convention (e > 0 = line to the RIGHT).
3. Set a **low forward speed** first: PID / PARAMS → fwd speed.
4. Turn the system ON: SYSTEM → ON. The robot starts following.
5. To stop at any time: SYSTEM → OFF (motors safely stopped), or hold
   **SELECT + BACK ~3 s** for a hard reboot. Both work from any screen.
6. If the robot steers the wrong way (away from the line), the polarity
   or channel order is wrong — go back to Step 2. Do not "fix" this by
   negating gains.

## Step 6 — Tuning and saving

1. Tune in PID / PARAMS: start with kp only (ki = kd = 0), increase
   until the robot follows, then add kd to damp oscillation, then a
   small ki if there is a steady offset. Adjust target position to
   centre the robot over the line.
2. **SELECT persists** the edited value to NVS (survives reboot);
   **BACK cancels** and restores the pre-edit value. Nothing is saved
   without an explicit SELECT.
3. RESET (menu item) clears all saved configuration and reboots with
   compiled-in defaults.

## Daily usage

Power on → system is OFF (motors stopped) → check MANUAL MODE if you
want a live sensor view → SYSTEM → ON to follow the line → SYSTEM → OFF
when done. Parameters persist across power cycles once saved with
SELECT. The serial commands
([below](#serial-diagnostics-commands)) mirror everything the UI does
and work even with no OLED/buttons attached.

---

## Hardware

| Component    | Model / value                  | Status |
|--------------|--------------------------------|--------|
| MCU          | ESP32 (Arduino framework)      | target |
| Line sensor  | Smartflex RLS08 (8-ch IR array)| driver written against digital-output behaviour (see note) |
| Motor driver | L298N dual H-bridge            | driver complete |
| Motors       | N20 geared DC                  | open-loop PWM (no encoders yet) |
| Wheel radius | ≈ 0.02 m                       | in config |
| Supply       | ≈ 6–7 V                        | affects achievable speed (TODO: measure) |
| IMU          | **unspecified**                | abstract interface only; no concrete driver |

**RLS08 note.** The RLS08's official datasheet is not present in this
repository. What the driver assumes — based on how community ESP32/AVR
line-follower projects wire this sensor — is: **8 IR channels, one
digital output per channel, HIGH = line detected**. Everything that
could differ on your board (polarity, channel order, analog vs digital
variant) is a config flag in `RLS08LineSensor::Config`, marked
`TODO(hardware)`. If your variant differs, you change **one file**
(`RLS08LineSensor.cpp`) or flip a config flag — the controller never
knows.

## Architecture

Two fully separated control layers with strictly one-way coupling:

```
FAST layer (200 Hz, micros()-scheduled, never blocks):
  LineSensor -> error estimate -> PID -> desired point velocity
             -> PointKinematics -> (v, omega) -> DifferentialDrive -> L298N -> N20

SLOW layer (~1 Hz, micros()-scheduled, optional):
  observes line error + IMU gyro -> windowed metrics -> objective J
  -> bounded coordinate descent -> proposes PID gains -> setGains()
```

The dependency direction (task §26):

```
Hardware -> Fast Controller -> Robot

Calibration -> observes controller/robot -> proposes PID parameters -> Fast Controller
```

**NOT** `Fast Controller -> Calibration -> Fast Controller`. The fast
controller never waits for, calls into, or knows about the calibrator.
The only interaction is `PathController::setGains()` — a cheap,
non-blocking write that the calibrator performs.

## Control-flow diagram

```
                     ┌──────────────────────── FAST LOOP (200 Hz) ────────────────────────┐
                     │                                                                    │
 ┌──────────┐   ┌────┴─────────┐   ┌──────────────┐   ┌────────────────┐   ┌──────────┐ │
 │ RLS08    │   │ Line/Path   │   │ Path         │   │ Point          │   │ Diff.    │ │
 │ 8-ch IR  ├──▶│ State       ├──▶│ Tracking     ├──▶│ Kinematics     ├──▶│ Drive    │ │
 │ array    │   │ Estimator   │   │ Controller   │   │ (virtual point)│   │ Mixing   │ │
 └──────────┘   │ (position,  │   │ (PID on line │   │ u -> (v,omega) │   │(v,omega) │ │
                │  confidence,│   │  error)      │   │                │   │-> wheels │ │
                │  valid)     │   └──────┬───────┘   └────────────────┘   └────┬─────┘ │
                └─────────────┘          │                                    │       │
                                        │◀── setGains() (cheap write)        │       │
                                        │                                    ▼       │
                     ┌───────────────────┴───────────── SLOW LOOP (~1 Hz) ┌───────┐     │
                     │                                                │ L298N │     │
                     │  ┌────────────┐  ┌──────────────┐  ┌─────────┐  └───────┘     │
                     │  │ Calibration│  │ Calibration │  │ PID     │        ▼        │
                     │  │ Metrics    │─▶│ Objective J  │─▶│ Calibr. │   ┌──────────┐   │
                     │  │ (windowed) │  │ (weighted)   │  │ (coord. │   │ N20      │   │
                     │  └─────▲──────┘  └──────────────┘  │ descent)│   │ motors   │   │
                     │        │                           └─────────┘   └──────────┘   │
                     │   ┌────┴─────┐                                                  │
                     │   │ IMU      │            (entire slow loop is OPTIONAL;      │
                     │   │ (gyro z) │             compiled out with                    │
                     │   └──────────┘             CALIBRATION_ENABLED = false)       │
                     └───────────────────────────────────────────────────────────────┘
```

## The mathematics

### 1. Unicycle model and the virtual control point

The robot is a unicycle:

```
ẋ = v·cos(θ)          ẏ = v·sin(θ)          θ̇ = ω
```

A **virtual control point** is defined at distance `l` in front of the
axle midpoint:

```
p_x = x + l·cos(θ)        p_y = y + l·sin(θ)
```

Differentiating gives the point velocity as a function of the body
twist:

```
[ṗx]   [  cos(θ)     −l·sin(θ) ] [v]
[ṗy] = [  sin(θ)      l·cos(θ) ] [ω]
```

The matrix determinant is `l ≠ 0`, so the **inverse mapping** is exact:

```
[v]     [ cos(θ)      sin(θ)  ] [ṗx]
[ω]   = [ −sin(θ)/l   cos(θ)/l] [ṗy]
```

This is implemented in `src/control/PointKinematics.{h,cpp}` as its own
reusable module — **not** buried in motor code.

**Body-frame shortcut used by the fast loop:** the line error is a
body-frame lateral quantity and no trustworthy heading θ exists yet
(no IMU), so the controller commands the point velocity in the *body*
frame. There the point sits rigidly at `(l, 0)`, so:

```
v = u_forward          ω = u_lateral / l
```

The full world-frame inverse is also implemented
(`worldFrameTwist`) for future use with a reliable absolute-yaw IMU.

### 2. Control law

```
e = sensorPosition − targetPosition        (e > 0 ⇒ line is RIGHT of centreline)

u_forward  = v_forward                      (cruise speed, config)
u_lateral  = −PID(e)                        (PID output = lateral point velocity [m/s])

v = u_forward,   ω = u_lateral / l          (PointKinematics)
```

Why `u_lateral = −PID(e)`: a positive error (line to the right) must
move the control point right, which is **negative body-y** (+y is
LEFT), producing `ω < 0` — the nose swings right, toward the line.

### 3. PID discretisation (`src/control/PIDController.cpp`)

```
P = Kp·e_k
I += Ki·e_k·dt_k                 (clamped to ±integralLimit)
D  = Kd·(α·de/dt|_k + (1−α)·D_filtered_prev)
u  = P + I + D                   (clamped to ±outputLimit)
```

- `dt_k` is always the **measured** elapsed time, never the nominal
  loop period.
- The derivative is low-pass filtered (`α` configurable) because raw
  differences of quantised sensor positions are noisy.
- The integral can be disabled (`integralEnabled`) while still being
  tracked internally.
- Anti-windup: symmetric clamp on the integral state.

### 4. Differential-drive mixing (`src/motion/DifferentialDrive.cpp`)

Given desired `(v, ω)` and wheel track `b`:

```
v_L = v − (b/2)·ω          v_R = v + (b/2)·ω
```

(ω > 0 = CCW = left wheel slower.) The inverse (for future odometry):

```
v = (v_R + v_L)/2          ω = (v_R − v_L)/b
```

Wheel speed [m/s] → motor command: divide by an estimated full-duty
wheel speed (config; TODO: measure on hardware). Motor commands are
normalised to [−1, 1]; PWM scaling happens only inside the `Motor`
class (hardware boundary).

### 5. Calibration objective (`src/calibration/`)

Over an evaluation window `T` (default 1 s):

```
E_tracking   = (1/N)·Σ e_k²                      (mean square error)
E_oscillation= ω_RMS/3 + peak-to-peak(e)/2       (normalised)
E_instability= line-lost fraction of the window

J = w_e·E_tracking + w_o·E_oscillation + w_s·E_instability
```

**Why tracking error must dominate (task §14):** minimising oscillation
alone has the degenerate optimum "zero gains ⇒ zero oscillation ⇒ the
robot never steers and leaves the line". With `E_tracking` in the
objective, zero gains make `J ≈ 1` (saturated error), so the optimiser
can never win by refusing to steer.

The search is deliberately simple and deterministic: **bounded
coordinate descent** — perturb one gain (kp, then ki, then kd; +step
then −step), evaluate a full window, keep or reject. All candidates are
clamped to hard bounds (`kpMin/kpMax`, …) **before** going live. The
strategy class (`PIDCalibrator`) is isolated behind
`CalibrationTarget` so the algorithm can be replaced later without
touching the PID or the controller.

## Enabling / disabling calibration

**Compile time** — `src/config/RobotConfig.h`:

```cpp
#define CALIBRATION_ENABLED false   // baseline: calibration fully off
```

With `false`:
- `main.ino` never constructs the calibrator,
- `Diagnostics` answers calibration commands with "compiled out",
- the fast controller runs purely on the baseline gains.

**Runtime** (when compiled in) — serial commands:

```
cal on       # enable supervisor (starts DISABLED even when compiled in)
cal off      # disable + restore last-known-good gains
cal reset    # reset all calibration state
cal restore  # immediately restore last-known-good gains
```

**Safety invariants** (enforced in code, not by convention):
- Candidates are clamped to hard bounds before being applied.
- `lastKnownGoodGains` are only committed after a full accepted round.
- Line lost > 0.3 s, timeout (120 s), or an invalid window ⇒ immediate
  revert to last-known-good + supervisor disables itself.
- The calibrator NEVER outputs motor commands — only `setGains()`.

## Repository layout

The Arduino build system compiles the sketch root and the `src/`
subfolder recursively — so `main.ino` lives at the repository root and
all modules live under `src/` (this is the standard Arduino multi-file
layout, not a limitation of the design):

```
main.ino                       # integration ONLY: wiring + two independent loops
                               #   (contains no control math and no driver details)

src/
├── config/
│   ├── RobotConfig.h          # ALL parameters: geometry, gains, limits, timing
│   └── PinConfig.h            # ALL pin assignments (+ TODO markers)
│
├── sensors/
│   ├── LineSensor.h           # hardware-independent line-sensor interface
│   ├── RLS08LineSensor.h/.cpp # RLS08 driver (digital 8-channel assumption)
│   ├── IMUInterface.h         # abstract IMU (model unknown — no fake driver)
│   └── MockSensors.h          # mock line sensor + mock gyro (tests/sim)
│
├── control/
│   ├── PIDController.h/.cpp   # pure PID: filtered D, anti-windup, measured dt
│   ├── PathController.h/.cpp  # the FAST loop: sensor -> error -> PID -> twist
│   └── PointKinematics.h/.cpp # virtual control-point math (own module)
│
├── motion/
│   ├── Motor.h/.cpp           # single-motor abstraction over L298N channel
│   └── DifferentialDrive.h/.cpp # (v,ω) <-> wheel mixing
│
├── calibration/               # ENTIRELY OPTIONAL (feature-flagged)
│   ├── CalibrationState.h     # explicit state machine
│   ├── CalibrationMetrics.h/.cpp # independent windowed metrics
│   └── PIDCalibrator.h/.cpp   # bounded coordinate-descent supervisor
│
├── system/
│   └── PersistentConfig.h/.cpp # NVS persistence (writes ONLY on user confirm)
│
├── ui/
│   ├── Display.h/.cpp         # U8g2 OLED wrapper (SH1106 default, SSD1306 TODO)
│   ├── Buttons.h/.cpp         # 4 debounced buttons + SELECT+BACK hard reset
│   └── Menu.h/.cpp            # menu state machine (AUTO/MANUAL/PARAMS/SYSTEM/RESET)
│
├── diagnostics/
│   └── Diagnostics.h/.cpp     # rate-limited serial commands + heartbeat
│
└── test/
    └── SelfTest.h/.cpp        # 29 unit tests (host or on-target)

test/
├── host_test.cpp              # g++ entry point for the suite
└── arduino_stubs/Arduino.h    # minimal Arduino API for host compilation

test_sensor/test_sensor.ino    # standalone sensor-verification sketch
test_motors/test_motors.ino    # standalone motor-verification sketch
```

## Building

**Arduino IDE / arduino-cli:** open/compile `main.ino` at the repository
root (the IDE compiles `main.ino` plus everything under `src/`
recursively). Requires the ESP32 Arduino core (3.x recommended; the
`Motor.cpp` LEDC calls use the 3.x API — see the TODO there for 2.x
compatibility) and the **U8g2** library.

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 .
```

Verified: compiles with both `CALIBRATION_ENABLED false` and `true`
(~25% flash, ~7% RAM). The two test sketches compile standalone with
the same FQBN.

**Host (no hardware):**

```bash
g++ -std=c++17 -Isrc -Itest/arduino_stubs \
    test/host_test.cpp \
    src/control/*.cpp src/motion/*.cpp \
    src/calibration/*.cpp src/test/SelfTest.cpp \
    -o host_test && ./host_test
```

## Testing

`src/test/SelfTest.cpp` contains 29 checks covering (task §22):

- PID: zero/± error, output saturation, error clamping, integral
  anti-windup, integral disable, derivative with variable dt, zero-dt safety
- PointKinematics: body/world point-velocity → (v, ω), degenerate-l safety
- DifferentialDrive: straight, pure rotation, general twist, inverse consistency
- CalibrationMetrics: MSE/RMS, line-lost fraction, oscillation RMS + zero crossings
- PIDCalibrator: candidate accept, candidate reject/revert, line-loss
  failure → last-known-good, timeout → last-known-good, hard-bounds clamp

Run them on the host (command above) or on-target by setting
`RUN_SELF_TEST_AT_BOOT true` in `main.ino`.

**Hardware testing** (the physical part of the test plan) is the
[step-by-step bring-up](#step-by-step-bring-up-and-usage) procedure:
`test_sensor.ino` (Step 2) and `test_motors.ino` (Step 3) are
deliberately separate sketches so each subsystem can be verified in
isolation before the full firmware runs.

## Serial diagnostics commands

115200 baud. All output is rate-limited (1 Hz heartbeat); nothing
prints inside the fast loop.

```
gains       current PID gains + integral flag
metrics     last calibration window metrics + objectives + state
sensor      raw 8-channel bits + position/confidence/valid
controller  line error, v, ω, line-lost flag
timing      loop period: target/last/min/max/avg + iteration count
cal on|off|reset|restore   calibration control
help        command list
```

## Coordinate frames, sign conventions and units

- **Body frame:** +x forward, +y LEFT, +z up (right-handed).
- **World frame:** standard right-handed x/y plane, θ measured CCW
  from +x_world.
- **ω > 0** = counter-clockwise turn (nose swings LEFT), right-hand
  rule around +z.
- **Line error `e > 0`** = line centre is to the RIGHT of the robot
  centreline (looking along +x_body).
- **Left/right motor:** positive command = forward; if a motor is
  wired backwards, flip `invert` in its config, never the math.
- **Units:** SI internally — metres, seconds, radians, rad/s, m/s.
  PWM duty and raw sensor bits exist only inside hardware drivers.
- **Control point:** `l` metres forward of the axle midpoint
  (config: `controlPointDistance`).

---

# Local UI (OLED + 4 buttons)

The OLED and four buttons provide the robot's local configuration
interface. All pin assignments are TODO until verified (see
[Hardware & Validation TODO](#hardware--validation-todo--start-here)).

```text
MAIN MENU
├── AUTO MODE    autonomous calibration: status, gains, start/stop
├── MANUAL MODE live sensor view: raw channels (S8..S1), position, error,
│                polarity flag toggle (runtime only)
├── PID / PARAMS kp, ki, kd, integral on/off, target position, fwd speed
├── SYSTEM       line-following ON/OFF (OFF = motors safely stopped)
└── RESET        clear saved config (factory defaults) + reboot
```

* **UP/DOWN** navigate / change values (auto-repeat while held).
* **SELECT** confirm / enter — this is the ONLY action that persists a
  parameter edit to NVS.
* **BACK** cancel / return — cancels an edit and restores the pre-edit value.
* **Hold SELECT + BACK ~3 s** (from any screen): **hard reset** — the
  ESP32 actually restarts (`ESP.restart()`). This is NOT the same as the
  RESET menu item (which clears saved configuration).

Persistence uses ESP32 NVS via the `Preferences` library (namespace
`fluffy`). Values are written **only** on user-confirmed edits. Missing
keys fall back to compiled-in defaults — the robot boots and runs
without any saved config or completed calibration.
