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

1. [Hardware](#hardware)
2. [Architecture](#architecture)
3. [Control-flow diagram](#control-flow-diagram)
4. [The mathematics](#the-mathematics)
5. [Enabling / disabling calibration](#enabling--disabling-calibration)
6. [Repository layout](#repository-layout)
7. [Building](#building)
8. [Testing](#testing)
9. [Serial diagnostics commands](#serial-diagnostics-commands)
10. [Coordinate frames, sign conventions and units](#coordinate-frames-sign-conventions-and-units)
11. [Unknown hardware information (TODOs)](#unknown-hardware-information-todos)

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
├── diagnostics/
│   └── Diagnostics.h/.cpp     # rate-limited serial commands + heartbeat
│
└── test/
    └── SelfTest.h/.cpp        # 29 unit tests (host or on-target)

test/
├── host_test.cpp              # g++ entry point for the suite
└── arduino_stubs/Arduino.h    # minimal Arduino API for host compilation
```

## Building

**Arduino IDE / arduino-cli:** open/compile `main.ino` at the repository
root (the IDE compiles `main.ino` plus everything under `src/`
recursively). Requires the ESP32 Arduino core (3.x recommended; the
`Motor.cpp` LEDC calls use the 3.x API — see the TODO there for 2.x
compatibility).

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 .
```

Verified: compiles with both `CALIBRATION_ENABLED false` and `true`
(~22% flash, ~7% RAM).

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

## Unknown hardware information (TODOs)

The task (§24) requires unknowns to be **explicit**, not invented:

| Item | Status | Where |
|------|--------|-------|
| RLS08 exact interface | Assumed 8× digital, HIGH=line; **verify polarity/order/variant** | `RLS08LineSensor::Config`, `PinConfig.h` |
| ESP32 pin assignments | Placeholders mirroring common wiring; **verify** | `PinConfig.h` |
| Exact IMU model | Unknown — abstract `IMUInterface` only; yaw explicitly NOT assumed | `IMUInterface.h`, `main.ino` |
| Motor gearbox ratio | Unknown — irrelevant for open-loop PWM; needed for future odometry | TODO in `DifferentialDrive.cpp` |
| Wheel track | Placeholder 0.15 m — **measure** | `RobotConfig.h` |
| Actual motor voltage range | ~6–7 V stated; L298N drops 1.5–2.5 V — full-duty wheel speed **measure** | `DifferentialDrive` `maxWheelSpeed_` |
| L298N enable/input pin arrangement | Standard ENA/IN1/IN2 + ENB/IN3/IN4 assumed — **verify** | `PinConfig.h` |
| Motor deadband | Not implemented (L298N has no feedback) — add per-motor once measured | TODO in `Motor.h` |

Search the code for `TODO(hardware)` to find every place that needs
verification before running on the physical robot.

---

# Hardware & Validation TODO

Everything below **still requires physical verification**. Nothing here
has been tested on hardware by the coding environment — compile success
is not hardware validation. Confirmed facts are marked CONFIRMED;
everything else is an open item.

## ⚠️ Electrical constraint — read before connecting the sensor

The **ESP32 GPIO/ADC inputs are NOT 5 V tolerant**. The RLS08 is
expected to be powered at **5 V**, and its actual AOUT voltage **must be
measured before connecting any channel to the ESP32**. Do **not** assume
that a 5 V-powered sensor only swings to 3.3 V. If AOUT can exceed ~3.3 V,
a voltage divider (or other protection) is **required** on every channel.

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

## Buttons (UP / DOWN / SELECT / BACK)

* [ ] Assign UP GPIO (currently `PIN_UNASSIGNED` — TODO)
* [ ] Assign DOWN GPIO (currently `PIN_UNASSIGNED` — TODO)
* [ ] Assign SELECT GPIO (currently `PIN_UNASSIGNED` — TODO)
* [ ] Assign BACK GPIO (currently `PIN_UNASSIGNED` — TODO)
* [ ] Verify pull-up/pull-down configuration (code assumes internal pull-up, pressed = LOW)
* [ ] Verify button electrical behavior

With unassigned button pins the UI is inert (no accidental resets); the
robot still runs and the serial interface still works.

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

## IMU (MPU6050) — FUTURE, deliberately not implemented

* [ ] Future: MPU6050 integration
* [ ] Future: MPU6050 testing
* [ ] Future: IMU calibration

The IMU is **completely inactive** in this milestone: no sensing, no
fusion, no calibration, no test code. Nothing fails or blocks because
the MPU6050 is absent. Do not implement it now.

---

# Local UI (OLED + 4 buttons)

The OLED and four buttons provide the robot's local configuration
interface. All pin assignments are TODO until verified (see above).

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
