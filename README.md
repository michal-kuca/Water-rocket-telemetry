# Water Rocket Telemetry System

An embedded telemetry system for a water rocket — captures flight data in real time,
transmits it wirelessly to a ground station, and post-processes the result in MATLAB.

Built as a team project ("Galway Lander") during the first year of Engineering at the
University of Galway, Spring 2026.

---

## What it does

The on-board system measures and broadcasts the following during flight:

- **Acceleration** on three axes (ADXL335 analog accelerometer)
- **Altitude** derived from atmospheric pressure (Adafruit BMP280)
- **Temperature** (BMP280 internal sensor)
- **Derived metrics:** velocity (integrated from acceleration), force, current flight phase

Data is transmitted in real time via an HC-12 radio module (433 MHz, UART link) to a ground
station, where it is logged as CSV and later analyzed in MATLAB.

After landing, the rocket broadcasts a flight summary: maximum altitude, peak ascent and
descent acceleration, peak velocities, total flight duration, and estimated peak thrust.

---

## Architecture
┌────────────────────────────────────┐
│  ROCKET (in-flight)                │
│                                    │
│  ADXL335 ──analog (A1–A3)─┐        │
│                           │        │
│  BMP280 ───I²C (0x76)─────┤        │
│                           │        │
│                    [Arduino Nano]  │
│                           │        │
│                         UART       │
│                           │        │
│                       [HC-12] ─────┼──► 433 MHz radio
└────────────────────────────────────┘
│
▼
┌────────────────────────────────────┐
│  GROUND STATION                    │
│                                    │
│  ◄──433 MHz─── [HC-12]             │
│                    │               │
│                  UART              │
│                    │               │
│              [Laptop / PC]         │
│                    │               │
│             CSV logging            │
│                    │               │
│            MATLAB analysis         │
└────────────────────────────────────┘

---

## Key engineering features

**Finite state machine with debouncing.**
The firmware uses a four-state FSM (`PRELAUNCH`, `ASCENT`, `DESCENT`, `LANDED`).
Transitions require three consecutive condition matches before switching to prevent false
triggers from sensor noise. This is software-level debouncing of flight phase events.

**Sampling control with absolute timestamps.**
The main loop uses an absolute next-sample-time approach (`nextSampleTime += sampling_interval`)
rather than relative delays. Cycle delays don't accumulate over time, and a slow iteration is
automatically compensated for in the next cycle.

**Drift compensation.**
Integrating accelerometer data gives velocity, but integration also amplifies bias errors.
The firmware applies a multiplicative decay factor (≈0.98) to velocity each cycle to suppress
slow drift without affecting real motion.

**Moving average filter.**
All sensor outputs (accelerometer axes, altitude, pressure, temperature) are smoothed through
a ring-buffer moving average with configurable filter size.

**Calibration phase.**
On startup the system averages ten pressure and temperature readings to establish a ground
reference before launch, compensating for varying atmospheric conditions on the day.

**Noise threshold.**
Acceleration magnitudes below a configurable threshold are clamped to zero to prevent drift
accumulation from sensor noise when the rocket is stationary.

**Conditional integration.**
Velocity integration is gated by the launch state — accelerometer integration only begins
after `PRELAUNCH` is exited, preventing drift accumulation during the unknown wait time on
the launchpad.

---

## Hardware

| Component         | Role                              | Interface             |
|-------------------|-----------------------------------|-----------------------|
| Arduino Nano      | Main controller                   | —                     |
| Adafruit BMP280   | Pressure & temperature sensor     | I²C (address 0x76)    |
| ADXL335           | 3-axis analog accelerometer       | Analog (pins A1–A3)   |
| HC-12             | Wireless transceiver, 433 MHz     | UART (pins 6 = TX, 7 = RX) |
| 9 V battery       | Power supply, regulated to 5 V    | —                     |

Rocket mass during flight test: **0.25 kg**.

---

## Flight test results

The complete system was tested in a live launch on 5 April 2026. The state machine
successfully detected all four flight phases, and the in-flight summary was transmitted
correctly via HC-12 after landing.

### Key measurements

| Metric                       | Value          | Time (T+) |
|------------------------------|----------------|-----------|
| Maximum altitude             | 6.16 m         | 1.70 s    |
| Peak acceleration (ascent)   | 11.05 m/s²     | 1.44 s    |
| Peak acceleration (descent)  | 5.53 m/s²      | 2.30 s    |
| Maximum ascent velocity      | 5.00 m/s       | —         |
| Maximum descent velocity     | 4.28 m/s       | 2.60 s    |
| Total flight duration        | 2.70 s         | —         |
| Estimated peak thrust        | 2.76 N         | —         |
| Sampling rate                | 100 ms (10 Hz) | —         |

### Validation — trapezoidal integration

The MATLAB analysis integrates Y-axis velocity over the flight to estimate total vertical
distance traveled:

```matlab
distance = trapz(Timeshort, -Velocity_y);   % → 10.9 m
```

Combined with the late ascent detection offset (integration only began at ~1.5 m altitude),
this gives an estimated **12.4 m total path length**, which matches the expected 12.32 m
(twice the peak altitude, since the rocket goes up then back down). This validates the
velocity integration within the limits noted below.

### Observations and known limitations

**Premature LANDED state detection.**
Parachute deployment caused a sudden deceleration that triggered the landing condition
early via the acceleration threshold. This shortened the window for descent data collection.
A more robust approach would require sustained low acceleration over several cycles, plus
altitude rate-of-change as a secondary criterion.

**Velocity drift.**
Integration of accelerometer data accumulates bias error despite the per-cycle damping
factor. Proper sensor fusion (e.g. a Kalman filter combining accelerometer and altitude
rate) would give significantly better velocity estimates.

**Single-axis gravity compensation.**
The firmware subtracts 9.81 m/s² from the Y axis only, assuming the rocket stays vertical.
Rotation during flight breaks this assumption. A full IMU with gyroscope would allow proper
gravity compensation in the rocket reference frame.

**Sampling rate.**
10 Hz is low for a short-duration rocket flight. Faster sampling would capture finer
acceleration peaks. The rate was limited by the HC-12 baud rate (9600) and the per-sample
data payload size.

---

## What I would do differently

- **Replace ADXL335 + BMP280 with a single IMU** (e.g. MPU6050 or BNO055). Gyroscope data
  would enable full orientation tracking, and digital I²C output is significantly less noisy
  than analog ADC sampling of the ADXL335.
- **Add onboard SD card logging.** Radio packets can drop. Local storage means flight data
  survives even if the link fails.
- **Move to interrupt-driven or RTOS-based sampling** rather than a polled loop, for more
  deterministic timing under varying CPU load.
- **Replace per-cycle damping with proper sensor fusion** using altitude rate as a velocity
  correction signal — this would eliminate most of the drift without the artifacts that
  multiplicative damping introduces.

---

## Repository structure
.
├── firmware/                  # Arduino firmware for the rocket
│   └── rocket.ino
├── matlab/                    # Post-flight data analysis
│   ├── flight_analysis.m
│   ├── Testresults_New2.xls   # Live flight telemetry
│   └── DropTestData.csv       # Pre-flight drop test data
├── galway_lander_report.pdf # Project report
└── media/                     # Photos and flight footage

---

## My role in the project

This was a four-person team project. My responsibilities:

- Designed and implemented the on-board firmware in C/C++
- Designed the finite state machine and sampling architecture
- Implemented sensor calibration, moving-average filtering, and drift compensation
- Wrote **Section 4 (Arduino System)** and **Section 5 (Testing)** of the technical report,
  plus the code appendix
- Performed post-flight data analysis in MATLAB

Rocket structure, parachute design, capsule construction, and the corresponding sections of
the report (1, 2, 3, 6, 7) were the work of the other team members. See **Section 6**
(Individual Contributions) of the report for the formal team breakdown.

---

## Author

Michal Kuča — Engineering student, University of Galway.
First-year project (Spring 2026).

Contact: misazolomouce@gmail.com
