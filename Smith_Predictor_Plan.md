# Smith Predictor — Detailed Design Plan
### Pool Fire Level Control System — Mode 3

---

## 1. Your Experiment in Context

Before designing the controller, it helps to understand exactly what the physics of your experiment are doing.

### 1.1 Observed Burn Rate Behaviour

From empirical observation over a 30-minute experiment set:

```
Pump voltage required to maintain fuel level:

Voltage (V)
  4.5 │                          ┌─────────────────────────
  4.3 │                     ─────┘   4.3 V (steady state)
  4.0 │           ──────────┘        4.0 V
  3.5 │ ──────────                   3.5 V
  3.0 │
      └────────────────────────────────────────────────────▶
       0      5     10     15     20     25     30      t (min)
              ↑            ↑
           Burn rate     Burn rate
           increases     plateaus
```

This tells you several things:

1. **The burn rate is not constant.** It increases over the first 15 minutes then stabilises. This is normal pool fire behaviour — the flame evolves as the fuel surface temperature rises.

2. **The steady-state pump voltage required at any time ≈ the instantaneous burn rate.** The PI controller's integral term will automatically track this drifting set point — you do not need to program the voltage schedule manually. It discovers it by itself.

3. **The Smith Predictor's job is not to handle the burn rate change.** It exists solely to remove the dead time from the control loop. The PI integral handles the slow drift in burn rate. Both components are needed, and they operate independently.

4. **Your operating range is roughly 3.5 V – 4.3 V.** The PWM-to-voltage converter must be reliable across this range. The cascade inner control loop (voltage sensor → PWM) ensures this.

---

## 2. The Full Control Architecture

The complete Mode 3 controller has three layers, each handling a distinct problem:

```
  ┌─────────────────────────────────────────────────────────────────────┐
  │                         MODE 3 CONTROLLER                           │
  │                                                                     │
  │  LAYER 1 — SMITH PREDICTOR (removes dead time from loop)            │
  │                                                                     │
  │    Real sensor y (delayed) ──► + ──► y_corrected                   │
  │                                ▲                                    │
  │                          correction                                 │
  │                    (model_now − model_delayed)                      │
  │                                                                     │
  │  LAYER 2 — PI OUTER LOOP (level control, 10 Hz)                     │
  │                                                                     │
  │    DStart ──► error = DStart − y_corrected                         │
  │               ──► Kp × error + Ki × ∫error dt                      │
  │               ──► desired_voltage V_cmd                             │
  │                                                                     │
  │  LAYER 3 — P INNER LOOP (voltage tracking, 50 Hz)                  │
  │                                                                     │
  │    V_cmd ──► voltage_error = V_cmd − V_measured                    │
  │              ──► PWM adjustment                                     │
  │              ──► actual pump PWM                                    │
  │                                                                     │
  └─────────────────────────────────────────────────────────────────────┘
```

| Layer | Solves | Runs at | Input | Output |
|---|---|---|---|---|
| Smith Predictor | Transport dead time | 10 Hz | Sensor + model | Corrected level |
| PI outer loop | Fuel level regulation, burn rate adaptation | 10 Hz | Level error | Desired voltage |
| P inner loop | PWM converter inaccuracy | 50 Hz | Voltage error | PWM duty |

---

## 3. What the Smith Predictor Does — Intuition

### 3.1 The Problem Without It

Without compensation, the closed-loop behaves like this when the level drops:

```
t=0s    Level drops below setpoint
t=0s    PI sees error → increases pump voltage
t=0s    Fuel enters the main container
                 ...Td seconds of silence...
t=Td    Fuel arrives at sensing container
t=Td    Level starts rising
t=Td    PI: "still an error!" → increases pump again
t=Td    Level overshoots setpoint
t=Td    PI: "now it's too high!" → reduces pump
        → oscillation begins
```

The problem is not the PI — it is that the PI is making decisions on information that is Td seconds out of date.

### 3.2 The Pipeline Analogy

Think of the hydraulic connection between the containers as a conveyor belt:

```
Pump injects ──► [═══════════════════════] ──► Sensing container
                   ←────── Td seconds ───────►
                   
                   Everything currently on
                   this belt is "in transit":
                   it has been pumped but not
                   yet measured.
```

At any moment, there is a known amount of fuel in transit — you sent those commands, you know exactly how much. The Smith Predictor calculates this in-transit volume and adds it to the sensor reading, giving the PI controller a **virtual current level** that includes both what the sensor sees and what is already on the way.

```
Virtual level = Sensor reading + In-transit fuel effect
             = y_measured     + correction
```

With this corrected signal, the PI controller effectively operates on a system with **zero dead time**. It can be tuned much more aggressively and will not oscillate due to delay.

### 3.3 Block Diagram

```
                         ┌──────────────────────────────────────────┐
                         │          SMITH PREDICTOR                  │
                         │                                          │
u_now (current PWM) ─────┼──► Model — no delay  ──► ŷ_now          │
                    │    │         (integrate u_now)                │
                    │    │                            │             │
                    │    │                            ▼             │
                    │    │                       ŷ_now − ŷ_delayed  │──► correction
                    │    │                            ▲             │
                    │    │                            │             │
u_delayed (from     │    │         (integrate         │             │
ring buffer) ────────────┼──► Model — with delay ─► ŷ_delayed      │
                    │    │          u_delayed)                      │
                    │    └──────────────────────────────────────────┘
                    │                                        │
                    │                                        ▼
                    │    y_real (sensor, delayed) ──► [+] ──► y_corrected
                    │                                        │
                    │                                        ▼
                    │                              ┌─────────────────┐
                    │                              │   PI CONTROLLER  │
                    │                              │                 │
                    └──────────────────────────────┤  error =        │
                                                   │  DStart −       │
                                                   │  y_corrected    │
                                                   └─────────────────┘
```

---

## 4. The Algorithm — Step by Step

### 4.1 Parameters You Will Know After the Td Experiment

| Parameter | Symbol | Source | Typical unit |
|---|---|---|---|
| Dead time | Td_s | Td experiment mean + 2σ | seconds |
| Process gain | K_MODEL | Td experiment slope / TEST_PWM | mm / PWM / s |
| Sensor noise | σ_sensor | Td experiment baseline σ | mm |
| Buffer depth | N | round(Td_s / 0.1) | samples |

### 4.2 Data Structures

```
Ring buffer:   uint8_t  pwmBuffer[N]     // circular, stores past PWM commands
               int      bufHead = 0      // write pointer

Model states:  float    modelNow = 0     // integrated with current commands
               float    modelDelayed = 0 // integrated with delayed commands

PI state:      float    integrator = 0   // pre-seeded with VStart at Button3 press
               float    DStart = 0       // reference level (mm)
               float    VStartV = 0      // starting voltage (V), from voltage sensor

Inner loop:    float    V_cmd = 0        // desired voltage from outer PI
               int      currentPWM = 0  // actual PWM output
```

### 4.3 Startup (Button 3 Pressed)

```
1. Read current distance         → DStart
2. Read current voltage sensor   → VStartV
3. Convert VStartV to PWM        → VStartPWM  (via inverse of VOLTAGE_SCALE)
4. Pre-seed PI integrator        → integrator = VStartV  (bumpless start)
5. Zero both model states        → modelNow = 0, modelDelayed = 0
6. Fill ring buffer with VStartPWM (assume steady state before press)
7. Start pump at VStartPWM
```

Pre-seeding the integrator at VStartV means the pump starts at exactly the voltage the user set with the potentiometer, with no step jump. The PI then makes only small corrections from that baseline.

Filling the ring buffer with VStartPWM at startup is important. Without this, the buffer contains zeros, and during the first Td seconds the model would think no fuel was being pumped previously — causing a false over-correction spike at startup.

### 4.4 Every 100 ms — Smith Predictor Update (10 Hz)

```
Step 1 — Read sensor
    success = readDistance(y_real)
    if not success → skip this tick

Step 2 — Retrieve delayed command from ring buffer
    u_delayed = pwmBuffer[bufHead]          // oldest entry = Td seconds ago

Step 3 — Advance model states
    modelNow     += K_MODEL × currentPWM × 0.1
    modelDelayed += K_MODEL × u_delayed  × 0.1

Step 4 — Store current PWM into ring buffer (overwrites the oldest slot)
    pwmBuffer[bufHead] = currentPWM
    bufHead = (bufHead + 1) % N

Step 5 — Compute corrected level
    correction   = modelNow − modelDelayed
    y_corrected  = y_real + correction

Step 6 — PI outer loop
    error = DStart − y_corrected
    if |error| < deadband (= 3 × σ_sensor):   error = 0   // do not chase noise
    integrator += Ki × error × 0.1
    integrator  = clamp(integrator, V_MIN, V_MAX)          // anti-windup
    V_cmd = integrator + Kp × error

Step 7 — Clamp V_cmd to safe operating range
    V_cmd = clamp(V_cmd, 0.0, V_MAX)

Step 8 — Pass V_cmd to inner loop (runs at 50 Hz, see Section 4.5)
```

### 4.5 Every 20 ms — Inner P Loop (50 Hz Voltage Tracking)

```
Step 1 — Read voltage sensor
    V_measured = readVoltage()

Step 2 — Voltage error
    V_error = V_cmd − V_measured

Step 3 — Adjust PWM
    PWM_adjust  = Kp_inner × V_error
    currentPWM  = clamp(currentPWM + PWM_adjust, 0, 255)

Step 4 — Write to pump
    ledcWrite(PIN_PWM, currentPWM)
```

This inner loop runs 5× faster than the outer loop. It continuously nudges the PWM until the measured voltage matches whatever the outer PI has commanded — making the outer loop behave as if it has a perfect, linear voltage source regardless of the converter's non-linearity.

### 4.6 Model State Reset (Every 60 Seconds)

The model states `modelNow` and `modelDelayed` are pure integrators. Over a 30-minute experiment, any small error in K_MODEL accumulates. Since we only use their **difference**, absolute drift doesn't matter — but as a precaution, reset both to zero every 60 seconds:

```
if (millis() - lastModelResetMs > 60000) {
    modelNow     = 0;
    modelDelayed = 0;
    lastModelResetMs = millis();
}
```

After the reset, the correction spikes briefly (the buffer still holds the last Td seconds of commands), then settles correctly within one Td window. The PI's integrator absorbs any transient.

---

## 5. How the Algorithm Handles Your Changing Burn Rate

This is the most important practical consideration for your specific experiment.

### 5.1 What Happens at Each Phase Transition

**t = 0 min — Experiment starts, VStart = 3.5 V**

```
PI integrator ≈ 3.5 V  (pre-seeded from potentiometer)
Smith correction ≈ constant (steady state)
System: balanced, level stable
```

**t ≈ 8 min — Burn rate starts increasing**

```
Fuel level begins to drop slightly (more fuel burning than being pumped)
y_corrected dips below DStart
error becomes positive
PI integrator slowly winds up: 3.5 → 3.6 → 3.7 → ...
Pump voltage increases automatically
```

**t = 10 min — System reaches 4.0 V**

```
PI integrator has found 4.0 V as the new balance point
Level stabilised at DStart again
Smith Predictor: correction adapts to new steady-state pump speed
No human intervention needed
```

**t = 15 min — System reaches 4.3 V, burn rate stabilises**

```
PI integrator settles at 4.3 V
Integrator stops winding (error ≈ 0)
System runs at 4.3 V for remaining 15 minutes
```

### 5.2 Visualising the PI Integrator Over 30 Minutes

```
V_cmd (V)
  4.5 │                              ┌───────────────────────
  4.3 │                         ─────┘  integrator settled
  4.0 │              ───────────┘
  3.8 │         ─────┘
  3.5 │ ─────────
  3.0 │
      └─────────────────────────────────────────────────────▶
       0      5      10     15     20     25     30      t (min)
       
  The PI integral "discovers" the required voltage by itself.
  No pre-programmed schedule is needed.
```

### 5.3 The Smith Predictor During Burn Rate Changes

During the burn rate transition, the pump speed is changing. This means:
- `model_now` is integrating a higher (and changing) PWM
- `model_delayed` is integrating the lower PWM from Td seconds ago
- `correction` is larger during the transition (more fuel in transit than at steady state)

This is correct and expected behaviour. The Smith Predictor is telling the PI: "more fuel than usual is already on the way — factor that in." This prevents the PI from over-pumping during the transition.

---

## 6. ESP32 Memory and Timing

### 6.1 Ring Buffer Size

The ring buffer must store Td / 0.1 samples of 8-bit PWM values.

| Td (s) | Buffer size | Memory |
|---|---|---|
| 2 s | 20 bytes | 20 B |
| 5 s | 50 bytes | 50 B |
| 10 s | 100 bytes | 100 B |
| 30 s | 300 bytes | 300 B |

Even a 30-second dead time uses only 300 bytes. The ESP32 has 320 kB of RAM. Memory is not a concern.

### 6.2 Timing Budget Per Tick (100 ms window)

| Task | Typical duration |
|---|---|
| RS-485 Modbus read (8 byte TX + 9 byte RX at 9600 baud) | ~20 ms |
| Smith Predictor update | < 1 ms |
| PI outer loop update | < 1 ms |
| LCD update (every 150 ms) | ~5 ms |
| Total | ~26 ms |

The 10 Hz tick has a 100 ms window. With ~26 ms of work, there is **74 ms of idle time per tick**. The inner voltage loop (50 Hz, every 20 ms) runs in the remaining time without conflict.

### 6.3 Non-Blocking Task Schedule

```
loop() runs as fast as possible, millisecond-level checks:

  every 100 ms  → readDistance() + Smith update + PI outer loop
  every  20 ms  → readVoltage()  + inner P loop
  every 150 ms  → LCD update
  every 500 ms  → Serial data log
  every  60 s   → model state reset
  always        → E-stop check (no delay, highest priority)
```

---

## 7. Complete Parameter Reference

### 7.1 Parameters from Td Experiment (fill in after running)

```cpp
// --- FROM Td EXPERIMENT ---
const float Td_s          = 0.0;   // mean Td + 2*sigma (seconds)
const int   BUFFER_SIZE   = (int)roundf(Td_s / 0.1f);   // at 10 Hz
const float K_MODEL       = 0.0;   // mm / PWM / s  (from slope / TEST_PWM)
const float SIGMA_SENSOR  = 0.0;   // mm  (noise floor from baseline phase)
const float DEADBAND      = SIGMA_SENSOR * 3.0f;         // mm
```

### 7.2 System Parameters (from hardware / calibration)

```cpp
// --- HARDWARE ---
const float V_MAX         = 7.8f;      // max converter output voltage (V)
const float V_MIN         = 0.0f;
const float VOLTAGE_SCALE = 5.305f;    // empirical voltage sensor scaling factor
const float ADC_REF       = 3.3f;
const int   ADC_MAX       = 4095;

// Relay
const int   RELAY_ON  = HIGH;
const int   RELAY_OFF = LOW;
```

### 7.3 Tuning Parameters (set empirically)

```cpp
// --- PI OUTER LOOP (level → voltage) ---
float Kp_outer = 0.5f;    // V per mm of error  — start here, tune up/down
float Ki_outer = 0.05f;   // V per mm*s of accumulated error — start small

// --- P INNER LOOP (voltage → PWM) ---
float Kp_inner = 20.0f;   // PWM counts per volt of error — start here
```

### 7.4 Suggested Tuning Procedure

Run in this order — do not skip steps:

```
Step 1: Set Ki_outer = 0,  Kp_outer = 0.2
        Run Mode 3. Observe: level should move toward DStart slowly.
        If no movement: increase Kp_outer.
        If oscillation: decrease Kp_outer.
        Target: level settles to within 1 mm of DStart with no oscillation.

Step 2: Increase Kp_outer until level settles in ~10 s with minimal overshoot.
        Record this as Kp_outer_final.

Step 3: Set Ki_outer = Kp_outer_final / 50.
        Run Mode 3. Observe: integrator should slowly eliminate any residual
        offset over 30–60 seconds.
        If oscillation with period > 30 s: reduce Ki_outer.

Step 4: Tune Kp_inner (inner voltage loop).
        With pump running at a fixed V_cmd, watch the voltage sensor on LCD.
        Kp_inner too low: measured voltage lags V_cmd.
        Kp_inner too high: voltage oscillates rapidly.
        Target: V_measured tracks V_cmd within 0.1 V in under 1 second.
```

---

## 8. Expected Behaviour During a 30-Minute Run

### 8.1 Timeline

```
t = 0:00  Button 3 pressed.
          DStart locked in. VStart = potentiometer reading (≈ 3.5 V).
          PI integrator pre-seeded at 3.5 V. Ring buffer pre-filled.
          Pump starts at 3.5 V. Smith Predictor active.
          
          LCD:  D:47.18mm  dE:+0.00
                V:3.50V    RUN

t = 0:00  to 8:00 — Stable phase
          Burn rate matches pump rate. Error ≈ 0.
          PI integrator holds at 3.5 V.
          Correction term constant (steady state).
          
t ≈ 8:00  Burn rate begins to increase.
          Level drops slightly. y_corrected < DStart.
          Error becomes positive. PI integrator begins to wind up.
          Pump voltage slowly increases: 3.5 → 3.6 → 3.7 ...
          
          LCD:  D:47.10mm  dE:+0.08
                V:3.72V    RUN+

t = 10:00 PI integrator has settled at ~4.0 V.
          Level back at DStart. Error ≈ 0.
          
t ≈ 12:00 Burn rate increases again.
          Repeat adaptation: 4.0 → 4.1 → 4.2 → 4.3 V.
          
t = 15:00 PI integrator settles at 4.3 V. Burn rate plateaus.
          
t = 15:00 to 30:00 — Steady phase
          System holds 4.3 V. Error ≈ 0.
          Correction term constant.
          Smith Predictor quiescent.
          
t = 30:00 Button 3 pressed again → experiment ends.
          Final summary: total runtime, mean error, max correction, etc.
```

### 8.2 LCD Layout During Run

```
Line 1:  D: 47.18mm  e:+0.02      (current distance, error in mm)
Line 2:  V: 3.86V   RUN  *        (* flashes every 400ms while running)
```

When correction is significant (|correction| > 0.5 mm), show it:
```
Line 1:  D: 47.18  C:+0.34mm
Line 2:  V: 3.86V  RUN  *
```

### 8.3 Serial Log During Run (for post-experiment analysis)

```
timestamp_ms, dist_mm, y_corrected, correction, error, V_cmd, V_measured, PWM
0,            47.182,  47.182,      0.000,       0.000, 3.50,  3.49,       114
100,          47.183,  47.185,      0.002,      -0.003, 3.50,  3.50,       114
...
480200,       47.095,  47.131,      0.036,       0.051, 3.58,  3.57,       117
...
900000,       47.179,  47.181,      0.002,       0.001, 4.01,  4.00,       131
```

Plotting `y_corrected` vs time will show a much smoother signal than `dist_mm` alone. Plotting `V_cmd` over time gives you the burn rate history for your thesis data.

---

## 9. Limitations and Failure Modes

### 9.1 K_MODEL Inaccuracy

If K_MODEL is measured at one pump speed and the system operates at a different speed, the correction term will be slightly wrong. This matters most during the burn rate transitions when pump speed is changing.

**Effect:** Partial dead time compensation — still much better than no Smith Predictor.

**Mitigation:** Run the Td experiment at 3.5 V, 4.0 V, and 4.3 V and average the K_MODEL values. If they differ by less than 15%, use the average. If they differ more, consider a speed-dependent K_MODEL lookup table.

### 9.2 Td Variability

If the connecting pipe has variable resistance (partial blockage, air bubble, temperature-dependent viscosity), Td will vary between experiments. Using `Td_conservative = Td_mean + 2σ` ensures you always compensate for at least as long as the actual delay.

**Effect of Td_estimate > Td_actual:** Slight over-correction — controller predicts a bit more fuel in transit than is actually there. PI integrator compensates. System is stable.

**Effect of Td_estimate < Td_actual:** Under-correction — some residual oscillation remains, but much less than with no Smith Predictor.

### 9.3 Pump Failure During Run

If the pump stops unexpectedly (relay fault, power loss):
- Ring buffer fills with zeros over the next Td seconds
- `modelNow` stops growing; `modelDelayed` continues growing from past commands
- `correction` decreases toward zero correctly
- `y_corrected` → `y_real` (Smith Predictor gracefully degrades to raw sensor)
- PI integrator sees a large error, tries to increase voltage
- E-stop or timeout logic should catch this

### 9.4 Large Disturbances (Flame Flare-Up)

A sudden large increase in burn rate (e.g., flame flare) will cause a fast level drop. The Smith Predictor does not handle sudden disturbances faster than the outer PI can respond. In this case:

- The PI's proportional term gives an immediate large correction
- The integral term continues adapting
- The Smith Predictor ensures the correction response arrives at the sensor promptly

For extreme disturbances, a feed-forward term based on flame intensity could help — but this requires additional flame sensing hardware.

---

## 10. What to Implement — Checklist

Before writing Mode 3:

- [ ] Run Td experiment → obtain `Td_s`, `K_MODEL`, `SIGMA_SENSOR`
- [ ] Verify K_MODEL at 3 voltage levels (3.5 V, 4.0 V, 4.3 V)
- [ ] Confirm ring buffer size fits in ESP32 DRAM (always yes — see Section 6.1)
- [ ] Confirm inner loop (voltage sensor → PWM) is working correctly in isolation

When writing Mode 3:

- [ ] Pre-fill ring buffer with VStartPWM at Button 3 press
- [ ] Pre-seed PI integrator with VStartV at Button 3 press
- [ ] Implement 10 Hz outer loop: Smith correction → PI → V_cmd
- [ ] Implement 50 Hz inner loop: voltage sensor → PWM
- [ ] Add 60-second model state reset
- [ ] Add deadband (= 3 × SIGMA_SENSOR)
- [ ] Add anti-windup clamp on PI integrator
- [ ] Add E-stop (highest priority, no debounce)
- [ ] Add CSV serial logging (for post-experiment analysis in thesis)
- [ ] Add LCD display: distance, error, voltage, correction, status

---

## 11. Summary — How Everything Fits Together

```
┌─────────────────────────────────────────────────────────────────────────┐
│  INPUTS                    ALGORITHM              OUTPUTS               │
│                                                                         │
│  Laser sensor  ─────────►  Smith Predictor  ────► y_corrected (mm)     │
│  (10 Hz)           +       (dead time removed)                          │
│                    │                                    │               │
│  Ring buffer ──────┘            PI outer loop  ◄────────┘               │
│  (past PWM,        │            (10 Hz)                                 │
│   depth N)         │                 │                                  │
│                    │                 ▼                                  │
│  Voltage sensor ──────────►  P inner loop  ──────────► PWM output      │
│  (50 Hz)                     (50 Hz)                                    │
│                                                                         │
│  Potentiometer ──► VStart (locked at Button 3 press)                   │
│  Distance sensor ► DStart (locked at Button 3 press)                   │
│                                                                         │
│  E-stop ──────────────────────────────────────────► pump off (always)  │
└─────────────────────────────────────────────────────────────────────────┘

Key values from Td experiment:
  Td_s       → ring buffer depth (N samples)
  K_MODEL    → model integrator gain
  SIGMA_SENSOR → deadband width

Key tuning values (empirical):
  Kp_outer, Ki_outer → level response speed and steady-state accuracy
  Kp_inner           → voltage tracking speed
```
