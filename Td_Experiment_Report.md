# Transport Dead Time (Td) Characterisation Experiment
### Pool Fire Level Control System — ESP32 Controller

---

## 1. Introduction

### 1.1 Background

In this pool fire experiment, liquid fuel is injected into a main container by a peristaltic pump. The fuel level is not monitored directly in the main container but in a **secondary container hydraulically connected** to the main one. Because these two containers communicate through a pipe, any change in fuel level caused by injection does not appear instantaneously at the laser distance sensor.

This delay between the moment of injection and the sensor's first detectable response is called the **transport dead time**, denoted **Td**.

### 1.2 Why Dead Time Matters for Control

A feedback controller such as the PI controller planned for Mode 3 relies on prompt sensor feedback to make corrections. When a dead time is present:

- The controller applies a correction (pumps more fuel)
- The sensor does not respond immediately
- The controller, seeing no change, applies *more* correction
- After Td seconds, the full effect of all accumulated corrections arrives at once
- The level overshoots the setpoint, the controller reverses, and oscillation begins

The severity of this instability grows with the ratio of **Td / process time constant**. For slow pool fire experiments, Td of even a few seconds can cause significant hunting if the controller is not compensated.

### 1.3 Purpose of this Experiment

This experiment precisely characterises Td so that it can be accounted for in the Mode 3 control algorithm (Smith Predictor or conservative PI detuning). It also extracts the **process gain K** (how fast the level rises per unit of pump output), which is required for the Smith Predictor internal model.

### 1.4 System Schematic

```
  ┌───────────────────┐          ┌──────────────────────┐
  │   MAIN CONTAINER  │          │  SENSING CONTAINER   │
  │                   │          │                      │
  │  Fuel burning ↓   │          │   Laser sensor       │
  │                   │◄────────►│   above fuel level   │
  │  Pump injects ↑   │  pipe    │                      │
  │                   │          │                      │
  └───────────────────┘          └──────────────────────┘
         ▲
    Peristaltic pump
    (controlled by ESP32)
```

The hydraulic connection between the two containers introduces a transport delay: fuel injected into the main container must flow through the connecting pipe before raising the level in the sensing container.

---

## 2. Objectives

1. Measure the transport dead time **Td** (seconds) at the typical operating pump speed
2. Quantify the sensor **noise floor** (σ, in µm) to confirm measurement reliability
3. Extract the **process gain K** (mm/s per PWM count) from the step response slope
4. Assess **repeatability** across multiple trials
5. (Optional) Determine whether Td varies with pump speed

---

## 3. Equipment

| Item | Specification |
|---|---|
| Laser distance sensor | STUOB SDL Series, RS-485 Modbus RTU, 1 µm resolution |
| Controller | ESP32 WROOM |
| RS-485 transceiver | MAX485 module |
| Peristaltic pump | Variable speed via PWM-to-voltage converter |
| Relay module | Relay1 (D19): pump start/stop; Relay2 (D25): direction |
| I2C LCD | 16×2, address 0x27 |
| Data logging | Arduino Serial Monitor → CSV export |
| Analysis software | Excel / MATLAB / Python (any) |

---

## 4. Definitions

| Symbol | Unit | Meaning |
|---|---|---|
| Td | s | Transport dead time: time from pump activation to first sensor response |
| D_baseline | mm | Mean sensor reading during the stationary baseline period |
| σ | mm | Standard deviation of sensor readings (noise floor) |
| D_threshold | mm | Detection threshold = D_baseline + 3σ |
| K | mm / (PWM·s) | Process gain: rate of level rise per unit pump output after Td |
| VStart | PWM | Pump command at which Td is measured (match your Mode 3 operating value) |

---

## 5. Pre-Test Checklist

Complete **every item** before starting. Check each box before proceeding.

- [ ] **Flame is extinguished** — no burning during this test
- [ ] Fuel level in both containers is at the **intended operating level** (the level Mode 3 will maintain)
- [ ] Connecting pipe is **fully primed** — no air bubbles (run pump briefly in CCW to flush if needed, then wait 60 s)
- [ ] Pump has been **OFF for at least 60 seconds** — surface waves and pipe pressure fully settled
- [ ] Serial Monitor open at **115200 baud**, ready to log
- [ ] Serial Monitor **timestamp enabled** (or use the sketch's own timestamp column)
- [ ] `TEST_PWM` in the sketch is set to your **intended VStart value**
- [ ] RS-485 wiring verified: TX2 → B, RX2 → A, D4 → RE/DE
- [ ] Sensor giving valid readings (distance displays on LCD before starting)

---

## 6. Experimental Procedure

### Phase 1 — Noise Characterisation (Automated, 30 seconds)

**Purpose:** Establish the stationary noise floor of the sensor and compute the detection threshold.

1. Confirm all pre-test conditions are met
2. Open Serial Monitor (115200 baud). Enable "Save to file" if available, or copy-paste output afterwards
3. Press **BTN1** on the controller
4. The sketch automatically samples the sensor at 10 Hz for **30 seconds** with the pump OFF
5. At the end, the sketch prints and displays:
   - `D_baseline` (mean distance, mm)
   - `σ` (standard deviation, mm)
   - `D_threshold = D_baseline + 3σ`

**Do not touch anything during these 30 seconds.** Vibrations from walking near the rig will inflate σ.

**Pass/fail criterion:** σ < 0.05 mm. If σ ≥ 0.05 mm, investigate vibration sources before continuing. The sensor resolution is 1 µm (0.001 mm); a noise floor larger than 0.05 mm indicates external vibration, not sensor noise.

---

### Phase 2 — Step Response Test (Automated, up to 90 seconds)

**Purpose:** Apply a known step input to the pump and observe when the sensor first responds.

1. After the baseline completes, the sketch automatically counts down 3 seconds, then:
   - Turns **Relay1 ON** (pump starts)
   - Sets **PWM = TEST_PWM** (CW direction, Relay2 OFF)
   - Records the exact start timestamp: **t_start**
2. The sketch samples the sensor at 10 Hz and logs every reading
3. Detection criterion: **3 consecutive readings** above `D_threshold`
   - Using 3 consecutive readings prevents a single noise spike from false-triggering
   - The dead time is measured to the **first** of these 3 readings: **t_detect**
4. On detection:
   - Pump turns OFF immediately
   - LCD displays: `Td = X.XX s  Trial N`
   - Serial prints: the measured Td and the full data log

**If no detection within 90 seconds:** The sketch reports a timeout. Check:
- Pump is actually running (listen for it, check voltage sensor)
- RS-485 sensor is returning valid readings
- The threshold is not set unrealistically high (redo baseline if level shifted)

---

### Phase 3 — Repeat Trials

**Purpose:** Assess repeatability. Td has natural variability due to fluid dynamics.

1. After a trial completes, **wait until the level fully stabilises** — this typically takes 1–3 minutes after the pump stops
2. Confirm the level is stationary (watch LCD distance reading for no drift)
3. Press **BTN1** to begin the next trial (new 30 s baseline + step)
4. Repeat for **5 trials** minimum
5. After Trial 5, the sketch automatically computes and prints:
   - Mean Td
   - Standard deviation of Td across trials
   - Min and max Td
   - Recommended conservative Td = mean + 2σ (for use in the Smith Predictor)

---

### Phase 4 (Optional) — Speed Dependency

**Purpose:** Determine whether Td varies significantly with pump speed.

Repeat the full procedure (5 trials each) at:
- `TEST_PWM = 64`  (25% of max)
- `TEST_PWM = 128` (50% of max) ← your primary operating value
- `TEST_PWM = 192` (75% of max)
- `TEST_PWM = 255` (100%)

Change `TEST_PWM` in the sketch and re-upload for each set. If Td varies by more than 20% across speeds, use the **highest Td value** (most conservative) in the Smith Predictor to ensure stability across all operating conditions.

---

## 7. Data Collection Format

The sketch outputs CSV data to Serial in this format:

```
timestamp_ms,distance_mm,phase
0,47.1820,BASELINE
100,47.1815,BASELINE
...
30000,47.1822,STEP
30100,47.1819,STEP
...
35200,47.1930,STEP,ABOVE(1)
35300,47.2010,STEP,ABOVE(2)
35400,47.2150,STEP,ABOVE(3)   ← detection here, Td = 5.40 s
```

Copy the Serial output into a `.csv` file and open in Excel/Python/MATLAB for plotting.

---

## 8. Expected Outcomes and Sample Graphs

### Figure 1 — Noise Floor Characterisation

During the 30-second stationary baseline, the sensor should show small, random fluctuations around a stable mean. A good result (low-vibration environment, good sensor mounting) looks like this:

```
Distance (mm)
  47.200 │                                                    
  47.195 │    ·       ·               ·                       
  47.190 │  ·   · · ·   · · ·   · ·    · · ·   ·   · ·   ·  
  47.185 │·           ·       · ·         ·   ·   ·     ·    ← mean
  47.182 │────────────────────────────────────────────────── 
  47.178 │                  ·                 ·              
  47.175 │          ·                                         
         └──────────────────────────────────────────────────▶
          0       5       10      15      20      25      30
                                                         t (s)

  D_baseline = 47.182 mm
  σ          = 0.005 mm  (5 µm — good result)
  Threshold  = 47.182 + 3(0.005) = 47.197 mm
```

**Warning sign:** If the noise looks like a slow drift rather than random scatter, the level is still settling — wait longer before restarting the baseline.

---

### Figure 2 — Single Step Response (Good Result)

A clean result shows a flat baseline, followed by a flat period after pump start (the dead time), then a ramp up as the injected fuel arrives:

```
Distance (mm)
  47.380 │                                            ╭──────
  47.340 │                                        ╭───╯      
  47.300 │                                    ╭───╯           
  47.260 │                                ╭───╯              
  47.220 │                            ╭───╯                  
  47.200 │─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─╭╯─ ─ ─ ─  D_threshold 
  47.182 │· · · · · · · · · · · · · ·╯ · · · ·  D_baseline  
         └──────────────┬─────────────┬──────────────────────▶
          0             │             │                    t (s)
                     t_start       t_detect
                      Pump ON       Detection
                        ├────  Td  ──┤
                                  
  Td = t_detect - t_start  (example: 5.4 s)

  Slope after Td: K_process = ΔD / Δt  (example: 0.040 mm/s)
  K_MODEL = K_process / TEST_PWM       (example: 0.040/128 = 0.000313 mm/PWM/s)
```

**Note:** The slope after Td is the process gain. Extract it by fitting a straight line to the rising portion of the curve (from t_detect to where the curve begins to flatten). This value feeds directly into the Smith Predictor as `K_MODEL`.

---

### Figure 3 — Trial-to-Trial Repeatability

With 5 trials, you expect small variability due to fluid dynamics in the connecting pipe. A good result:

```
Td (s)
  9 │                                            
  8 │                                            
  7 │         ████                               
  6 │    ████ ████ ████      ████           mean ─ ─
  5 │    ████ ████ ████ ████ ████  ─────────────────
  4 │    ████ ████ ████ ████ ████               
  3 │    ████ ████ ████ ████ ████               
  2 │    ████ ████ ████ ████ ████               
  1 │    ████ ████ ████ ████ ████               
  0 └─────┴────┴────┴────┴────┴──────────────────▶
         T1   T2   T3   T4   T5
         
  Example: Td = [5.3, 5.7, 5.1, 5.5, 5.4] s
  Mean  = 5.40 s
  σ_Td  = 0.22 s
  Range = 5.1 – 5.7 s  (variation < 12%: good repeatability)
  
  Use for Smith Predictor: Td_conservative = 5.40 + 2(0.22) = 5.84 s ≈ 5.8 s
```

**Warning sign:** If variation > 30% (e.g. 3 s to 7 s), the connecting pipe flow is not consistent — check for partial blockages or air pockets.

---

### Figure 4 — Process Gain Extraction

Zoom into the period immediately after t_detect. The level should rise approximately linearly before slowing down as the containers re-equilibrate:

```
ΔDistance from baseline (mm)
  2.50 │                                  ╭──────────────
  2.00 │                             ╭────╯              
  1.50 │                        ╭────╯                  
  1.00 │                   ╭────╯   ← fit this region   
  0.50 │              ╭────╯        slope = K_process    
  0.10 │─ ─ ─ ─ ─ ─ ─╯─ ─ ─ ─  threshold               
  0.00 │──────────────                                   
       └──────────────┬──────────────────────────────────▶
        0            Td                               t (s)
        
  Fit linear regression to the first 10–20 s after t_detect.
  K_process = slope (mm/s) at TEST_PWM = 128
  K_MODEL   = K_process / 128  (mm per PWM per second)
```

---

### Figure 5 — Td vs Pump Speed (Optional, Phase 4)

If Td decreases significantly at higher pump speeds, the Smith Predictor must use the most conservative (longest) Td:

```
Td (s)
  12 │  ×                                         
  10 │       ×                                     
   8 │                                             
   6 │             ×  ─ ─ ─ ─ ─ ─  mean (PWM=128) 
   5 │                    ×                        
   4 │                         ×                   
   3 │                                             
     └──────┬──────┬──────┬──────┬──────┬──────────▶
            64     96    128    160    192    255
                               ↑                  PWM
                           VStart (typical)
                           
  If curve is flat: Td is independent of speed — good.
  If Td drops sharply at high speeds: use Td at your VStart, 
  and ensure Mode 3 does not command pump much below VStart.
```

---

## 9. Analysis Method

### Step 1 — Compute Td for each trial
```
Td_i = t_detect_i − t_start_i   (already computed by sketch)
```

### Step 2 — Summary statistics
```
Td_mean  = mean(Td_1 ... Td_N)
Td_sigma = std(Td_1 ... Td_N)
Td_conservative = Td_mean + 2 × Td_sigma
```

### Step 3 — Process gain
From the post-detection rising portion of any trial:
```
K_process = linear slope of ΔDistance vs time  (mm/s)
K_MODEL   = K_process / TEST_PWM               (mm / PWM / s)
```

### Step 4 — Enter into Mode 3
```cpp
// In Mode 3 Smith Predictor:
const float Td_s    = Td_conservative;          // from this experiment
const float K_MODEL = K_process / TEST_PWM;     // from this experiment
const int   DELAY_SAMPLES = round(Td_s / 0.1);  // at 10 Hz sampling
```

---

## 10. Results Table

Fill in after completing the experiment:

| Parameter | Value | Unit |
|---|---|---|
| Test date | | |
| Fuel type | | |
| Operating fill level | | mm |
| Test PWM (VStart) | | PWM (0–255) |
| Baseline mean D_baseline | | mm |
| Sensor noise σ | | µm |
| Detection threshold | | mm |
| Td Trial 1 | | s |
| Td Trial 2 | | s |
| Td Trial 3 | | s |
| Td Trial 4 | | s |
| Td Trial 5 | | s |
| **Td mean** | | **s** |
| **Td σ** | | **s** |
| **Td conservative** | | **s** |
| K_process | | mm/s |
| K_MODEL | | mm/PWM/s |
| Notes | | |

---

## 11. Safety Notes

- **Never run this experiment with the flame active.** The fuel level must remain stable under the only influence of the pump to isolate Td.
- Ensure the pump is running in **CW (forward / injection) direction** before the test. Relay2 must be OPEN (LOW). Confirm this by observing the fuel level rising (never falling) after the step.
- If the E-stop button is pressed at any time, the sketch halts all pump activity immediately.
- If the detected level change is in the **wrong direction** (level falls instead of rises), the pump is running CCW — stop immediately, check Relay2 wiring.

---

## 12. Application to Mode 3 Controller

The values from this experiment directly configure the Mode 3 control system:

| Experiment output | Used in |
|---|---|
| Td_conservative | Smith Predictor buffer depth; or PI detuning rule Ti ≥ 5 × Td |
| K_MODEL | Smith Predictor internal model gain constant |
| σ (noise floor) | PI controller deadband = 3σ (don't chase noise) |
| Td_sigma | Confidence in Smith Predictor — if σ_Td is large, use more conservative Td |
