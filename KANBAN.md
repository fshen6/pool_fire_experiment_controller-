# Pool Fire Controller — Kanban Board

_Last updated: 2026-05-28_

Legend: 🔴 blocker · 🟡 needs hardware · 🟢 ready to code · ⚙️ code · 🧪 experiment · 📄 docs

---

## ✅ Done

| # | Task | Notes |
|---|------|-------|
| D1 | Hardware wiring (ESP32 + MAX485 + pump + relays + LCD) | Per `readme.md` pin map |
| D2 | Component test sketches | buttons, toggle, voltmeter, PWM→V converter, pot chain |
| D3 | Mode 1 — manual prime | Working |
| D4 | Mode 2 — constant injection | Working |
| D5 | 📄 Laser sensor comms + troubleshooting guide | `laser_sensor_comms_guide.md` |
| D6 | 🧪 Td experiment design (report + sketch) | `Td_Experiment_Report.md` + `Td_Measurement/Td_Measurement.ino` |
| D7 | 📄 Smith Predictor design plan | `Smith_Predictor_Plan.md` |
| D8 | Mode 3 module + framework design | Layered HAL → Smith → PI outer → P inner, scheduler defined |
| D9 | Confirm fuel injection logic + sensor sign convention | Sensor above/down → inject when distance > DStart |

---

## 🟡 In Progress

| # | Task | Type | Status |
|---|------|------|--------|
| P1 | Fix laser sensor — 5V/3.3V logic-level mismatch on RO line | 🟡🔴 | Root cause confirmed. Fix options: MAX3485 @ 3.3V · RO resistor divider · power module from 3.3V. **User testing on bench.** |

---

## 🔴 Blocked (waiting on P1)

| # | Task | Blocked by | Notes |
|---|------|-----------|-------|
| X1 | 🧪 Run Td experiment → extract `Td_s`, `K_MODEL`, `DELAY_SAMPLES` | P1 | Needs valid Modbus distance readings |
| X2 | 📄 Update comms guide — add Cause 6 (5V/3.3V mismatch) | — | Ready to write, just low priority until sensor is fixed |

---

## 🟢 To Do (ready to start)

| # | Task | Type | Notes |
|---|------|------|-------|
| T1 | ⚙️ Write `Mode_3/Mode_3.ino` | ⚙️ | Smith Predictor + PI outer (10 Hz) + P inner (50 Hz) + deadband + anti-windup + bumpless start + E-stop + LCD + CSV log. Inject CW when distance > DStart; drain CCW when distance < DStart − 0.1. 150% speed bump for 5 samples on error. |
| T2 | ⚙️ Raw-byte diagnostic sketch | ⚙️🧪 | Flash separately; print raw Modbus bytes to confirm `01 03 04 …` before integrating into Mode 3 |
| T3 | 📄 Add Cause 6 to `laser_sensor_comms_guide.md` | 📄 | 5V/3.3V level mismatch section with resistor divider diagram and MAX3485 recommendation |

---

## 🔵 Backlog (future / nice-to-have)

| # | Task | Notes |
|---|------|-------|
| B1 | Td speed-dependency sweep (Phase 4 of Td report) | Is Td constant across PWM values? Informs whether one `DELAY_SAMPLES` is enough |
| B2 | Serial-to-PC CSV capture pipeline for 30-min run analysis | `logger.h` already planned in module design |
| B3 | Fuel-mass scale integration | Independent ground-truth of injected volume via precision scale on storage tank |
| B4 | LCD run-screen polish | setpoint, error, V, PWM, elapsed time all on one screen |

---

## Clarification needed before coding T1

> **Q:** When readme says "decrease speed by 150%", does that mean:
> - **A** — Run at `VStart × 0.5` (half speed) when over-level
> - **B** — Run at `VStart − (VStart × 0.5)` same thing, just worded differently
>
> Assumption for now: **150% = multiplier** → over-level runs at `0.5 × VStart`, under-level at `1.5 × VStart`.

---

## Critical Path

```
P1 (fix sensor 5V/3.3V)
   └── X1 (Td experiment → Td_s, K_MODEL, DELAY_SAMPLES)
          └── T1 (Mode_3.ino with tuned Smith Predictor)
                 └── Bench tune (Kp=0.5, Ki=0.05, Kp_inner=20)
                        └── Live 30-min burn validation (3.5→4.0→4.3 V)

T2, T3 — parallel, no dependencies
```

**Next action:** Resolve P1 on bench. While waiting, T1 and T3 can be written.
