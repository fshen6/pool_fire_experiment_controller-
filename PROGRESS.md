# Mode 3 — Progress Tracker

_Update this file as you complete each task._

---

## H — Hardware

| Status | # | Task | Notes |
|--------|---|------|-------|
| ✅ | H1 | Fix laser sensor 5V/3.3V mismatch on RO line | Resolved by swapping MAX485 → MAX3485 (3.3V native). Sensor confirmed communicating. |
| ⬜ | H2 | Verify relay wiring: Relay1 HIGH=run, Relay2 LOW=CW inject | Wrong wiring = pump drains instead of injects |
| ⬜ | H3 | Confirm E-stop switch: normally-closed to GND on D27 | Code activates E-stop when D27 HIGH — wrong wiring = stuck in E-stop |

---

## C — Calibration

> Requires H1 complete first.

| Status | # | Task | Notes |
|--------|---|------|-------|
| ⬜ | C1 | Flash `Td_Measurement.ino`, run step-response experiment | Outputs `Td_s` and `K_MODEL` on serial — record both values |
| ⬜ | C2 | Edit `Mode_3.ino`: fill in `Td_s` and `K_MODEL`, set `SMITH_PREDICTOR_ENABLED 1` | Found at top of file in CONFIGURATION section |

---

## V — Bench Verification

> No fire. Run these before any live experiment.

| Status | # | Task | What to check |
|--------|---|------|---------------|
| ⬜ | V1 | Flash Mode 3, open serial monitor at 115200 baud | CSV header prints → code is running |
| ⬜ | V2 | In IDLE: turn pot → watch PWM and voltage change on LCD | Confirms pot → PWM → converter → voltage sensor chain works |
| ⬜ | V3 | In IDLE: manually lower fuel in sensing container → distance reading increases on LCD | Confirms sensor polarity correct (sensor above = fuel drops = distance increases) |
| ⬜ | V4 | Press B3 → check serial for printed `DStart` and `VStart` values | Must be non-zero and sensible (e.g. DStart ≈ real distance, VStart ≈ pot voltage) |
| ⬜ | V5 | In RUNNING: manually remove fuel from sensing container → watch `V_cmd` rise on LCD | Confirms PI responds in correct direction |
| ⬜ | V6 | Trigger E-stop (open D27 circuit) → pump must reverse CCW at max speed | Safety check — must pass before any live experiment |

---

## T — Tuning

> Requires all V tasks complete. Do on first live burn.

| Status | # | Task | Notes |
|--------|---|------|-------|
| ⬜ | T1 | First burn: run with default gains `Kp=0.3, Ki=0.02` | If level oscillates → reduce gains. If response too slow → increase. |
| ⬜ | T2 | Save 30-min serial CSV, plot `V_cmd` vs time | Should drift from ~3.5V to ~4.3V automatically. If flat → `Ki_OUTER` too small. |

---

## Status key

| Symbol | Meaning |
|--------|---------|
| ⬜ | Not started |
| 🔄 | In progress |
| ✅ | Complete |
| ❌ | Blocked / failed — add note |

---

## Critical path

```
H1 (fix sensor)
 └── H2, H3 (verify relays + E-stop)
       └── C1 (Td experiment)
             └── C2 (fill in Td/K, enable Smith)
                   └── V1 → V2 → V3 → V4 → V5 → V6 (bench verify)
                                                         └── T1 → T2 (live tune)
```
