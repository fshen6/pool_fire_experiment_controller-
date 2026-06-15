# Mode 3 — To-Do List & Progress Tracker

_Goal: automatic liquid level control using ESP32 + peristaltic pump + SDL laser sensor._
_Update status symbols as you go._

---

## Status key

| Symbol | Meaning |
|--------|---------|
| ⬜ | Not started |
| 🔄 | In progress |
| ✅ | Complete |
| ❌ | Blocked / failed — add note |

---

## STAGE 1 — Hardware verification
> Do these on the bench with no fire. Takes ~30 min.

| Status | # | Task | How to verify |
|--------|---|------|---------------|
| ✅ | H1 | Swap MAX485 → MAX3485 (fix 5V/3.3V mismatch on RO line) | `Sensor_Comms_Test.ino` prints `OK Distance = X mm` |
| ⬜ | H2 | Verify Relay1 wiring: HIGH = pump runs, LOW = pump stops | Toggle D19 HIGH/LOW manually in a test sketch — confirm pump starts/stops |
| ⬜ | H3 | Verify Relay2 wiring: LOW = CW (inject), HIGH = CCW (drain) | Toggle D25 while pump runs — confirm direction matches |
| ⬜ | H4 | Verify E-stop wiring: D27 HIGH = active (normally-closed switch to GND) | Open E-stop switch → D27 goes HIGH. Close it → D27 LOW. Wrong way = Mode 3 stuck in E-stop forever |
| ⬜ | H5 | Verify potentiometer reads 0→4095 across full rotation | Print `analogRead(32)` — should span full 12-bit range |
| ⬜ | H6 | Verify voltage sensor reads correctly: compare to multimeter on converter output | Run `potentiometer_VSensor_Convertor_Test` — displayed V should match multimeter ±0.1V |
| ⬜ | H7 | Verify LCD shows correct I2C address (0x27 or 0x3F) | LCD lights up and shows text on startup |

---

## STAGE 2 — Mode 3 bench test (no fire, no fuel)
> Flash `Mode_3/Mode_3.ino`. Open Serial Monitor at 115200 baud. Use water in sensing container.

| Status | # | Task | What to check |
|--------|---|------|---------------|
| ⬜ | V1 | Flash Mode 3, open serial monitor | CSV header line prints: `timestamp_ms,distance_mm,...` → code running |
| ⬜ | V2 | IDLE state: turn pot → pump runs CW, voltage changes | LCD line 2 shows `V_meas` and PWM changing. Confirms pot→PWM→converter→sensor chain |
| ⬜ | V3 | IDLE state: manually lower water level in sensing container | LCD `D` value increases (sensor above → level drops → distance increases). If it decreases, invert the error sign in Mode 3 |
| ⬜ | V4 | Press B3 to start → check serial output | Should print `DStart=XX.XXX mm  VStart=X.XXX V`. Both must be non-zero |
| ⬜ | V5 | RUNNING: manually remove water → watch LCD `V_cmd` rise | Confirms PI responds: more error → higher voltage commanded → faster pump |
| ⬜ | V6 | RUNNING: manually add water (over-level) → watch `V_cmd` drop | Confirms PI responds in both directions |
| ⬜ | V7 | Trigger E-stop (open D27 circuit) → pump reverses CCW at max | LCD shows `!! E-STOP !!`. **Must pass before any live fire experiment** |
| ⬜ | V8 | Release E-stop → returns to IDLE, pump stops | LCD shows `E-stop cleared` |
| ⬜ | V9 | Sensor fault test: unplug sensor cable while RUNNING | LCD shows `SENSOR FAULT!`, pump holds last PWM (does not slam to 0 or max) |

---

## STAGE 3 — Td experiment (measure transport dead time)
> Requires Stage 1 complete. Run with actual fuel (or water for a dry run).
> This gives you the two numbers needed to enable the Smith Predictor.

| Status | # | Task | Notes |
|--------|---|------|-------|
| ⬜ | C1 | Flash `Td_Measurement/Td_Measurement.ino` | Same hardware setup as Mode 3 |
| ⬜ | C2 | Run baseline phase (30 s) — sensor stationary | Serial prints noise floor σ. Record it. |
| ⬜ | C3 | Press B3 to trigger step injection — watch serial CSV | Wait for `DETECTED` message. Record `Td_s` and `K_MODEL` printed at the end |
| ⬜ | C4 | Repeat C2–C3 at least 3 times | Check `Td_s` is consistent (±20%) across trials. If wildly different, check hydraulic connection for trapped air |
| ⬜ | C5 | Fill measured values into `Mode_3.ino` config section | `#define Td_s X.X` and `#define K_MODEL X.XXXX` |
| ⬜ | C6 | Set `SMITH_PREDICTOR_ENABLED 1` in `Mode_3.ino` | Smith Predictor now active |
| ⬜ | C7 | Re-flash Mode 3, verify serial header shows `Smith Predictor: ENABLED` | Confirms the define is active |

---

## STAGE 4 — First live burn (tuning)
> Requires Stages 1–3 complete. Use your actual fuel. Have E-stop accessible.

| Status | # | Task | Notes |
|--------|---|------|-------|
| ⬜ | T1 | Set conservative starting gains in `Mode_3.ino`: `Kp_OUTER=0.3, Ki_OUTER=0.02, Kp_INNER=20` | These are intentionally slow — better to undershoot than oscillate on first run |
| ⬜ | T2 | Dial pot to ~3.5V (starting burn rate for your fuel), press B3 | Watch level hold for 2–3 min. If it oscillates → halve both Kp and Ki. If too sluggish → double Ki only |
| ⬜ | T3 | Run for 30 min, save serial CSV to file | Use Arduino Serial Monitor → "Save output", or a terminal logger (e.g. `python -m serial.tools.miniterm --raw COM? 115200 > log.csv`) |
| ⬜ | T4 | Plot `V_cmd` vs time from CSV | Should drift from ~3.5V up to ~4.3V automatically as burn rate increases. If it stays flat → `Ki_OUTER` too small |
| ⬜ | T5 | Plot `distance_mm` vs time | Should stay within ±0.1 mm of DStart in steady state. Larger than ±0.5 mm → gains need tuning |

---

## STAGE 5 — Validated experiment
> Run only after T4 and T5 show acceptable performance.

| Status | # | Task | Notes |
|--------|---|------|-------|
| ⬜ | E1 | Run full experiment set with controller active | Compare fuel consumption (mass scale) to controller-commanded `V_cmd` trend |
| ⬜ | E2 | Check repeatability: run same fuel, same DStart, same gains | `V_cmd` profiles across runs should be similar — confirms controller is working, not fighting |
| ⬜ | E3 | Test with different fuel (7× different burn rate) | Controller should settle at a very different `V_cmd` with no gain changes needed — confirms PI adapts |

---

## Critical path

```
H1 ✅ (sensor working)
 │
 ├── H2, H3, H4, H5, H6, H7  (bench hardware checks)
 │         │
 │         └── V1 → V2 → V3 → V4 → V5 → V6 → V7 → V8 → V9
 │                                                      │
 │                                            C1 → C2 → C3 → C4 → C5 → C6 → C7
 │                                                                          │
 │                                                               T1 → T2 → T3 → T4 → T5
 │                                                                                    │
 └────────────────────────────────────────────────────────────────────────── E1 → E2 → E3
```

**Next action: H2 — verify relay wiring.**
