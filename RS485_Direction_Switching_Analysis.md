# Why the RS-485 Sketch Failed — Root Cause Analysis

## Summary

The working sketch has two `delayMicroseconds(200)` calls that the previous versions did not. These are not optional — they are mandatory guards for the physics of RS-485 direction switching. Without them, the query frame is corrupted and/or the receiver captures noise when it first enables, causing the sensor to either ignore the query or return garbage.

---

## The Two Versions Side by Side

### Non-working (previous guide template)

```cpp
digitalWrite(PIN_REDE, HIGH);          // switch to TX
Serial2.write(query, sizeof(query));   // send immediately
Serial2.flush();                       // "wait until physically transmitted"
digitalWrite(PIN_REDE, LOW);           // switch to RX "immediately"
```

### Working (confirmed)

```cpp
digitalWrite(RE_DE_PIN, HIGH);         // switch to TX
delayMicroseconds(200);                // ← GUARD 1
RS485Serial.write(request, sizeof(request));
RS485Serial.flush();
delayMicroseconds(200);                // ← GUARD 2
digitalWrite(RE_DE_PIN, LOW);          // switch to RX
```

The only structural difference is the two `delayMicroseconds(200)` guards. Everything else (UART2, pins 16/17, 9600 baud, same query bytes) is identical.

---

## Guard 1 — Why You Must Wait After DE Goes HIGH Before Writing

### What happens without it

```
t = 0 µs:   digitalWrite(HIGH)  → DE pin goes HIGH on ESP32
t ≈ 1 µs:   Serial2.write()     → UART immediately clocks out first bit of 0x01
t ≈ 50 µs:  MAX3485 driver fully enabled (typical, depends on module circuit)
```

The UART starts transmitting the moment `write()` is called — it does not wait for anything. If the MAX3485 driver is not yet fully enabled, the first bits go out onto the A/B lines while the driver is still in a transitional state (partially enabled, or high-impedance).

**Effect:** The first byte (0x01 — slave address) is corrupted. The sensor validates the slave address as the first thing it does. A corrupted address means the sensor completely ignores the frame and never sends a response.

### Why 200 µs?

At 9600 baud, one bit period = **104.17 µs**.

The MAX3485 IC itself enables in a few hundred nanoseconds. But the full system includes:
- ESP32 GPIO rise time and settling
- PCB trace capacitance
- Any RC filtering on the DE pin on the module
- The RS-485 bus capacitance charging to the driven level

200 µs = ~1.9 bit periods. This comfortably covers all of the above. The driver is rock-solid before the first bit goes out.

### Timing diagram

```
DE pin:   _____|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                ↑                    ↑
                DE goes HIGH     write() called
                                 (200µs after DE HIGH)
                    |← 200 µs →|

Driver:   _____|~~~~~~~~~~|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                           ↑
                      Driver fully active
                      (well before first bit)

Bus:      IDLE  |← settling→| 0  1  0  1  0  0  0  0  ...
                                 (byte 0x01 = clean)
```

Without Guard 1, `write()` fires during the `~~` region — the driver is partially active and the first bits are corrupted.

---

## Guard 2 — Why You Must Wait After flush() Before DE Goes LOW

### What flush() actually does (and what it does NOT do)

`Serial2.flush()` on ESP32 calls `uart_wait_tx_done()` from the ESP-IDF. This waits for:
- The TX FIFO to drain, AND
- The TX shift register to finish shifting out all bits

So when `flush()` returns, the last bit has been shifted out of the UART's shift register onto the internal TX pin. But "shifted out" is not the same as "finished on the physical RS-485 bus."

**What flush() does NOT guarantee:**
1. That the stop bit has fully propagated through the MAX3485 to the A/B lines
2. That the RS-485 bus has returned to idle (mark state)
3. That no glitch will appear on the bus during the DE transition

### What happens without Guard 2

```
flush() returns     → last bit has left the shift register
→ immediately:      → digitalWrite(LOW): DE goes LOW
→ MAX3485 switches  → driver shuts off, receiver enables
```

The problem: at the instant `flush()` returns and DE goes LOW simultaneously, the MAX3485 is:
- Still propagating the stop bit through its output drivers onto A/B
- Being commanded to shut off its driver and enable its receiver

During this simultaneous transition, the A/B differential voltage is in an undefined state — the driver is shutting off mid-bit. The line transitions from a driven voltage to floating, partly controlled by whatever bus termination and biasing exists.

**Effect on the UART receiver:** When the MAX3485 receiver enables, it is listening to a bus that is transitioning from a driven stop bit (HIGH) to floating/idle. A falling edge during or just after this transition can look like a **start bit** to the UART receiver. The UART then tries to frame the noise that follows as data bytes, filling the RX buffer with garbage before the sensor has even started to reply.

This is why the previous version produced gibberish — not random noise, but bytes that appear to be UART-framed data because the receiver was triggered by the direction-switch glitch.

### What 200 µs of guard time does

At 9600 baud, one stop bit = 104.17 µs.

200 µs after `flush()`:
- The stop bit has fully propagated through the MAX3485 to the bus (the IC itself takes ~120 ns, so this is 1500× the IC's propagation delay)
- The RS-485 bus has returned to idle/mark state
- There is a clean, settled idle period on the bus

When DE then goes LOW, the receiver enables into a clean idle bus — no transition, no glitch, no false start bit.

### Timing diagram

```
UART TX:  ... [byte 7: 0xC6] [stop] ___idle___
                                ↑         ↑
                          flush() returns  DE LOW (200µs later)
                              |← 200 µs →|

Bus (A-B): ... driven high (stop) ... settling ... IDLE
                                                    ↑
                                             Receiver enables here
                                             (clean idle = no false start bit)
```

Without Guard 2:
```
UART TX:  ... [byte 7: 0xC6] [stop]
                                ↑
                          flush() returns
                          DE LOW immediately

Bus (A-B): ... driven high ... ↓↗ GLITCH
                                  ↑
                         Receiver enables during bus transition
                         → false start bit → garbage in RX buffer
```

---

## Why flush() Alone Seemed Correct (and Was Not)

The previous guide stated:

> `Serial2.flush()` — wait until all bytes physically transmitted

This was wrong in a subtle but important way. `flush()` waits until the UART's shift register is empty, meaning the last bit has left the UART peripheral. But:

1. The UART shift register is internal to the ESP32 chip
2. The MAX3485 adds its own propagation delay (~120 ns)
3. The RS-485 differential bus has capacitance that takes time to settle
4. The stop bit adds one more bit period (104 µs) of driven voltage after the last data bit

The physical signal on the A/B wires lags behind the UART shift register by all of the above. `flush()` does not know about the A/B wires — it only knows about its own shift register.

The correct mental model is:

```
flush() = "the UART is done with its internal job"
≠ "the physical bus is clean and ready to switch direction"
```

---

## Other Differences (Not Root Cause)

| Difference | Working version | Previous version | Impact |
|---|---|---|---|
| `HardwareSerial RS485Serial(2)` vs `Serial2` | Explicit instance | Pre-defined global | **None** — identical in ESP32 Arduino core |
| Explicit pin spec `begin(9600, SERIAL_8N1, 16, 17)` | Yes | No | **None** — default pins for Serial2 are also 16/17 on WROOM |
| No RX buffer flush before query | Working version skips it | Previous version flushes | **Slightly worse** in working version, not the issue |
| 50ms timeout vs 200ms | 50ms | 200ms | **None** — both exceed the ~20ms round trip |
| Silent on failure vs printing errors | Working version silent | Previous version verbose | **None** — diagnostic only |

---

## The Corrected Mental Model for RS-485 Direction Switching

```
                ┌──────────────────────────────────────────────┐
                │         CORRECT SEQUENCE                     │
                └──────────────────────────────────────────────┘

1. DE = HIGH                         (enable transmit driver)
2. delay ≥ 2 bit periods             (let driver fully activate and bus settle)
3. write(query)                      (transmit all bytes)
4. flush()                           (wait for UART shift register to empty)
5. delay ≥ 2 bit periods             (let stop bit propagate, bus return to idle)
6. DE = LOW                          (enable receiver — into clean idle bus)
7. collect response bytes            (now the receiver will see only sensor data)
```

The two delays bracket the actual transmission. They are not optional — they compensate for the physics of the RS-485 bus that the UART peripheral cannot see.

---

## Corrected Code Template

```cpp
#define RE_DE_PIN 4

HardwareSerial RS485Serial(2);

const uint8_t QUERY[8] = {0x01, 0x03, 0x00, 0x3B, 0x00, 0x02, 0xB5, 0xC6};

void setup() {
    Serial.begin(115200);
    RS485Serial.begin(9600, SERIAL_8N1, 16, 17);
    pinMode(RE_DE_PIN, OUTPUT);
    digitalWrite(RE_DE_PIN, LOW);       // start in RX mode
}

bool readDistance(float &distance_mm) {
    // 1. Enable transmit driver and wait for it to fully activate
    digitalWrite(RE_DE_PIN, HIGH);
    delayMicroseconds(200);             // ← Guard 1: driver settle time

    // 2. Send query
    RS485Serial.write(QUERY, sizeof(QUERY));
    RS485Serial.flush();                // wait for UART shift register to empty

    // 3. Wait for stop bit to propagate, then switch to receive
    delayMicroseconds(200);             // ← Guard 2: bus settle time
    digitalWrite(RE_DE_PIN, LOW);

    // 4. Collect response
    uint8_t buf[9];
    int got = 0;
    unsigned long t0 = millis();
    while (got < 9 && millis() - t0 < 50) {
        if (RS485Serial.available()) buf[got++] = RS485Serial.read();
    }

    if (got < 9) return false;
    if (buf[0] != 0x01 || buf[1] != 0x03 || buf[2] != 0x04) return false;

    uint32_t raw = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16)
                 | ((uint32_t)buf[5] <<  8) |  (uint32_t)buf[6];
    distance_mm = raw / 1000.0f;        // SDL-100/200/400: 1 µm resolution
    return true;
}
```

---

## Key Takeaway

> **`flush()` is necessary but not sufficient.**  
> It ensures the UART is done. It does not ensure the bus is done.  
> Two `delayMicroseconds(200)` guards — one before the first bit, one after the last bit — are required for reliable RS-485 communication at 9600 baud on this hardware.

The 200 µs value (≈ 2 bit periods at 9600 baud) is a safe minimum. It can be reduced for higher baud rates proportionally, but for 9600 baud these two guards are the difference between reliable communication and intermittent/no response.
