# Laser Distance Sensor (SDL Series) — Communication & Troubleshooting Guide

## Hardware Overview

- **Sensor:** STUOB SDL Series — CMOS Micro Laser Displacement Sensor  
- **Interface:** RS-485 (half-duplex), Modbus RTU protocol  
- **Transceiver:** MAX485 module  
- **Controller:** ESP32 WROOM using Serial2

### Wiring

| Sensor Terminal | MAX485 Pin | ESP32 Pin |
|---|---|---|
| RS-485 A | A | — |
| RS-485 B | B | — |
| — | RO (Receive Out) | RX2 (GPIO 16) |
| — | DI (Driver In) | TX2 (GPIO 17) |
| — | RE + DE (tied together) | D4 (GPIO 4) |

- Sensor powered from **24V** supply (separate from ESP32)
- All grounds must share a **common reference**
- MAX485 and ESP32 run on **5V / 3.3V** (check your module)

---

## RS-485 Communication Settings

| Parameter | Value |
|---|---|
| Baud rate | 9600 bps (factory default) |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Protocol | Modbus RTU |
| Mode | Half-duplex |
| Slave address | 0x01 (default) |

---

## Modbus RTU — Reading Distance

### Query Frame (send this 8 bytes)

```
01  03  00  3B  00  02  B5  C6
```

| Byte | Value | Meaning |
|---|---|---|
| 1 | `0x01` | Slave address |
| 2 | `0x03` | Function code: Read Holding Registers |
| 3 | `0x00` | Register address high byte |
| 4 | `0x3B` | Register address low byte (register 0x003B) |
| 5 | `0x00` | Register count high byte |
| 6 | `0x02` | Register count low byte (read 2 registers = 32 bits) |
| 7 | `0xB5` | CRC16 low byte |
| 8 | `0xC6` | CRC16 high byte |

### Response Frame (expect 9 bytes)

```
01  03  04  D0  D1  D2  D3  CRC_L  CRC_H
```

| Byte | Meaning |
|---|---|
| `0x01` | Slave address (echo) |
| `0x03` | Function code (echo) |
| `0x04` | Byte count (4 data bytes follow) |
| D0–D3 | 32-bit distance value, big-endian |
| CRC_L/H | Modbus CRC16 checksum |

### Decoding the Distance Value

The sensor sends the 32-bit value **word-swapped** (Modbus register format): the low 16-bit word arrives first on the wire, then the high 16-bit word. Each word is big-endian internally.

```
Wire bytes:   B8 47  00 00
              ─────  ─────
              loWord hiWord   →  raw = (hiWord << 16) | loWord = 0x0000B847 = 47175
```

```cpp
uint16_t loWord = ((uint16_t)buf[3] << 8) | buf[4];   // first word on wire = low 16 bits
uint16_t hiWord = ((uint16_t)buf[5] << 8) | buf[6];   // second word on wire = high 16 bits
int32_t  raw    = (int32_t)(((uint32_t)hiWord << 16) | loWord);
float distance_mm = raw / 1000.0f;   // see divisor table below; value can be negative
```

**Example from manual:**  
Wire bytes: `B8 47 00 00` → loWord = 0xB847, hiWord = 0x0000 → raw = 0x0000B847 = 47175  
→ 47175 µm = **47.175 mm**

> ⚠️ Note: a naive big-endian decode (`buf[3]<<24 | buf[4]<<16 | buf[5]<<8 | buf[6]`) gives the correct answer only when the distance is < 65.535 mm (high word = 0). For any larger distance the words must be swapped as shown above.

### Resolution by Model

| Model | Resolution | Divide raw by |
|---|---|---|
| SDL-030, SDL-050 | 0.1 µm | 10000 |
| SDL-100, SDL-200, SDL-400 | 1 µm | 1000 |
| SDL-800 | 10 µm | 100 |

---

## CRC16 (Modbus)

| Field | Value |
|---|---|
| Algorithm | CRC-16/MODBUS |
| Polynomial | 0x8005 |
| Initial value | 0xFFFF |
| Input reflected | true |
| Output reflected | true |
| XOR out | 0x0000 |

```cpp
uint16_t crc16Modbus(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}
```

---

## Correct Read Cycle (Code Template)

> ⚠️ **Critical:** Two `delayMicroseconds(200)` guards are required around the transmission.
> See `RS485_Direction_Switching_Analysis.md` for the full explanation.
> `flush()` alone is NOT sufficient — it only empties the UART shift register, not the physical bus.

This is the confirmed-working sketch (`Sensor_Comms_Test/Sensor_Comms_Test.ino`):

```cpp
// ---------- Pins ----------
static const int PIN_RX2  = 16;   // MAX3485 RO  -> ESP32 RX2
static const int PIN_TX2  = 17;   // MAX3485 DI  <- ESP32 TX2
static const int PIN_REDE = 4;    // MAX3485 RE+DE (tied) <- ESP32 GPIO4

// ---------- Comms settings ----------
static const uint32_t SENSOR_BAUD         = 9600;
static const uint8_t  SENSOR_ADDR         = 0x01;
static const uint32_t POLL_INTERVAL_MS    = 20;
static const uint32_t RESPONSE_TIMEOUT_MS = 50;

/*
 * Distance divisor depends on the SDL model:
 *   SDL-030 / SDL-050           -> 10000.0  (0.1 µm resolution)
 *   SDL-100 / SDL-200 / SDL-400 ->  1000.0  (1 µm resolution)   <-- default
 *   SDL-800                     ->   100.0  (10 µm resolution)
 * If readings are exactly 10× or 100× off, change this value.
 */
static const float DISTANCE_DIVISOR = 1000.0f;

uint8_t query[8];
uint8_t buf[16];

uint16_t crc16(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;
}

void buildQuery() {
    query[0] = SENSOR_ADDR;
    query[1] = 0x03;        // read holding registers
    query[2] = 0x00;        // register address hi
    query[3] = 0x3B;        // register address lo
    query[4] = 0x00;        // register count hi
    query[5] = 0x02;        // register count lo (2 regs = 32 bits)
    uint16_t c = crc16(query, 6);
    query[6] = c & 0xFF;    // CRC low byte first (Modbus little-endian)
    query[7] = c >> 8;
}

// Send the query and read up to 9 bytes. Returns byte count received.
int pollSensor() {
    while (Serial2.available()) Serial2.read();   // drain stale bytes

    // --- Transmit ---
    digitalWrite(PIN_REDE, HIGH);     // driver enabled -> TX mode
    delayMicroseconds(200);           // Guard 1: let DE settle before first start bit
    Serial2.write(query, 8);
    Serial2.flush();                  // wait for frame to finish sending
    delayMicroseconds(200);           // Guard 2: bus settle before releasing driver
    digitalWrite(PIN_REDE, LOW);      // switch to RX before reply lands

    // --- Receive ---
    int idx = 0;
    uint32_t start = millis();
    while (idx < 9 && (millis() - start) < RESPONSE_TIMEOUT_MS) {
        if (Serial2.available()) buf[idx++] = Serial2.read();
    }
    return idx;
}

void setup() {
    Serial.begin(115200);
    delay(300);
    pinMode(PIN_REDE, OUTPUT);
    digitalWrite(PIN_REDE, LOW);
    Serial2.begin(SENSOR_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
    buildQuery();
}

void loop() {
    int n = pollSensor();

    if (n < 9 || buf[0] != SENSOR_ADDR || buf[1] != 0x03 || buf[2] != 0x04) {
        delay(POLL_INTERVAL_MS);
        return;
    }

    uint16_t crcCalc = crc16(buf, 7);
    uint16_t crcRecv = ((uint16_t)buf[8] << 8) | buf[7];  // buf[7]=lo, buf[8]=hi
    if (crcCalc != crcRecv) { delay(POLL_INTERVAL_MS); return; }

    // Decode: wire order is word-swapped (low word first, then high word)
    // e.g. wire bytes B8 47 00 00 → loWord=0xB847, hiWord=0x0000 → 47175 → 47.175 mm
    uint16_t loWord = ((uint16_t)buf[3] << 8) | buf[4];
    uint16_t hiWord = ((uint16_t)buf[5] << 8) | buf[6];
    int32_t  raw    = (int32_t)(((uint32_t)hiWord << 16) | loWord);
    float distance_mm = raw / DISTANCE_DIVISOR;

    Serial.print("Distance: ");
    Serial.print(distance_mm, 3);
    Serial.println(" mm");

    delay(POLL_INTERVAL_MS);
}
```

---

## Troubleshooting Guide

### Symptom: Gibberish / corrupted bytes received

**Cause 1 — REDE pin not switched to LOW before sensor replies (most common)**  
If `D4` stays HIGH after the query is sent, the MAX485 stays in transmit mode. The sensor's reply arrives on the bus but the ESP32 is still driving it — you read your own transmitted bytes back as garbage.

Fix:
```cpp
Serial2.write(query, sizeof(query));
Serial2.flush();          // <-- critical: wait for UART to physically finish
digitalWrite(PIN_REDE, LOW);  // only THEN switch to RX
```
Never toggle REDE before `Serial2.flush()`.

---

**Cause 2 — A and B terminals swapped**  
RS-485 A/B polarity is mandatory. If A and B are swapped anywhere in the chain (at the MAX485 screw terminal or at the sensor), you get inverted signal levels = framing errors = garbage.

Check: With D4 LOW and sensor powered, measure voltage between A and B at idle. A should be more positive than B (~0.2V or more). If B > A, swap the wires.

---

**Cause 3 — Sensor baud rate was changed from factory default**  
The SDL sensor's baud rate is configurable from its front panel (see manual pages 3–4, RS-485 digital output section). If it was ever changed, 9600 will produce misframed garbage.

Fix: Try 19200 → 38400 → 115200 in your `Serial2.begin()` call. Once you find the matching rate, reconfigure the sensor back to 9600 via its panel if desired.

---

**Cause 4 — Stale bytes in receive buffer**  
If previous failed reads left bytes in `Serial2`'s buffer, the next read starts mid-frame.

Fix: Always flush the RX buffer before sending a new query:
```cpp
while (Serial2.available()) Serial2.read();
```

---

**Cause 5 — Response timeout too short**  
At 9600 baud, transmitting the 8-byte query takes ~8.3ms and the 9-byte response takes ~9.4ms to arrive. Total round-trip is ~18–20ms minimum. If you check for data in under 20ms you may read a partial frame.

Fix: Use a timeout of at least **100ms** to be safe.

---

### Symptom: No data received at all (buffer always empty)

- Check 24V sensor power supply is on
- Check common ground between sensor, MAX485, and ESP32
- Verify TX2 (GPIO17) → DI on MAX485, and RO on MAX485 → RX2 (GPIO16)
- Confirm D4 is going HIGH during transmit (measure with multimeter)
- Try swapping A and B wires — if A/B are swapped, the sensor won't decode the query and won't respond at all

---

### Symptom: Distance value is clearly wrong (e.g. 10× off)

- Check your sensor model: SDL-030/050 → divide by 10000; SDL-100/200/400 → divide by 1000; SDL-800 → divide by 100
- Confirm byte order: D0 is the most-significant byte (big-endian)

---

### Symptom: CRC validation fails

- The CRC covers the first 7 bytes of the response
- The CRC in the frame is **little-endian**: byte[7] is low byte, byte[8] is high byte
- Use CRC-16/MODBUS (not CRC-16/IBM or others — the init value and reflection matter)

---

### Diagnostic: Print raw bytes first

Before trying to decode, print every received byte in HEX to confirm what you're actually getting:

```cpp
for (int i = 0; i < received; i++) {
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX);
    Serial.print(" ");
}
Serial.println();
```

A valid response starts with `01 03 04` — if you see anything else, use this guide to diagnose.
