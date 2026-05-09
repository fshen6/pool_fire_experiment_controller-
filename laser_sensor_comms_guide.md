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

Reconstruct the 32-bit integer from D0–D3 (big-endian):

```cpp
uint32_t raw = ((uint32_t)D0 << 24) | ((uint32_t)D1 << 16) | ((uint32_t)D2 << 8) | D3;
float distance_mm = raw / 1000.0;
```

**Example from manual:**  
Response data bytes: `00 00 B8 47`  
→ `0x0000B847` = 47175 (decimal)  
→ 47175 µm = **47.175 mm**

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

```cpp
#define PIN_REDE 4
#define BAUD     9600

const uint8_t query[8] = {0x01, 0x03, 0x00, 0x3B, 0x00, 0x02, 0xB5, 0xC6};
const int RESPONSE_LEN = 9;
const int RESPONSE_TIMEOUT_MS = 100;

void setup() {
    Serial.begin(115200);
    Serial2.begin(BAUD, SERIAL_8N1);  // RX2=16, TX2=17
    pinMode(PIN_REDE, OUTPUT);
    digitalWrite(PIN_REDE, LOW);      // start in RX mode
}

bool readDistance(float &distance_mm) {
    // 1. Flush any stale incoming data
    while (Serial2.available()) Serial2.read();

    // 2. Switch to TX, send query
    digitalWrite(PIN_REDE, HIGH);
    Serial2.write(query, sizeof(query));
    Serial2.flush();                  // wait until all bytes physically transmitted

    // 3. Switch back to RX immediately
    digitalWrite(PIN_REDE, LOW);

    // 4. Wait for full response
    uint8_t buf[RESPONSE_LEN];
    int received = 0;
    unsigned long start = millis();

    while (received < RESPONSE_LEN) {
        if (millis() - start > RESPONSE_TIMEOUT_MS) return false; // timeout
        if (Serial2.available()) buf[received++] = Serial2.read();
    }

    // 5. Validate frame structure
    if (buf[0] != 0x01) return false;  // wrong slave address
    if (buf[1] != 0x03) return false;  // wrong function code
    if (buf[2] != 0x04) return false;  // wrong byte count

    // 6. (Optional but recommended) Validate CRC
    uint16_t crcCalc = crc16Modbus(buf, 7);
    uint16_t crcRecv = (uint16_t)buf[8] << 8 | buf[7];  // CRC is little-endian in frame
    if (crcCalc != crcRecv) return false;

    // 7. Decode distance
    uint32_t raw = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16)
                 | ((uint32_t)buf[5] <<  8) |  (uint32_t)buf[6];
    distance_mm = raw / 1000.0;
    return true;
}

void loop() {
    float dist;
    if (readDistance(dist)) {
        Serial.print("Distance: ");
        Serial.print(dist, 3);
        Serial.println(" mm");
    } else {
        Serial.println("No valid response");
    }
    delay(100);  // 10 Hz
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
