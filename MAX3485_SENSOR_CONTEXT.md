# Context: ESP32 + MAX3485 + SDL Laser Distance Sensor

Paste this file into a new conversation to get full context for testing or debugging the RS-485 sensor link.

---

## Goal

Test whether the MAX3485 module can communicate with the SDL series laser distance sensor over RS-485 Modbus RTU. The sensor should return a valid 9-byte response containing a 32-bit distance value in micrometres.

---

## Hardware

| Component | Detail |
|---|---|
| Microcontroller | ESP32 WROOM |
| RS-485 transceiver | MAX3485 module (3.3V native — no level shifting needed) |
| Sensor | STUOB SDL Series CMOS Micro Laser Displacement Sensor |
| Sensor interface | RS-485, Modbus RTU, half-duplex |
| Sensor power | 24V DC (separate supply) |
| ESP32 + MAX3485 power | 3.3V / 5V from USB |

---

## Wiring

| Signal | MAX3485 pin | ESP32 pin |
|---|---|---|
| RS-485 A | A | — |
| RS-485 B | B | — |
| Receive Out | RO | RX2 = GPIO 16 |
| Driver In | DI | TX2 = GPIO 17 |
| RE + DE (tied together) | RE, DE | D4 = GPIO 4 |
| Power | VCC | 3.3V |
| Ground | GND | GND (must be common with sensor ground) |

**Important:** Sensor is powered from 24V. All grounds (24V supply, MAX3485, ESP32) must share a common reference.

---

## RS-485 Communication Settings

| Parameter | Value |
|---|---|
| Baud rate | 9600 bps |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Protocol | Modbus RTU |
| Mode | Half-duplex |
| Sensor slave address | 0x01 |

---

## Modbus Query Frame (8 bytes — send this to read distance)

```
01  03  00  3B  00  02  B5  C6
```

| Byte | Value | Meaning |
|---|---|---|
| 1 | `0x01` | Slave address |
| 2 | `0x03` | Function: Read Holding Registers |
| 3–4 | `0x00 0x3B` | Register address 0x003B |
| 5–6 | `0x00 0x02` | Read 2 registers (32 bits) |
| 7–8 | `0xB5 0xC6` | CRC16 little-endian |

---

## Expected Response Frame (9 bytes)

```
01  03  04  D0  D1  D2  D3  CRC_L  CRC_H
```

- Bytes 0–2: echo of address, function code, byte count (must be `01 03 04`)
- Bytes 3–6: 32-bit distance value, big-endian
- Bytes 7–8: CRC16, little-endian

### Decoding distance

```cpp
uint32_t raw = ((uint32_t)D0 << 24) | ((uint32_t)D1 << 16)
             | ((uint32_t)D2 <<  8) |  (uint32_t)D3;
float distance_mm = raw / 1000.0f;   // SDL-100/200/400: 1 µm resolution
```

Example: bytes `00 00 B8 47` → raw = 47175 → **47.175 mm**

---

## RE/DE Direction Pin

The MAX3485 is half-duplex. RE and DE are tied together and driven from GPIO 4:
- `HIGH` = transmit mode (ESP32 sends query)
- `LOW` = receive mode (ESP32 listens for sensor reply)

**Critical sequence:**
```cpp
digitalWrite(PIN_REDE, HIGH);   // switch to TX
Serial2.write(QUERY, 8);
Serial2.flush();                // wait until last bit physically sent
digitalWrite(PIN_REDE, LOW);   // THEN switch to RX — sensor reply is already coming
```
Never toggle LOW before `flush()` — you will cut off the last bytes of your own query.

---

## CRC-16/MODBUS

```cpp
uint16_t crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;
}
// CRC covers first 7 bytes of response.
// Received CRC: buf[7] = low byte, buf[8] = high byte.
uint16_t crcRecv = (uint16_t)buf[8] << 8 | buf[7];
```

---

## Test Sketch

File: `Sensor_Comms_Test/Sensor_Comms_Test.ino`

Polls at 1 Hz, prints raw hex bytes every cycle, decodes distance if valid. Open Serial Monitor at **115200 baud**.

**Healthy output:**
```
Raw (9 bytes): 01 03 04 00 00 B8 47 xx xx
OK  Distance = 47.175 mm
```

---

## Troubleshooting

| Symptom | Most likely cause | Fix |
|---|---|---|
| 0 bytes received | 24V power off, or A/B not connected | Check supply and wiring |
| 0 bytes received | D4 not going HIGH during TX | Confirm `digitalWrite(PIN_REDE, HIGH)` before write |
| Garbled / wrong slave address | A and B wires swapped | Swap A↔B at sensor or module |
| Garbled | Baud rate mismatch | Try 19200, 38400 in `Serial2.begin()` |
| Partial frame (< 9 bytes) | Timeout too short | Increase `RESPONSE_TIMEOUT_MS` |
| CRC mismatch | Noise, or swapped A/B | Check termination, try swapping A↔B |
| Distance value 10× off | Wrong sensor model divisor | SDL-030/050 → /10000, SDL-100/200/400 → /1000, SDL-800 → /100 |

---

## Current Project Status

This sensor test is part of a larger pool fire experiment controller (ESP32). The full project includes:
- Mode 1: manual prime (fill/drain)
- Mode 2: constant injection at pot-set speed
- Mode 3: automatic fuel level control (PI controller + optional Smith Predictor for transport dead time)

The sensor test is step H1 of the project's PROGRESS.md checklist — everything else depends on getting valid distance readings.
