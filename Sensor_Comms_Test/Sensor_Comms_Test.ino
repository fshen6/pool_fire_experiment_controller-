// ============================================================
//  Sensor_Comms_Test.ino
//  Diagnostic sketch for MAX3485 + SDL laser distance sensor
//
//  Wiring (MAX3485 is 3.3V native — no level shifting needed):
//    MAX3485 RO  → ESP32 RX2 (GPIO 16)
//    MAX3485 DI  → ESP32 TX2 (GPIO 17)
//    MAX3485 RE+DE (tied) → ESP32 D4 (GPIO 4)
//    MAX3485 VCC → ESP32 3.3V
//    MAX3485 GND → common GND
//    Sensor RS-485 A → MAX3485 A
//    Sensor RS-485 B → MAX3485 B
//    Sensor powered from 24V (separate supply, common GND)
//
//  Open Serial Monitor at 115200 baud.
//  A healthy response starts with: 01 03 04 ...
// ============================================================

#define PIN_REDE  4      // MAX3485 RE+DE direction control
#define RS485_BAUD  9600
#define RESPONSE_TIMEOUT_MS  200   // generous timeout for first test

// Modbus RTU query: read 2 registers at address 0x003B
const uint8_t QUERY[8] = {0x01, 0x03, 0x00, 0x3B, 0x00, 0x02, 0xB5, 0xC6};

// ─── CRC-16/MODBUS ───────────────────────────────────────
uint16_t crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;
}

// ─── SETUP ───────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial2.begin(RS485_BAUD, SERIAL_8N1);  // RX2=GPIO16, TX2=GPIO17
    pinMode(PIN_REDE, OUTPUT);
    digitalWrite(PIN_REDE, LOW);            // start in receive mode

    Serial.println("=== MAX3485 + SDL Laser Sensor — Comms Test ===");
    Serial.println("Polling at 1 Hz. Healthy response starts: 01 03 04 ...");
    Serial.println();
}

// ─── LOOP ────────────────────────────────────────────────
void loop() {
    Serial.println("--- Query sent ---");

    // 1. Flush stale RX bytes
    while (Serial2.available()) Serial2.read();

    // 2. Switch to TX, send query
    digitalWrite(PIN_REDE, HIGH);
    Serial2.write(QUERY, sizeof(QUERY));
    Serial2.flush();                        // wait until last bit physically sent

    // 3. Switch back to RX immediately after last bit
    digitalWrite(PIN_REDE, LOW);

    // 4. Collect response bytes
    uint8_t buf[16];
    int received = 0;
    unsigned long t0 = millis();

    while (millis() - t0 < RESPONSE_TIMEOUT_MS) {
        if (Serial2.available()) {
            buf[received++] = Serial2.read();
            if (received >= 9) break;       // full frame received
        }
    }

    // 5. Print raw bytes regardless of validity
    Serial.print("Received ");
    Serial.print(received);
    Serial.print(" bytes: ");
    for (int i = 0; i < received; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // 6. Check frame
    if (received < 9) {
        Serial.println("FAIL: timeout — fewer than 9 bytes received");
        diagnose(received, buf);
        Serial.println();
        delay(1000);
        return;
    }

    if (buf[0] != 0x01) { Serial.println("FAIL: wrong slave address (byte 0)"); Serial.println(); delay(1000); return; }
    if (buf[1] != 0x03) { Serial.println("FAIL: wrong function code (byte 1)"); Serial.println(); delay(1000); return; }
    if (buf[2] != 0x04) { Serial.println("FAIL: wrong byte count (byte 2)");   Serial.println(); delay(1000); return; }

    uint16_t crcCalc = crc16(buf, 7);
    uint16_t crcRecv = (uint16_t)buf[8] << 8 | buf[7];  // little-endian in frame
    if (crcCalc != crcRecv) {
        Serial.print("FAIL: CRC mismatch — calculated 0x");
        Serial.print(crcCalc, HEX);
        Serial.print(", received 0x");
        Serial.println(crcRecv, HEX);
        Serial.println();
        delay(1000);
        return;
    }

    // 7. Decode distance (SDL-100/200/400 = divide by 1000 for mm)
    uint32_t raw = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16)
                 | ((uint32_t)buf[5] <<  8) |  (uint32_t)buf[6];
    float distance_mm = raw / 1000.0f;

    Serial.print("OK  Distance = ");
    Serial.print(distance_mm, 3);
    Serial.println(" mm");
    Serial.println();

    delay(1000);  // 1 Hz poll
}

// ─── DIAGNOSIS HINTS ─────────────────────────────────────
void diagnose(int received, uint8_t* buf) {
    if (received == 0) {
        Serial.println("  Hint: No bytes at all.");
        Serial.println("  Check: 24V sensor power on? A/B wires connected?");
        Serial.println("  Check: D4 goes HIGH during TX? (measure with multimeter)");
        Serial.println("  Check: MAX3485 VCC = 3.3V, GND shared with ESP32 and sensor?");
    } else if (received > 0 && buf[0] != 0x01) {
        Serial.println("  Hint: Got bytes but frame looks wrong.");
        Serial.println("  Check: A and B wires — try swapping them.");
        Serial.println("  Check: Sensor baud rate — try 19200 or 38400 in RS485_BAUD.");
    } else {
        Serial.println("  Hint: Partial frame — sensor replied but was cut short.");
        Serial.println("  Check: Increase RESPONSE_TIMEOUT_MS if this repeats.");
    }
}
