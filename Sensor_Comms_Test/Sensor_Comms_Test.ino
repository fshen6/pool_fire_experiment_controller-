// ============================================================
//  Sensor_Comms_Test.ino
//  SDL laser distance sensor — basic comms test
//
//  Module: MAX3485 with RE/DE pins (3.3V native, no level shifting needed)
//  Wiring:
//    Module RO      → ESP32 RX2 (GPIO 16)
//    Module DI      → ESP32 TX2 (GPIO 17)
//    Module RE + DE (tied together) → ESP32 D4 (GPIO 4)
//    Module VCC     → ESP32 3.3V
//    Module GND     → common GND
//    Sensor RS-485 A → Module A
//    Sensor RS-485 B → Module B
//    Sensor powered from 24V (common GND with ESP32)
//
//  Open Serial Monitor at 115200 baud.
//  Healthy response starts: 01 03 04 ...
// ============================================================

#define PIN_REDE            4     // RE and DE tied together, driven from this pin
#define RS485_BAUD          9600
#define RESPONSE_TIMEOUT_MS 200

const uint8_t QUERY[8] = {0x01, 0x03, 0x00, 0x3B, 0x00, 0x02, 0xB5, 0xC6};

uint16_t crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(RS485_BAUD, SERIAL_8N1);  // RX2=GPIO16, TX2=GPIO17
    pinMode(PIN_REDE, OUTPUT);
    digitalWrite(PIN_REDE, LOW);            // start in receive mode
    delay(500);
    Serial.println("=== SDL Sensor Comms Test (MAX3485 with RE/DE) ===");
    Serial.println("Polling at 1 Hz. Good response: 01 03 04 ...");
    Serial.println();
}

void loop() {
    // Flush stale bytes
    while (Serial2.available()) Serial2.read();

    // Switch to TX, send query, wait until last bit physically sent, switch back to RX
    digitalWrite(PIN_REDE, HIGH);
    Serial2.write(QUERY, sizeof(QUERY));
    Serial2.flush();                        // critical: wait until UART finishes
    digitalWrite(PIN_REDE, LOW);            // switch to RX before sensor replies

    // Collect response
    uint8_t buf[9];
    int got = 0;
    unsigned long t0 = millis();
    while (got < 9 && millis() - t0 < RESPONSE_TIMEOUT_MS) {
        if (Serial2.available()) buf[got++] = Serial2.read();
    }

    // Print raw bytes
    Serial.print("Raw (");
    Serial.print(got);
    Serial.print(" bytes): ");
    for (int i = 0; i < got; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Validate and decode
    if (got < 9) {
        Serial.println("FAIL: timeout — check 24V power, A/B wires, baud rate");
    } else if (buf[0] != 0x01 || buf[1] != 0x03 || buf[2] != 0x04) {
        Serial.println("FAIL: bad frame — try swapping A and B wires");
    } else {
        uint16_t crcCalc = crc16(buf, 7);
        uint16_t crcRecv = (uint16_t)buf[8] << 8 | buf[7];
        if (crcCalc != crcRecv) {
            Serial.println("FAIL: CRC mismatch — noise or baud rate mismatch");
        } else {
            uint32_t raw = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16)
                         | ((uint32_t)buf[5] <<  8) |  (uint32_t)buf[6];
            Serial.print("OK  Distance = ");
            Serial.print(raw / 1000.0f, 3);
            Serial.println(" mm");
        }
    }

    Serial.println();
    delay(1000);
}
