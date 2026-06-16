/*
 * Response_Time_Test.ino
 * -----------------------------------------------------------------------
 * Measures the hydraulic response time between pump injection and the
 * corresponding level change visible in the sensing container.
 *
 * Procedure (all automatic after button press):
 *   1. Press B3  → 10 s baseline recording (distance only, pump OFF)
 *   2. Auto      → pump fires CCW (drain) at target voltage for 5 s (step input)
 *   3. Auto      → 60 s post-step recording (pump OFF, watch for level change)
 *   4. Done      → summary printed; press B3 again to repeat
 *
 * Output: CSV on Serial at 115200 baud — paste into Excel or plot in Python.
 * Columns: timestamp_ms, phase, distance_mm, voltage_V
 *
 * Wiring:
 *   MAX3485 RO  → GPIO16 (RX2)     MAX3485 DI  ← GPIO17 (TX2)
 *   MAX3485 RE+DE → GPIO4           Relay1 (run) → GPIO19   HIGH=ON
 *   Relay2 (dir)  → GPIO25          LOW=CW(inject)
 *   PWM out       → GPIO13          Voltage sensor → GPIO33
 *   B3            → GPIO14          INPUT_PULLUP, press=LOW
 * -----------------------------------------------------------------------
 */

// ── Pins ────────────────────────────────────────────────────────────────
#define PIN_RX2     16
#define PIN_TX2     17
#define PIN_REDE     4
#define PIN_RELAY1  19    // HIGH = pump ON
#define PIN_RELAY2  25    // LOW  = CW (inject)
#define PIN_PWM     13
#define PIN_VSENSE  33
#define PIN_BTN3    14    // INPUT_PULLUP, press = LOW

// ── Test parameters ─────────────────────────────────────────────────────
#define BASELINE_MS   10000   // 10 s of recording before pump fires
#define PUMP_ON_MS     5000   // 5 s pump burst (step input)
#define POST_RECORD_MS 60000  // 60 s recording after pump stops
#define SAMPLE_MS       100   // 10 Hz sampling rate

// Target pump voltage. Converter maps PWM 0–255 → roughly 0–10 V.
// 7 V ≈ PWM 178. Verify against voltage sensor reading on first run
// and adjust if your converter output differs.
#define PUMP_PWM   178    // ≈ 7 V — tune if voltage sensor shows different

// ── RS-485 / Modbus ─────────────────────────────────────────────────────
#define SENSOR_BAUD         9600
#define RESPONSE_TIMEOUT_MS   50
#define DISTANCE_DIVISOR    1000.0f   // SDL-100/200/400: 1 µm per count

HardwareSerial RS485(2);

uint8_t modbusQuery[8];

uint16_t crc16(const uint8_t *d, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;
}

void buildQuery() {
    modbusQuery[0] = 0x01; modbusQuery[1] = 0x03;
    modbusQuery[2] = 0x00; modbusQuery[3] = 0x3B;
    modbusQuery[4] = 0x00; modbusQuery[5] = 0x02;
    uint16_t c = crc16(modbusQuery, 6);
    modbusQuery[6] = c & 0xFF; modbusQuery[7] = c >> 8;
}

// Returns true and fills dist_mm, false on timeout/CRC failure
bool readDistance(float &dist_mm) {
    while (RS485.available()) RS485.read();  // flush stale bytes

    digitalWrite(PIN_REDE, HIGH);
    delayMicroseconds(200);                  // Guard 1: driver settle
    RS485.write(modbusQuery, 8);
    RS485.flush();
    delayMicroseconds(200);                  // Guard 2: bus settle
    digitalWrite(PIN_REDE, LOW);

    uint8_t buf[9];
    int got = 0;
    uint32_t t0 = millis();
    while (got < 9 && millis() - t0 < RESPONSE_TIMEOUT_MS)
        if (RS485.available()) buf[got++] = RS485.read();

    if (got < 9) return false;
    if (buf[0] != 0x01 || buf[1] != 0x03 || buf[2] != 0x04) return false;
    uint16_t crcCalc = crc16(buf, 7);
    uint16_t crcRecv = ((uint16_t)buf[8] << 8) | buf[7];
    if (crcCalc != crcRecv) return false;

    // Word-swapped: low word on wire first, then high word
    uint16_t lo = ((uint16_t)buf[3] << 8) | buf[4];
    uint16_t hi = ((uint16_t)buf[5] << 8) | buf[6];
    int32_t  raw = (int32_t)(((uint32_t)hi << 16) | lo);
    dist_mm = raw / DISTANCE_DIVISOR;
    return true;
}

// 8-sample averaged voltage sensor read
float readVoltage() {
    long sum = 0;
    for (int i = 0; i < 8; i++) { sum += analogRead(PIN_VSENSE); delay(1); }
    return (sum / 8.0f) * 3.3f / 4095.0f * 5.305f;
}

// ── Pump helpers ─────────────────────────────────────────────────────────
void pumpStop()  { digitalWrite(PIN_RELAY1, LOW);  analogWrite(PIN_PWM, 0); }
void pumpFireCCW(uint8_t pwm) {
    digitalWrite(PIN_RELAY2, HIGH);  // CCW = drain (reversed direction)
    digitalWrite(PIN_RELAY1, HIGH);  // run
    analogWrite(PIN_PWM, pwm);
}

// ── Button edge detect ───────────────────────────────────────────────────
bool btn3Pressed() {
    static bool prev = false;
    bool now = (digitalRead(PIN_BTN3) == LOW);
    bool edge = now && !prev;
    prev = now;
    return edge;
}

// ── State machine ────────────────────────────────────────────────────────
enum State { IDLE, BASELINE, PUMP_ON, POST_RECORD, DONE } state = IDLE;

uint32_t phaseStart = 0;
uint32_t lastSample = 0;
float    distAtPumpStart = 0.0f;
int      sampleCount = 0;

void setup() {
    Serial.begin(115200);
    RS485.begin(SENSOR_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
    buildQuery();

    pinMode(PIN_REDE,   OUTPUT); digitalWrite(PIN_REDE,   LOW);
    pinMode(PIN_RELAY1, OUTPUT); pumpStop();
    pinMode(PIN_RELAY2, OUTPUT); digitalWrite(PIN_RELAY2, LOW);
    pinMode(PIN_BTN3,   INPUT_PULLUP);

    analogReadResolution(12);
    analogWriteFrequency(PIN_PWM, 1000);
    analogWriteResolution(PIN_PWM, 8);

    Serial.println();
    Serial.println(F("# Response Time Test"));
    Serial.println(F("# Press B3 to start. Sequence: 10s baseline → 1s pump → 30s record"));
    Serial.print  (F("# Pump PWM = ")); Serial.print(PUMP_PWM);
    Serial.println(F(" (check voltage sensor on first sample of PUMP_ON phase)"));
    Serial.println(F("timestamp_ms,phase,distance_mm,voltage_V"));
}

void logSample(const char *phase) {
    float dist = 0.0f;
    bool  ok   = readDistance(dist);
    float v    = readVoltage();

    Serial.print(millis());     Serial.print(',');
    Serial.print(phase);        Serial.print(',');
    if (ok) Serial.print(dist, 3); else Serial.print(F("ERR"));
    Serial.print(',');
    Serial.println(v, 3);

    if (ok && state == PUMP_ON && sampleCount == 0) {
        distAtPumpStart = dist;  // capture baseline level at pump fire moment
    }
    sampleCount++;
}

void loop() {
    uint32_t now = millis();

    switch (state) {

        case IDLE:
            if (btn3Pressed()) {
                Serial.println(F("# --- NEW RUN ---"));
                sampleCount   = 0;
                phaseStart    = now;
                lastSample    = now;
                state         = BASELINE;
                Serial.println(F("# BASELINE started (10 s, pump OFF)"));
            }
            break;

        case BASELINE:
            if (now - lastSample >= SAMPLE_MS) {
                lastSample = now;
                logSample("BASELINE");
            }
            if (now - phaseStart >= BASELINE_MS) {
                // Fire pump
                pumpFireCCW(PUMP_PWM);
                phaseStart  = now;
                lastSample  = now;
                sampleCount = 0;
                state       = PUMP_ON;
                Serial.print(F("# PUMP_ON CCW ("));
                Serial.print(PUMP_PWM);
                Serial.println(F(" PWM, 5 s)"));
            }
            break;

        case PUMP_ON:
            if (now - lastSample >= SAMPLE_MS) {
                lastSample = now;
                logSample("PUMP_ON");
            }
            if (now - phaseStart >= PUMP_ON_MS) {
                pumpStop();
                phaseStart  = now;
                lastSample  = now;
                sampleCount = 0;
                state       = POST_RECORD;
                Serial.println(F("# POST_RECORD (pump OFF, watching for level change)"));
            }
            break;

        case POST_RECORD:
            if (now - lastSample >= SAMPLE_MS) {
                lastSample = now;
                logSample("POST");
            }
            if (now - phaseStart >= POST_RECORD_MS) {
                state = DONE;
                // Print summary
                Serial.println(F("# ── SUMMARY ──────────────────────────────────────"));
                Serial.print  (F("# Distance at pump start : "));
                Serial.print  (distAtPumpStart, 3); Serial.println(F(" mm"));
                Serial.println(F("# Draining (CCW): level drops -> distance INCREASES."));
                Serial.println(F("# Look at the CSV: find first POST row where"));
                Serial.println(F("#   distance_mm > (distance at pump start + 0.05 mm)"));
                Serial.println(F("# That timestamp - pump start timestamp = response time"));
                Serial.println(F("# ───────────────────────────────────────────────────"));
                Serial.println(F("# Press B3 to run again."));
            }
            break;

        case DONE:
            if (btn3Pressed()) {
                state = IDLE;
                // short delay so the edge doesn't re-trigger immediately
                delay(300);
            }
            break;
    }
}
