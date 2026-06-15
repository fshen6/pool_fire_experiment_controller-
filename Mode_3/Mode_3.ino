// ============================================================
//  Mode_3.ino — Automatic fuel level control
//  PI outer loop (10 Hz) + P inner voltage loop (50 Hz)
//  Optional Smith Predictor for transport dead-time compensation
//
//  Hardware: ESP32 WROOM + MAX485 + SDL laser sensor + peristaltic pump
// ============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ──────────────────────────────────────────────────────────
//  CONFIGURATION — edit these before each experiment set
// ──────────────────────────────────────────────────────────

// --- Smith Predictor ---
// Set SMITH_PREDICTOR_ENABLED to 1 after running Td_Measurement.ino
// and filling in Td_s and K_MODEL from that experiment's output.
#define SMITH_PREDICTOR_ENABLED  0       // 0 = PI only | 1 = PI + Smith Predictor

#define Td_s        10.0f                // measured transport dead time (seconds)
#define K_MODEL     0.005f               // measured process gain (mm / PWM_count / second)

// --- Outer PI loop (10 Hz, runs on distance error) ---
#define Kp_OUTER        0.3f            // proportional gain (V per mm error)
#define Ki_OUTER        0.02f           // integral gain    (V per mm per second)
#define DEADBAND_MM     0.05f           // ignore error smaller than this
#define INTEGRATOR_MAX  3.0f            // anti-windup clamp (volts)

// --- Inner P loop (50 Hz, tracks V_cmd via voltage sensor) ---
#define Kp_INNER        20.0f           // PWM counts per volt error

// --- Pump voltage limits ---
#define V_MIN   0.5f                    // minimum output voltage (prevents stall)
#define V_MAX  10.0f                    // maximum output voltage

// --- Sensor fault tolerance ---
#define SENSOR_FAIL_MAX  5              // consecutive Modbus failures before freezing PWM

// ──────────────────────────────────────────────────────────
//  PIN MAP
// ──────────────────────────────────────────────────────────

#define PIN_PWM     13      // PWM → voltage converter
#define PIN_POT     32      // potentiometer (for manual VStart override)
#define PIN_VSENSE  33      // voltage sensor (parallel to converter output)
#define PIN_SDA     21
#define PIN_SCL     22
#define PIN_RELAY1  19      // Relay1: HIGH = closed = pump ON
#define PIN_RELAY2  25      // Relay2: HIGH = closed = CCW (drain) | LOW = open = CW (inject)
#define PIN_BTN1    23      // active HIGH (external PULLDOWN)
#define PIN_BTN3    14      // active LOW  (INPUT_PULLUP)
#define PIN_ESTOP   27      // HIGH = active (normally-closed switch to GND — fail-safe)
#define PIN_REDE    4       // MAX485 RE+DE direction pin

// ──────────────────────────────────────────────────────────
//  RS-485 / MODBUS CONSTANTS
// ──────────────────────────────────────────────────────────

#define RS485_BAUD          9600
#define MODBUS_TIMEOUT_MS   50          // < 100ms outer tick; sensor replies in ~20ms

const uint8_t MODBUS_QUERY[8] = {0x01, 0x03, 0x00, 0x3B, 0x00, 0x02, 0xB5, 0xC6};

// ──────────────────────────────────────────────────────────
//  SMITH PREDICTOR RING BUFFER
// ──────────────────────────────────────────────────────────

#define OUTER_PERIOD_S   0.1f           // 10 Hz outer loop
#define MAX_DELAY_SAMPLES 300           // supports up to 30 s dead time

static uint8_t  pwmRingBuf[MAX_DELAY_SAMPLES];
static int      ringHead      = 0;
static int      delaySamples  = 1;     // computed in setup()
static float    modelNow      = 0.0f;
static float    modelDelayed  = 0.0f;
static unsigned long modelResetMs = 0;

// ──────────────────────────────────────────────────────────
//  SHARED STATE
// ──────────────────────────────────────────────────────────

enum Phase { IDLE, RUNNING, ESTOP } phase = IDLE;

static float   DStart       = 0.0f;
static float   VStart       = 0.0f;
static float   V_cmd        = 0.0f;
static float   integrator   = 0.0f;
static uint8_t currentPWM   = 0;
static float   y_real       = 0.0f;    // raw laser distance (mm)
static float   y_corrected  = 0.0f;    // Smith-compensated distance
static float   V_meas       = 0.0f;    // voltage sensor reading

static int  sensorFailCount = 0;
static bool sensorFrozen    = false;

// ──────────────────────────────────────────────────────────
//  LCD
// ──────────────────────────────────────────────────────────

static LiquidCrystal_I2C lcd(0x27, 16, 2);
static bool lcdOk = false;

static void initLcd() {
    Wire.begin(PIN_SDA, PIN_SCL);
    const uint8_t addrs[] = {0x27, 0x3F};
    for (uint8_t addr : addrs) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            lcd = LiquidCrystal_I2C(addr, 16, 2);
            lcd.init();
            lcd.backlight();
            lcdOk = true;
            return;
        }
    }
}

static void lcdShow(const char* l0, const char* l1) {
    if (!lcdOk) return;
    lcd.setCursor(0, 0); lcd.print(l0);
    lcd.setCursor(0, 1); lcd.print(l1);
}

// ──────────────────────────────────────────────────────────
//  CRC-16/MODBUS
// ──────────────────────────────────────────────────────────

static uint16_t crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;
}

// ──────────────────────────────────────────────────────────
//  SENSOR READ
// ──────────────────────────────────────────────────────────

static bool readDistance(float& out_mm) {
    while (Serial2.available()) Serial2.read();     // flush stale bytes

    // Guard 1: let DE fully activate before first bit (see RS485_Direction_Switching_Analysis.md)
    digitalWrite(PIN_REDE, HIGH);
    delayMicroseconds(200);
    Serial2.write(MODBUS_QUERY, 8);
    Serial2.flush();
    // Guard 2: let stop bit propagate before receiver enables
    delayMicroseconds(200);
    digitalWrite(PIN_REDE, LOW);

    uint8_t buf[9];
    int got = 0;
    unsigned long t0 = millis();
    while (got < 9) {
        if (millis() - t0 > MODBUS_TIMEOUT_MS) return false;
        if (Serial2.available())               buf[got++] = Serial2.read();
    }

    if (buf[0] != 0x01 || buf[1] != 0x03 || buf[2] != 0x04) return false;
    uint16_t crcCalc = crc16(buf, 7);
    uint16_t crcRecv = (uint16_t)buf[8] << 8 | buf[7];
    if (crcCalc != crcRecv) return false;

    // Word-swapped decode: wire sends low word first, then high word
    // e.g. B8 47 00 00 → loWord=0xB847, hiWord=0x0000 → 47175 → 47.175 mm
    uint16_t loWord = ((uint16_t)buf[3] << 8) | buf[4];
    uint16_t hiWord = ((uint16_t)buf[5] << 8) | buf[6];
    int32_t  raw    = (int32_t)(((uint32_t)hiWord << 16) | loWord);
    out_mm = raw / 1000.0f;
    return true;
}

// ──────────────────────────────────────────────────────────
//  VOLTAGE SENSOR
// ──────────────────────────────────────────────────────────

#define VSENSE_RATIO    5.305f
#define VSENSE_SAMPLES  8

static float readVoltage() {
    long sum = 0;
    for (int i = 0; i < VSENSE_SAMPLES; i++) {
        sum += analogRead(PIN_VSENSE);
        delay(1);
    }
    float adcV = (sum / (float)VSENSE_SAMPLES) * 3.3f / 4095.0f;
    return adcV * VSENSE_RATIO;
}

// ──────────────────────────────────────────────────────────
//  PUMP CONTROL
// ──────────────────────────────────────────────────────────

static void setPWM(uint8_t v)  { currentPWM = v; analogWrite(PIN_PWM, v); }
static void pumpStop()         { digitalWrite(PIN_RELAY1, LOW);  }
static void pumpRunCW()        { digitalWrite(PIN_RELAY2, LOW);  digitalWrite(PIN_RELAY1, HIGH); }
static void pumpRunCCW()       { digitalWrite(PIN_RELAY2, HIGH); digitalWrite(PIN_RELAY1, HIGH); }

// ──────────────────────────────────────────────────────────
//  SMITH PREDICTOR
// ──────────────────────────────────────────────────────────

static float smithCorrection(uint8_t curPWM) {
    uint8_t u_delayed = pwmRingBuf[ringHead];

    modelNow     += K_MODEL * (float)curPWM     * OUTER_PERIOD_S;
    modelDelayed += K_MODEL * (float)u_delayed  * OUTER_PERIOD_S;

    pwmRingBuf[ringHead] = curPWM;
    ringHead = (ringHead + 1) % delaySamples;

    // Prevent model integrator drift — reset every 60 s
    if (millis() - modelResetMs >= 60000UL) {
        modelNow = modelDelayed = 0.0f;
        modelResetMs = millis();
    }

    return modelNow - modelDelayed;
}

// ──────────────────────────────────────────────────────────
//  HELPERS
// ──────────────────────────────────────────────────────────

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static bool btn3Debounced() {
    // Returns true on the rising edge (press event) of BTN3 (active LOW)
    static bool prev = false;
    bool now = (digitalRead(PIN_BTN3) == LOW);
    bool edge = now && !prev;
    prev = now;
    return edge;
}

// ──────────────────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial2.begin(RS485_BAUD, SERIAL_8N1);   // RX2=GPIO16, TX2=GPIO17

    pinMode(PIN_REDE,   OUTPUT); digitalWrite(PIN_REDE,   LOW);
    pinMode(PIN_RELAY1, OUTPUT); digitalWrite(PIN_RELAY1, LOW);
    pinMode(PIN_RELAY2, OUTPUT); digitalWrite(PIN_RELAY2, LOW);
    pinMode(PIN_BTN1,   INPUT);              // external 10k PULLDOWN, active HIGH
    pinMode(PIN_BTN3,   INPUT_PULLUP);       // active LOW
    pinMode(PIN_ESTOP,  INPUT_PULLUP);       // normally-closed to GND; HIGH = active

    analogReadResolution(12);
    analogWriteFrequency(PIN_PWM, 1000);
    analogWriteResolution(PIN_PWM, 8);
    analogWrite(PIN_PWM, 0);

    delaySamples = max(1, (int)roundf(Td_s / OUTER_PERIOD_S));
    delaySamples = min(delaySamples, MAX_DELAY_SAMPLES);
    memset(pwmRingBuf, 0, sizeof(pwmRingBuf));

    initLcd();
    lcdShow("Mode 3 Ready    ", "B3=Start        ");

    // CSV header on serial
    Serial.println("# Pool Fire Controller — Mode 3");
    Serial.print  ("# Smith Predictor: ");
    Serial.println(SMITH_PREDICTOR_ENABLED ? "ENABLED" : "DISABLED");
    Serial.print  ("# Td="); Serial.print(Td_s, 2);
    Serial.print  ("s  K="); Serial.println(K_MODEL, 5);
    Serial.println("timestamp_ms,distance_mm,y_corrected_mm,V_meas,V_cmd,PWM,integrator,phase");
}

// ──────────────────────────────────────────────────────────
//  LOOP
// ──────────────────────────────────────────────────────────

static unsigned long tFast = 0, tMed = 0, tSlow = 0;

void loop() {
    unsigned long now = millis();

    // ─── E-STOP — runs every iteration, overrides everything ───
    bool eStopActive = (digitalRead(PIN_ESTOP) == HIGH);
    if (eStopActive) {
        if (phase != ESTOP) {
            phase = ESTOP;
            pumpRunCCW();
            setPWM(255);
        }
        // Still update LCD and log at slow/med rates even in E-stop
        if (now - tSlow >= 500) {
            tSlow = now;
            lcdShow("!! E-STOP !!    ", "DRAIN CCW MAX   ");
        }
        if (now - tMed >= 100) {
            tMed = now;
            float dist;
            if (readDistance(dist)) y_real = dist;
            Serial.print(now); Serial.print(",");
            Serial.print(y_real, 3); Serial.print(",,,,,, ESTOP");
            Serial.println();
        }
        return;
    }

    // E-stop just cleared
    if (phase == ESTOP) {
        pumpStop();
        setPWM(0);
        integrator = 0.0f;
        modelNow = modelDelayed = 0.0f;
        phase = IDLE;
        lcdShow("E-stop cleared  ", "B3=Restart      ");
    }

    // ─── BUTTON 3 — toggle IDLE ↔ RUNNING ───
    if (btn3Debounced()) {
        if (phase == IDLE) {
            // Capture setpoints at start
            DStart    = y_real;
            VStart    = V_meas;
            V_cmd     = VStart;
            integrator = 0.0f;

            // Pre-fill ring buffer with current PWM for bumpless Smith start
            memset(pwmRingBuf, currentPWM, sizeof(pwmRingBuf));
            modelNow = modelDelayed = 0.0f;
            modelResetMs = now;

            pumpRunCW();
            phase = RUNNING;

            Serial.print("# START  DStart="); Serial.print(DStart, 3);
            Serial.print("mm  VStart=");      Serial.print(VStart, 3);
            Serial.println("V");
        } else if (phase == RUNNING) {
            // Return to pot-controlled IDLE — pump keeps running at pot speed
            integrator = 0.0f;
            phase = IDLE;
            Serial.println("# STOPPED by B3 — returning to pot control");
        }
    }

    // ─── FAST TICK: 50 Hz — inner P voltage loop (RUNNING) or pot control (IDLE) ───
    if (now - tFast >= 20) {
        tFast = now;
        V_meas = readVoltage();

        if (phase == RUNNING) {
            // Auto: inner P loop drives PWM to hit V_cmd
            float vErr   = V_cmd - V_meas;
            float newPWM = (float)currentPWM + Kp_INNER * vErr * 0.02f;
            setPWM((uint8_t)clampf(newPWM, 0.0f, 255.0f));
        } else if (phase == IDLE) {
            // Manual: potentiometer directly sets pump speed (pump runs CW in IDLE)
            int potRaw = analogRead(PIN_POT);
            uint8_t potPWM = (uint8_t)map(potRaw, 0, 4095, 0, 255);
            pumpRunCW();
            setPWM(potPWM);
        }
    }

    // ─── MED TICK: 10 Hz — laser + outer PI + Smith ───
    if (now - tMed >= 100) {
        tMed = now;

        // Read laser distance
        float newDist;
        if (readDistance(newDist)) {
            y_real = newDist;
            sensorFailCount = 0;
            sensorFrozen    = false;
        } else {
            sensorFailCount++;
            if (sensorFailCount >= SENSOR_FAIL_MAX) sensorFrozen = true;
        }

        if (phase == RUNNING && !sensorFrozen) {
            // Smith Predictor correction
            float correction = 0.0f;
            if (SMITH_PREDICTOR_ENABLED) {
                correction = smithCorrection(currentPWM);
            }
            y_corrected = y_real + correction;

            // Outer PI loop
            float error = y_corrected - DStart;  // positive → fuel too low → inject more
            if (fabsf(error) < DEADBAND_MM) error = 0.0f;

            integrator += Ki_OUTER * error * OUTER_PERIOD_S;
            integrator = clampf(integrator, -INTEGRATOR_MAX, INTEGRATOR_MAX);

            V_cmd = VStart + Kp_OUTER * error + integrator;
            V_cmd = clampf(V_cmd, V_MIN, V_MAX);
        } else if (phase == RUNNING && sensorFrozen) {
            // Hold current V_cmd — do not update
        }

        // CSV log every 100 ms
        const char* phaseStr = (phase == RUNNING) ? "RUN" :
                               (phase == ESTOP)   ? "ESTOP" : "IDLE";
        Serial.print(now);          Serial.print(',');
        Serial.print(y_real, 3);    Serial.print(',');
        Serial.print(y_corrected, 3); Serial.print(',');
        Serial.print(V_meas, 3);    Serial.print(',');
        Serial.print(V_cmd, 3);     Serial.print(',');
        Serial.print(currentPWM);   Serial.print(',');
        Serial.print(integrator, 4); Serial.print(',');
        Serial.println(phaseStr);
    }

    // ─── SLOW TICK: 2 Hz — LCD update ───
    if (now - tSlow >= 500) {
        tSlow = now;
        char l0[17], l1[17];

        if (phase == RUNNING) {
            float err = y_real - DStart;
            // Line 0: distance and error
            snprintf(l0, sizeof(l0), "D%6.2f E%+5.2f", y_real, err);
            // Line 1: voltage measured→commanded, PWM
            snprintf(l1, sizeof(l1), "V%4.1f>%4.1f P%3d", V_meas, V_cmd, currentPWM);
            if (sensorFrozen) {
                snprintf(l0, sizeof(l0), "SENSOR FAULT!   ");
            }
        } else {
            // IDLE: show distance and current pot-driven voltage
            snprintf(l0, sizeof(l0), "IDLE D%7.3f mm", y_real);
            snprintf(l1, sizeof(l1), "V%4.1f P%3d B3=GO", V_meas, currentPWM);
        }

        lcdShow(l0, l1);
    }
}
