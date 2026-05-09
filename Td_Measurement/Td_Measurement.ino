/*
 * Td_Measurement.ino
 * Transport Dead Time (Td) Characterisation Experiment
 * Pool Fire Level Control System — ESP32 WROOM
 *
 * Procedure:
 *   1. Press BTN1  → 30 s baseline (pump OFF, sensor logged, noise computed)
 *   2. Auto-step   → Pump ON at TEST_PWM, wait for sensor to respond
 *   3. Auto-detect → 3 consecutive readings above mean+3σ = Td detected
 *   4. Press BTN1  → repeat trial (up to MAX_TRIALS)
 *   5. After final trial → mean Td, σ_Td, K_MODEL printed to Serial
 *
 *   BTN3 at any time → abort and reset to IDLE
 *   E-STOP          → immediate pump off, sketch halts
 *
 * Serial output: CSV, 115200 baud. Copy to file for analysis.
 * See Td_Experiment_Report.md for full procedure and expected graphs.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ============================================================
// TEST PARAMETERS — SET BEFORE UPLOADING
// ============================================================
#define TEST_PWM              128     // Pump PWM for step test (0–255). Match your VStart.
#define BASELINE_DURATION_MS  30000   // Duration of stationary baseline (ms)
#define STEP_TIMEOUT_MS       90000   // Max time to wait for sensor response (ms)
#define SIGMA_MULTIPLIER      3.0f    // Detection = mean + N × sigma
#define CONSEC_REQUIRED       3       // Consecutive readings above threshold to confirm Td
#define MAX_TRIALS            5       // Number of repeat trials before final summary
#define SAMPLE_INTERVAL_MS    100     // 10 Hz sensor sampling

// ============================================================
// PIN DEFINITIONS — match your hardware
// ============================================================
#define PIN_RELAY1    19    // Pump start/stop: HIGH = ON, LOW = OFF
#define PIN_RELAY2    25    // Direction: LOW = CW (injection), HIGH = CCW
#define PIN_PWM       13    // PWM output to converter
#define PIN_REDE       4    // MAX485 direction: HIGH = TX, LOW = RX
#define PIN_BTN1      23    // Start/next trial (INPUT_PULLDOWN, active HIGH)
#define PIN_BTN3      14    // Abort/reset      (INPUT_PULLUP,   active LOW)
#define PIN_ESTOP     27    // Emergency stop   (INPUT_PULLUP,   HIGH = triggered)

#define RELAY_ON   HIGH
#define RELAY_OFF  LOW

// ============================================================
// MODBUS RTU — read distance register 0x003B (2 registers)
// ============================================================
const uint8_t modbusQuery[8] = {0x01, 0x03, 0x00, 0x3B, 0x00, 0x02, 0xB5, 0xC6};

// ============================================================
// LCD
// ============================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============================================================
// STATE MACHINE
// ============================================================
enum State {
    STATE_IDLE,
    STATE_BASELINE,
    STATE_SETTLING,
    STATE_STEP,
    STATE_DETECTED,
    STATE_COMPLETE
};
State state = STATE_IDLE;
unsigned long stateEnteredMs = 0;

// ============================================================
// BASELINE STATISTICS
// ============================================================
double  blSum    = 0;
double  blSumSq  = 0;
int     blCount  = 0;
float   blMean   = 0;
float   blSigma  = 0;
float   detThreshold = 0;

// ============================================================
// STEP / DETECTION DATA
// ============================================================
unsigned long stepStartMs    = 0;
int           consecCount    = 0;
bool          firstAbove     = false;
unsigned long firstAboveMs   = 0;
float         lastValidDist  = 0;       // used for K_MODEL extraction
unsigned long lastValidMs    = 0;

// ============================================================
// PROCESS GAIN (K_MODEL) EXTRACTION
// ============================================================
// After detection, track the rising slope for K extraction
bool   kCapturing   = false;
float  kStartDist   = 0;
unsigned long kStartMs = 0;
float  kSlopeSum    = 0;
int    kSlopeCount  = 0;
#define K_CAPTURE_SAMPLES 20   // 20 samples = 2 s for slope estimation

// ============================================================
// TRIAL RESULTS
// ============================================================
float tdTrials[MAX_TRIALS];
float kTrials[MAX_TRIALS];
int   trialCount = 0;

// ============================================================
// TIMING
// ============================================================
unsigned long lastSampleMs   = 0;
unsigned long lastLcdMs      = 0;

// ============================================================
//  RS-485 DISTANCE READ
// ============================================================
bool readDistance(float &dist_mm) {
    while (Serial2.available()) Serial2.read();

    digitalWrite(PIN_REDE, HIGH);
    Serial2.write(modbusQuery, sizeof(modbusQuery));
    Serial2.flush();
    digitalWrite(PIN_REDE, LOW);

    uint8_t buf[9];
    int received = 0;
    unsigned long t0 = millis();
    while (received < 9 && millis() - t0 < 100) {
        if (Serial2.available()) buf[received++] = Serial2.read();
    }

    if (received < 9)                      return false;
    if (buf[0] != 0x01 || buf[1] != 0x03) return false;
    if (buf[2] != 0x04)                    return false;

    uint32_t raw = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16)
                 | ((uint32_t)buf[5] <<  8) |  (uint32_t)buf[6];
    dist_mm = raw / 1000.0f;
    return true;
}

// ============================================================
// PUMP CONTROL
// ============================================================
void pumpOff() {
    ledcWrite(PIN_PWM, 0);
    digitalWrite(PIN_RELAY1, RELAY_OFF);
    digitalWrite(PIN_RELAY2, RELAY_OFF);
}

void pumpCW(int pwm) {
    pwm = constrain(pwm, 0, 255);
    digitalWrite(PIN_RELAY2, RELAY_OFF);   // CW = injection
    digitalWrite(PIN_RELAY1, RELAY_ON);    // start
    ledcWrite(PIN_PWM, pwm);
}

// ============================================================
// E-STOP — call every loop iteration
// ============================================================
void checkEStop() {
    if (digitalRead(PIN_ESTOP) == HIGH) {
        pumpOff();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("**** E-STOP ****");
        lcd.setCursor(0, 1);
        lcd.print("Power cycle ESP ");
        Serial.println();
        Serial.println("# ==========================================");
        Serial.println("# E-STOP TRIGGERED — pump halted, sketch stopped");
        Serial.println("# ==========================================");
        while (true) delay(100);
    }
}

// ============================================================
// BUTTON EDGE DETECTION
// ============================================================
bool btn1Pressed() {
    static bool last = false;
    bool cur  = (digitalRead(PIN_BTN1) == HIGH);
    bool edge = cur && !last;
    last = cur;
    return edge;
}

bool btn3Pressed() {
    static bool last = false;
    bool cur  = (digitalRead(PIN_BTN3) == LOW);   // active LOW
    bool edge = cur && !last;
    last = cur;
    return edge;
}

// ============================================================
// STATE: IDLE
// ============================================================
void enterIdle() {
    state = STATE_IDLE;
    stateEnteredMs = millis();
    pumpOff();

    Serial.println("# STATE: IDLE");
    if (trialCount == 0) {
        Serial.println("# Press BTN1 to start baseline characterisation");
    } else {
        Serial.print("# Trial ");
        Serial.print(trialCount);
        Serial.print("/");
        Serial.print(MAX_TRIALS);
        Serial.println(" complete. Allow level to settle, then press BTN1 for next trial.");
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    if (trialCount == 0) {
        lcd.print("Td Experiment   ");
        lcd.setCursor(0, 1);
        lcd.print("BTN1 = Start    ");
    } else {
        lcd.print("Trial ");
        lcd.print(trialCount);
        lcd.print(" done   ");
        lcd.setCursor(0, 1);
        lcd.print("BTN1 = Next     ");
    }
}

// ============================================================
// STATE: BASELINE
// ============================================================
void enterBaseline() {
    state = STATE_BASELINE;
    stateEnteredMs = millis();
    blSum = 0; blSumSq = 0; blCount = 0;
    pumpOff();

    Serial.println("# ==========================================");
    Serial.print("# BASELINE — Trial ");
    Serial.print(trialCount + 1);
    Serial.println(" (pump OFF, do not disturb rig)");
    Serial.println("# ==========================================");
    Serial.println("timestamp_ms,distance_mm,phase,note");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BASELINE        ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
}

void updateBaseline(float dist, unsigned long nowMs) {
    unsigned long elapsed = nowMs - stateEnteredMs;

    blSum   += dist;
    blSumSq += (double)dist * dist;
    blCount++;

    Serial.print(elapsed);
    Serial.print(",");
    Serial.print(dist, 4);
    Serial.println(",BASELINE,");

    int remaining = (int)((BASELINE_DURATION_MS - elapsed) / 1000) + 1;
    lcd.setCursor(0, 1);
    lcd.print("Rem: ");
    if (remaining < 10) lcd.print(" ");
    lcd.print(remaining);
    lcd.print("s               ");

    if (elapsed < BASELINE_DURATION_MS) return;

    // Compute stats
    blMean      = (float)(blSum / blCount);
    float var   = (float)(blSumSq / blCount) - blMean * blMean;
    blSigma     = sqrtf(var > 0 ? var : 0);
    detThreshold = blMean + SIGMA_MULTIPLIER * blSigma;

    Serial.println("# --- BASELINE COMPLETE ---");
    Serial.print("# D_baseline = "); Serial.print(blMean,       4); Serial.println(" mm");
    Serial.print("# sigma      = "); Serial.print(blSigma*1000, 2); Serial.println(" um");
    Serial.print("# threshold  = "); Serial.print(detThreshold, 4); Serial.println(" mm");
    if (blSigma >= 0.05f) {
        Serial.println("# WARNING: sigma >= 0.05 mm — check for vibration sources before continuing");
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Thr:");
    lcd.print(detThreshold, 3);
    lcd.print("mm  ");
    lcd.setCursor(0, 1);
    lcd.print("sig:");
    lcd.print(blSigma * 1000.0f, 1);
    lcd.print("um       ");

    state = STATE_SETTLING;
    stateEnteredMs = millis();
}

// ============================================================
// STATE: SETTLING (3 s countdown before step)
// ============================================================
void updateSettling(unsigned long nowMs) {
    unsigned long elapsed = nowMs - stateEnteredMs;
    int countdown = (int)((3000 - elapsed) / 1000) + 1;

    if (millis() - lastLcdMs > 250) {
        lastLcdMs = millis();
        lcd.setCursor(0, 1);
        lcd.print("Step in ");
        lcd.print(countdown);
        lcd.print("s...    ");
    }

    if (elapsed < 3000) return;

    // Switch to STEP
    state        = STATE_STEP;
    stateEnteredMs = millis();
    stepStartMs  = millis();
    consecCount  = 0;
    firstAbove   = false;
    kCapturing   = false;
    kSlopeSum    = 0;
    kSlopeCount  = 0;

    pumpCW(TEST_PWM);

    Serial.println("# --- STEP START ---");
    Serial.print("# t_start = "); Serial.println(stepStartMs);
    Serial.print("# PWM     = "); Serial.println(TEST_PWM);
    Serial.println("timestamp_ms,distance_mm,phase,note");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("STEP ACTIVE     ");
    lcd.setCursor(0, 1);
    lcd.print("Watching...     ");
}

// ============================================================
// STATE: STEP — watching for sensor response
// ============================================================
void updateStep(float dist, unsigned long nowMs) {
    unsigned long elapsed = nowMs - stepStartMs;

    bool above = (dist > detThreshold);

    // Track consecutive detections
    if (above) {
        if (!firstAbove) {
            firstAbove   = true;
            firstAboveMs = nowMs;
            consecCount  = 1;
        } else {
            consecCount++;
        }
    } else {
        firstAbove  = false;
        consecCount = 0;
    }

    // Log
    Serial.print(elapsed);
    Serial.print(",");
    Serial.print(dist, 4);
    Serial.print(",STEP,");
    if (above) {
        Serial.print("ABOVE(");
        Serial.print(consecCount);
        Serial.print(")");
    }
    Serial.println();

    // LCD update
    if (millis() - lastLcdMs > 150) {
        lastLcdMs = millis();
        lcd.setCursor(0, 0);
        lcd.print("D:");
        lcd.print(dist, 3);
        lcd.print("mm     ");
        lcd.setCursor(0, 1);
        lcd.print("t:");
        lcd.print(elapsed / 1000.0f, 1);
        lcd.print("s ");
        lcd.print(consecCount);
        lcd.print("/");
        lcd.print(CONSEC_REQUIRED);
        lcd.print("       ");
    }

    // Detection confirmed
    if (consecCount >= CONSEC_REQUIRED) {
        float td_s = (firstAboveMs - stepStartMs) / 1000.0f;
        tdTrials[trialCount] = td_s;

        // Start K_MODEL capture
        kCapturing = true;
        kStartDist = dist;
        kStartMs   = nowMs;

        // Capture K over next K_CAPTURE_SAMPLES samples before stopping
        // Do this inline — keep pump on, capture slope, then stop
        float prevDist = dist;
        unsigned long prevMs = nowMs;

        Serial.println("# --- DETECTION ---");
        Serial.print("# t_detect = "); Serial.println(firstAboveMs);
        Serial.print("# Td       = "); Serial.print(td_s, 3); Serial.println(" s");
        Serial.println("# Capturing process gain (pump stays ON for 2 s)...");

        for (int i = 0; i < K_CAPTURE_SAMPLES; i++) {
            delay(SAMPLE_INTERVAL_MS);
            float kDist = 0;
            if (readDistance(kDist)) {
                unsigned long kNow = millis();
                float dt   = (kNow - prevMs) / 1000.0f;
                float slope = (dt > 0) ? (kDist - prevDist) / dt : 0;
                if (slope > 0) {
                    kSlopeSum += slope;
                    kSlopeCount++;
                }
                prevDist = kDist;
                prevMs   = kNow;
                Serial.print(kNow - stepStartMs);
                Serial.print(",");
                Serial.print(kDist, 4);
                Serial.println(",K_CAPTURE,");
            }
        }

        pumpOff();

        float kProcess = (kSlopeCount > 0) ? (kSlopeSum / kSlopeCount) : 0;
        float kModel   = (TEST_PWM > 0) ? (kProcess / TEST_PWM) : 0;
        kTrials[trialCount] = kModel;

        trialCount++;

        Serial.println("# --- TRIAL RESULT ---");
        Serial.print("# Trial    = "); Serial.println(trialCount);
        Serial.print("# Td       = "); Serial.print(td_s,    3); Serial.println(" s");
        Serial.print("# K_process= "); Serial.print(kProcess, 6); Serial.println(" mm/s");
        Serial.print("# K_MODEL  = "); Serial.print(kModel,   8); Serial.println(" mm/PWM/s");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Td=");
        lcd.print(td_s, 2);
        lcd.print("s  T");
        lcd.print(trialCount);
        lcd.print("     ");

        if (trialCount >= MAX_TRIALS) {
            state = STATE_COMPLETE;
            stateEnteredMs = millis();
            printFinalResults();
        } else {
            state = STATE_DETECTED;
            stateEnteredMs = millis();
            Serial.println("# Allow level to settle, then press BTN1 for next trial (BTN3 = reset)");
            lcd.setCursor(0, 1);
            lcd.print("BTN1=next trial ");
        }
        return;
    }

    // Timeout
    if (elapsed > STEP_TIMEOUT_MS) {
        pumpOff();
        Serial.println("# TIMEOUT: No response detected within limit");
        Serial.println("# Check pump output, RS-485 connection, and threshold value");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("TIMEOUT         ");
        lcd.setCursor(0, 1);
        lcd.print("BTN1=retry      ");

        state = STATE_DETECTED;
        stateEnteredMs = millis();
    }
}

// ============================================================
// PRINT FINAL SUMMARY
// ============================================================
void printFinalResults() {
    Serial.println();
    Serial.println("# ==========================================");
    Serial.println("# FINAL RESULTS");
    Serial.println("# ==========================================");

    float tdSum = 0, tdMin = 9999, tdMax = 0;
    float kSum  = 0;
    for (int i = 0; i < trialCount; i++) {
        Serial.print("# Trial "); Serial.print(i + 1);
        Serial.print(": Td = ");  Serial.print(tdTrials[i], 3);
        Serial.print(" s  | K_MODEL = "); Serial.print(kTrials[i], 8);
        Serial.println(" mm/PWM/s");
        tdSum += tdTrials[i];
        kSum  += kTrials[i];
        if (tdTrials[i] < tdMin) tdMin = tdTrials[i];
        if (tdTrials[i] > tdMax) tdMax = tdTrials[i];
    }

    float tdMean = tdSum / trialCount;
    float kMean  = kSum  / trialCount;

    float tdVar = 0;
    for (int i = 0; i < trialCount; i++) {
        tdVar += (tdTrials[i] - tdMean) * (tdTrials[i] - tdMean);
    }
    float tdSigma = sqrtf(tdVar / trialCount);
    float tdConservative = tdMean + 2.0f * tdSigma;

    Serial.println("# ------------------------------------------");
    Serial.print("# Td mean         = "); Serial.print(tdMean,         3); Serial.println(" s");
    Serial.print("# Td sigma        = "); Serial.print(tdSigma,        3); Serial.println(" s");
    Serial.print("# Td min / max    = "); Serial.print(tdMin, 2); Serial.print(" / "); Serial.print(tdMax, 2); Serial.println(" s");
    Serial.print("# Td conservative = "); Serial.print(tdConservative, 3); Serial.println(" s  (mean + 2sigma, use in Smith Predictor)");
    Serial.print("# K_MODEL mean    = "); Serial.print(kMean,          8); Serial.println(" mm/PWM/s");
    Serial.print("# Sensor noise σ  = "); Serial.print(blSigma * 1000, 2); Serial.println(" um");
    Serial.print("# Test PWM        = "); Serial.println(TEST_PWM);
    Serial.println("# ------------------------------------------");
    Serial.println("# Copy these values into Mode 3:");
    Serial.print("#   const float Td_s    = "); Serial.print(tdConservative, 2); Serial.println(";");
    Serial.print("#   const float K_MODEL = "); Serial.print(kMean,          8); Serial.println(";");
    Serial.print("#   const int DELAY_SAMPLES = "); Serial.print((int)roundf(tdConservative / 0.1f)); Serial.println(";  // at 10 Hz");
    Serial.print("#   // PI deadband = "); Serial.print(blSigma * 3 * 1000, 1); Serial.println(" um  (3 x noise sigma)");
    Serial.println("# ==========================================");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Td:");
    lcd.print(tdMean, 2);
    lcd.print("s sig:");
    lcd.print(tdSigma, 2);
    lcd.setCursor(0, 1);
    lcd.print("K:");
    lcd.print(kMean, 6);
    lcd.print("  Done");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(300);

    // Outputs
    pinMode(PIN_RELAY1, OUTPUT);
    pinMode(PIN_RELAY2, OUTPUT);
    pinMode(PIN_REDE,   OUTPUT);
    digitalWrite(PIN_RELAY1, RELAY_OFF);
    digitalWrite(PIN_RELAY2, RELAY_OFF);
    digitalWrite(PIN_REDE,   LOW);

    // Inputs
    pinMode(PIN_BTN1,  INPUT_PULLDOWN);
    pinMode(PIN_BTN3,  INPUT_PULLUP);
    pinMode(PIN_ESTOP, INPUT_PULLUP);

    // PWM
    ledcAttach(PIN_PWM, 5000, 8);
    ledcWrite(PIN_PWM, 0);

    // ADC and Serial2 (RS-485)
    analogReadResolution(12);
    Serial2.begin(9600, SERIAL_8N1);   // RX2=GPIO16, TX2=GPIO17

    // LCD
    Wire.begin(21, 22);
    lcd.init();
    lcd.backlight();

    // Header
    Serial.println("# ==========================================");
    Serial.println("# Td MEASUREMENT — Pool Fire Level Control");
    Serial.println("# ==========================================");
    Serial.print("# TEST_PWM            = "); Serial.println(TEST_PWM);
    Serial.print("# BASELINE_DURATION   = "); Serial.print(BASELINE_DURATION_MS / 1000); Serial.println(" s");
    Serial.print("# SIGMA_MULTIPLIER    = "); Serial.println(SIGMA_MULTIPLIER);
    Serial.print("# CONSEC_REQUIRED     = "); Serial.println(CONSEC_REQUIRED);
    Serial.print("# MAX_TRIALS          = "); Serial.println(MAX_TRIALS);
    Serial.print("# STEP_TIMEOUT        = "); Serial.print(STEP_TIMEOUT_MS / 1000); Serial.println(" s");
    Serial.println("# ==========================================");

    enterIdle();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    checkEStop();

    unsigned long nowMs = millis();

    // --- Button: BTN3 = abort/reset (any state) ---
    if (btn3Pressed()) {
        pumpOff();
        trialCount = 0;
        Serial.println("# RESET by BTN3");
        enterIdle();
        return;
    }

    // --- Button: BTN1 = start / next trial ---
    if (btn1Pressed()) {
        if (state == STATE_IDLE || state == STATE_DETECTED) {
            enterBaseline();
            return;
        }
    }

    // --- Settling state: time-driven, no sensor needed ---
    if (state == STATE_SETTLING) {
        updateSettling(nowMs);
        return;
    }

    // --- Complete / Idle state: nothing to do ---
    if (state == STATE_IDLE || state == STATE_COMPLETE || state == STATE_DETECTED) {
        return;
    }

    // --- Sensor-driven states at 10 Hz ---
    if (nowMs - lastSampleMs < SAMPLE_INTERVAL_MS) return;
    lastSampleMs = nowMs;

    float dist = 0;
    if (!readDistance(dist)) {
        Serial.println("# WARN: invalid sensor reading — check RS-485 wiring");
        lcd.setCursor(0, 0);
        lcd.print("SENSOR ERR      ");
        return;
    }

    switch (state) {
        case STATE_BASELINE: updateBaseline(dist, nowMs); break;
        case STATE_STEP:     updateStep(dist, nowMs);     break;
        default: break;
    }
}
