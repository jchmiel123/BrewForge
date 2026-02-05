/**
 * BrewForge - Espresso Controller
 * Pico 2W (RP2350)
 *
 * Controls:
 * - 4 Relays (pump, boiler, solenoid valve, cup warmer)
 * - Flow sensor (pulse counting)
 * - Thermistor for boiler temp
 * - Heater element (via relay)
 * - External brew button (GP7)
 *
 * Pinout:
 *   GP2:     Solenoid valve relay (water gate from RO)
 *   GP3:     Cup warmer relay (unused for now)
 *   GP4:     Water pump relay
 *   GP5:     Boiler heater relay
 *   GP6:     Flow sensor input (pulses)
 *   GP7:     External brew button (active LOW, internal pullup)
 *   GP26:    Thermistor ADC input (ADC0)
 *   GP27:    Optional second temp sensor (ADC1)
 *   GP28:    Optional pressure sensor (ADC2)
 *
 * Forked from PicoTest (2026-02-05)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LEAmDNS.h>
#include <EEPROM.h>
// OTA removed - Updater.h doesn't work on RP2350 (Error 4). See ForgeRepo/CAPABILITIES.md

// EEPROM layout for saving settings
#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0xC0FFEE43  // New magic for BrewForge (was 0xC0FFEE42 in PicoTest)
struct SavedSettings {
    uint32_t magic;
    uint8_t stepTimes[9];       // Step times in seconds (0-255) - 9 steps now
    float targetTemp;           // Target temperature
    bool useFahrenheit;         // Temp unit preference
    bool softPumpMode;          // Soft pump setting
    bool flowSensorEnabled;     // Flow sensor bypass
    uint16_t skipPrimeThresholdSec; // Seconds threshold to skip pressurize
};

// ============ WIFI CONFIG ============
// Multiple networks - will try each in order
struct WiFiNetwork {
    const char* ssid;
    const char* pass;
};

const WiFiNetwork WIFI_NETWORKS[] = {
    {"Founders3-Office", "Gu1fR3serVe13"},
    {"DropitlikeitsHotspot", "Nutmeg21"}
};
const int NUM_NETWORKS = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

// Fallback AP mode settings
const char* AP_SSID = "BrewForge";
const char* AP_PASS = "coffee123";  // Min 8 chars
bool apMode = false;

WebServer server(80);

// ============ PIN DEFINITIONS ============
// Relays - matched to Justin's wiring
// Relay 1 (GP2) = Solenoid (RO water valve)
// Relay 2 (GP3) = Cup warmer (unused for now)
// Relay 3 (GP4) = Pump
// Relay 4 (GP5) = Boiler
const int RELAY_PUMP = 4;        // Water pump (relay 3)
const int RELAY_BOILER = 5;      // Boiler heater element (relay 4)
const int RELAY_SOLENOID = 2;    // Solenoid valve - RO water gate
const int RELAY_WARMER = 3;      // Cup warmer - unused for now

// External brew button
const int BREW_BUTTON = 7;       // Active LOW, internal pullup, wire to GND

// Flow sensor (Hall effect, outputs pulses)
const int FLOW_SENSOR = 6;

// Temperature sensing
const int TEMP_BOILER = 26;      // ADC0 - Thermistor on boiler
const int TEMP_GROUP = 27;       // ADC1 - Optional group head temp

// Onboard LED
// Pico 2W: LED is on WiFi chip, use LED_BUILTIN (maps to CYW43 GPIO)
const int LED_PIN = LED_BUILTIN;

// ============ RELAY CONFIG ============
// Set to true if your relay board is active LOW (most are)
// Your board is ACTIVE HIGH - HIGH turns relay ON
const bool RELAY_ACTIVE_LOW = false;

#define RELAY_ON  (RELAY_ACTIVE_LOW ? LOW : HIGH)
#define RELAY_OFF (RELAY_ACTIVE_LOW ? HIGH : LOW)

// ============ FLOW SENSOR ============
// Typical: YF-S201 = 7.5 pulses per mL (450 pulses/L)
const float PULSES_PER_ML = 7.5;

volatile unsigned long flowPulseCount = 0;
unsigned long lastFlowCheck = 0;
float flowRate = 0;        // mL/sec
float totalVolume = 0;     // mL
bool flowSensorEnabled = true;  // Can be disabled via web UI

// ============ TEMPERATURE ============
// For 10K NTC thermistor with 10K pullup to 3.3V
// Steinhart-Hart coefficients (generic 10K NTC)
const float THERM_A = 0.001129148;
const float THERM_B = 0.000234125;
const float THERM_C = 0.0000000876741;
const float THERM_NOMINAL = 10000;  // 10K at 25C
const float TEMP_NOMINAL = 25;
const float THERM_BCOEFF = 3950;    // Beta coefficient
const float SERIES_RESISTOR = 10000; // 10K pullup

float boilerTemp = 0;
float targetTemp = 93.0;   // Default brew temp (espresso sweet spot)
bool heaterOn = false;
float tempHysteresis = 2.0; // +/- degrees
bool useFahrenheit = false; // Display in F if true

// Temperature rate-of-change tracking
#define TEMP_HISTORY_SIZE 10
float tempHistory[TEMP_HISTORY_SIZE];
unsigned long tempHistoryTimes[TEMP_HISTORY_SIZE];
int tempHistoryIndex = 0;
int tempHistoryCount = 0;
float tempRatePerSec = 0.0;       // Degrees C per second
float estimatedTimeToTarget = -1.0; // Seconds, -1 = unknown

// Info page password
const char* INFO_PASSWORD = "Coffee4Me!";

// ============ BREW SETTINGS ============
const unsigned long BREW_TIME_MS = 25000;  // 25 seconds (standard espresso)
const unsigned long PREHEAT_TIMEOUT_MS = 60000;  // Max 60s to heat up
const float MIN_BREW_TEMP = 85.0;  // Minimum temp to start brewing
unsigned long brewTimeMs = BREW_TIME_MS;  // Adjustable via web UI

// TEST MODE: Skip temp check, use timer instead
bool testMode = true;  // Set to false for production with real thermistor
const unsigned long TEST_PREHEAT_MS = 15000;  // 15 seconds for testing

// SOFT PUMP MODE: Pulse pump to prevent hose kinking
bool softPumpMode = false;
const unsigned long PUMP_ON_MS = 3000;  // Pump on 3 seconds
const unsigned long PUMP_OFF_MS = 1000; // Pump off 1 second (hose recovery)
unsigned long lastPumpToggle = 0;
bool pumpPulseState = false;  // Current state in pulse cycle

// ============ STATE ============
bool pumpOn = false;
bool solenoidOn = false;
bool warmerOn = false;
bool brewingActive = false;
bool preheating = false;
unsigned long brewStartTime = 0;
unsigned long preheatStartTime = 0;

// Time-since-last-brew tracking
unsigned long lastBrewCompleteTime = 0;
bool hasBrewedBefore = false;
unsigned int skipPrimeThresholdSec = 120;  // 2 minutes default

// Effective step time override for adaptive preheat (doesn't mutate saved settings)
int effectivePreheatTime = -1;  // -1 = use stepTimes[], >0 = override

// State machine for brew cycle - 9-step sequence
enum BrewState {
    IDLE,           // 0: Everything off, waiting for button
    TEMP_CHECK,     // 1: Read initial temp, display it
    PRESSURIZE,     // 2: Quick prime burst - solenoid+pump (pressurize from RO)
    PREHEAT,        // 3: Heater ON, wait for temp or timeout
    PRIME,          // 4: Pump ON ~4 sec to pop pod, heater ON
    PAUSE,          // 5: Pump OFF, heater ON, read temp
    BREW,           // 6: Pump ON + Heater ON, main extraction
    COOLDOWN,       // 7: Pump ON, Heater OFF (gradient cool)
    DONE            // 8: Everything off, show results
};
BrewState brewState = IDLE;

// Step timings - ADJUSTABLE (in seconds for easy UI)
unsigned int stepTimes[] = {
    0,    // 0: IDLE - not used
    1,    // 1: TEMP_CHECK - quick
    2,    // 2: PRESSURIZE - quick prime burst from RO
    15,   // 3: PREHEAT - 15 sec heater only
    4,    // 4: PRIME/POP - pump+heater to pop pod
    1,    // 5: PAUSE - brief
    15,   // 6: BREW - main extraction pump+heater
    3,    // 7: COOLDOWN - pump only, no heat
    0     // 8: DONE - auto back to idle
};

unsigned long stepStartTime = 0;  // When current step started

// Step names for display
const char* stepNames[] = {"IDLE", "TEMP_CHECK", "PRESSURIZE", "PREHEAT", "PRIME", "PAUSE", "BREW", "COOLDOWN", "DONE"};

// ============ LEARN BREW (Recording Mode) ============
// Records relay actions with timestamps for perfect playback
#define MAX_RECORDED_ACTIONS 50
#define SIMULTANEOUS_THRESHOLD_MS 100  // Actions within 100ms = simultaneous

struct RecordedAction {
    unsigned long timeMs;    // Time since recording started
    uint8_t relay;           // Which relay (0=pump, 1=boiler, 2=solenoid, 3=warmer)
    bool state;              // ON or OFF
};

RecordedAction recordedActions[MAX_RECORDED_ACTIONS];
int recordedCount = 0;
bool isRecording = false;
bool isPlaying = false;
unsigned long recordStartTime = 0;
unsigned long playStartTime = 0;
int playIndex = 0;

// ============ FORWARD DECLARATIONS ============
void flowSensorISR();
void updateFlowRate();
float readTemperature(int pin);
void updateHeaterControl();
void updateTempRate();
void setRelay(int pin, bool on, const char* name);
void toggleRelayDirect(int relayNum);
void allRelaysOff();
void printStatus();
void printHelp();
void handleCommand(char cmd);
void startBrewCycle();
void updateBrewCycle();
void stopBrew();
void abortBrew(const char* reason);
void nextStep();
void prevStep();
void goToStep(BrewState step);
void setupWiFi();
void updateWiFi();
void setupWebServer();
void saveSettings();
void loadSettings();
void startRecording();
void stopRecording();
void recordAction(int relay, bool state);
void startPlayback();
void updatePlayback();
void saveRecording();
void loadRecording();
unsigned long timeSinceLastBrewSec();
bool shouldSkipPressurize();
String getStatusJson();
String getWebPage();
float toFahrenheit(float c);

// WiFi state (defined here for forward reference)
extern int wifiNetworkIndex;
extern unsigned long wifiConnectStart;
extern bool wifiConnecting;
extern bool wifiSetupDone;
extern int wifiRetryCount;
extern unsigned int pollInterval;
extern bool apMode;

// ============ SETUP ============
void setup() {
    // CRITICAL: Set relay pins to OFF IMMEDIATELY before anything else
    digitalWrite(RELAY_PUMP, RELAY_OFF);
    digitalWrite(RELAY_BOILER, RELAY_OFF);
    digitalWrite(RELAY_SOLENOID, RELAY_OFF);
    digitalWrite(RELAY_WARMER, RELAY_OFF);
    pinMode(RELAY_PUMP, OUTPUT);
    pinMode(RELAY_BOILER, OUTPUT);
    pinMode(RELAY_SOLENOID, OUTPUT);
    pinMode(RELAY_WARMER, OUTPUT);

    Serial.begin(115200);
    delay(2000);

    // Initialize EEPROM and load saved settings
    EEPROM.begin(EEPROM_SIZE);
    loadSettings();
    loadRecording();

    // Flow sensor with pullup, interrupt on rising edge
    pinMode(FLOW_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR), flowSensorISR, RISING);

    // External brew button with internal pullup (button wires to GND)
    pinMode(BREW_BUTTON, INPUT_PULLUP);

    // ADC setup
    analogReadResolution(12);  // 12-bit ADC

    // LED
    pinMode(LED_PIN, OUTPUT);

    // Initialize temp history
    for (int i = 0; i < TEMP_HISTORY_SIZE; i++) {
        tempHistory[i] = 0;
        tempHistoryTimes[i] = 0;
    }

    Serial.println("================================");
    Serial.println("BrewForge - Espresso Controller");
    Serial.println("Pico 2W (RP2350)");
    Serial.println("================================");
    Serial.print("Relay mode: ");
    Serial.println(RELAY_ACTIVE_LOW ? "ACTIVE LOW" : "ACTIVE HIGH");
    Serial.print("Target temp: ");
    Serial.print(targetTemp);
    Serial.println(" C");
    Serial.print("Test mode: ");
    Serial.println(testMode ? "ON (15s preheat)" : "OFF (real temp)");
    Serial.print("Flow sensor: ");
    Serial.println(flowSensorEnabled ? "ENABLED" : "DISABLED");
    Serial.println();

    // Setup WiFi (non-blocking - connects in background)
    setupWiFi();

    printHelp();
}

// ============ MAIN LOOP ============
void loop() {
    static unsigned long lastTempRead = 0;
    static unsigned long lastStatusPrint = 0;
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    static bool lastButtonState = false;

    // External button debounce state
    static unsigned long lastExtButtonChange = 0;
    static bool extButtonDebounced = false;
    static bool lastExtButtonRaw = false;

    unsigned long now = millis();

    // Non-blocking WiFi connection
    updateWiFi();

    // Read BOOTSEL button (built into Pico)
    bool bootselPressed = BOOTSEL;

    // Read external brew button (active LOW - LOW when pressed)
    bool extButtonRaw = !digitalRead(BREW_BUTTON);

    // Debounce external button (50ms)
    if (extButtonRaw != lastExtButtonRaw) {
        lastExtButtonChange = now;
    }
    lastExtButtonRaw = extButtonRaw;
    if ((now - lastExtButtonChange) >= 50) {
        extButtonDebounced = extButtonRaw;
    }

    // Combine both buttons - either can trigger
    bool buttonPressed = bootselPressed || extButtonDebounced;

    // Button press detection (rising edge - just pressed)
    if (buttonPressed && !lastButtonState) {
        Serial.println(bootselPressed ? "BOOTSEL pressed!" : "Brew button pressed!");
        if (brewState == IDLE) {
            startBrewCycle();
        } else if (brewState == DONE) {
            goToStep(IDLE);
        } else {
            stopBrew();
        }
    }
    lastButtonState = buttonPressed;

    // LED pattern: blink N times for step N, then pause
    static int blinkCount = 0;
    static bool inPause = false;

    if (brewState == IDLE) {
        // Slow heartbeat when idle
        if (now - lastBlink >= 1000) {
            lastBlink = now;
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState);
        }
    } else if (brewState == DONE) {
        // Fast blink when done
        if (now - lastBlink >= 100) {
            lastBlink = now;
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState);
        }
    } else {
        // Blink N times for step N (1-8)
        int stepNum = (int)brewState;
        if (inPause) {
            if (now - lastBlink >= 800) {
                inPause = false;
                blinkCount = 0;
                lastBlink = now;
            }
        } else {
            if (now - lastBlink >= 150) {
                lastBlink = now;
                ledState = !ledState;
                digitalWrite(LED_PIN, ledState);
                if (!ledState) {
                    blinkCount++;
                    if (blinkCount >= stepNum) {
                        inPause = true;
                        blinkCount = 0;
                    }
                }
            }
        }
    }

    // Read temperature every 250ms
    if (now - lastTempRead >= 250) {
        lastTempRead = now;
        boilerTemp = readTemperature(TEMP_BOILER);

        // Record in circular buffer for rate-of-change
        tempHistory[tempHistoryIndex] = boilerTemp;
        tempHistoryTimes[tempHistoryIndex] = now;
        tempHistoryIndex = (tempHistoryIndex + 1) % TEMP_HISTORY_SIZE;
        if (tempHistoryCount < TEMP_HISTORY_SIZE) tempHistoryCount++;

        // Calculate rate of change
        updateTempRate();

        // Heater control runs during steps that have heater on
        if (brewState == PREHEAT || brewState == PRIME || brewState == PAUSE || brewState == BREW) {
            updateHeaterControl();
        }
    }

    // Update flow rate every 500ms
    if (now - lastFlowCheck >= 500) {
        updateFlowRate();
        lastFlowCheck = now;
    }

    // Run the brew state machine
    updateBrewCycle();

    // Run playback if active
    updatePlayback();

    // Auto status print every 2 seconds if not idle
    static unsigned long lastIdleStatus = 0;
    if (brewState != IDLE && (now - lastStatusPrint >= 2000)) {
        lastStatusPrint = now;

        Serial.print("[");
        Serial.print((int)brewState);
        Serial.print(":");
        Serial.print(stepNames[brewState]);

        if (brewState >= PRIME && brewState <= COOLDOWN) {
            unsigned long stepTime = (now - stepStartTime) / 1000;
            Serial.print(" ");
            Serial.print(stepTime);
            Serial.print("s");
        }

        Serial.print("] Temp: ");
        Serial.print(boilerTemp, 1);
        Serial.print("C");

        if (tempRatePerSec != 0.0) {
            Serial.print(" (");
            if (tempRatePerSec > 0) Serial.print("+");
            Serial.print(tempRatePerSec, 1);
            Serial.print("C/s)");
        }

        if (brewState >= PRIME && brewState <= COOLDOWN) {
            Serial.print(", Flow: ");
            Serial.print(flowRate, 1);
            Serial.print(" mL/s, Vol: ");
            Serial.print(totalVolume, 1);
            Serial.print(" mL");
        }
        Serial.println();
    }

    // Print IP and status every 30 seconds when idle
    if (brewState == IDLE && (now - lastIdleStatus >= 30000)) {
        lastIdleStatus = now;
        Serial.print("[IDLE] Temp: ");
        Serial.print(boilerTemp, 1);
        Serial.print("C | ");
        if (hasBrewedBefore) {
            Serial.print("Last brew: ");
            unsigned long sec = timeSinceLastBrewSec();
            if (sec < 60) { Serial.print(sec); Serial.print("s"); }
            else if (sec < 3600) { Serial.print(sec / 60); Serial.print("m"); }
            else { Serial.print(sec / 3600); Serial.print("h"); }
            Serial.print(" ago | ");
        }
        if (apMode) {
            Serial.print("AP: ");
            Serial.print(AP_SSID);
            Serial.print(" | http://");
            Serial.println(WiFi.softAPIP());
        } else if (WiFi.status() == WL_CONNECTED) {
            Serial.print("http://");
            Serial.print(WiFi.localIP());
            Serial.print(" | brewforge.local | ");
            Serial.print(WiFi.RSSI());
            Serial.println("dBm");
        } else {
            Serial.println("WiFi disconnected");
        }
    }

    // Serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }

    // Handle web requests (WiFi STA or AP mode)
    if (wifiSetupDone && (apMode || WiFi.status() == WL_CONNECTED)) {
        server.handleClient();
        if (!apMode) MDNS.update();
    }
}

// ============ FLOW SENSOR ISR ============
void flowSensorISR() {
    flowPulseCount++;
}

void updateFlowRate() {
    static unsigned long lastPulseCount = 0;
    static unsigned long lastTime = 0;

    unsigned long now = millis();
    unsigned long pulses = flowPulseCount - lastPulseCount;
    unsigned long dt = now - lastTime;

    if (dt > 0) {
        float ml = pulses / PULSES_PER_ML;
        flowRate = ml * 1000.0 / dt;  // mL/sec
        totalVolume += ml;
    }

    lastPulseCount = flowPulseCount;
    lastTime = now;
}

// ============ TEMPERATURE READING ============
float readTemperature(int pin) {
    int raw = analogRead(pin);

    // Convert to resistance
    float voltage = raw * 3.3 / 4095.0;
    float resistance = SERIES_RESISTOR * voltage / (3.3 - voltage);

    // Steinhart-Hart equation
    float steinhart;
    steinhart = resistance / THERM_NOMINAL;
    steinhart = log(steinhart);
    steinhart /= THERM_BCOEFF;
    steinhart += 1.0 / (TEMP_NOMINAL + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;

    return steinhart;
}

// ============ TEMPERATURE RATE OF CHANGE ============
void updateTempRate() {
    if (tempHistoryCount < 3) {
        tempRatePerSec = 0.0;
        estimatedTimeToTarget = -1.0;
        return;
    }

    // Use oldest and newest readings in the circular buffer
    int oldest = (tempHistoryCount < TEMP_HISTORY_SIZE) ? 0 : tempHistoryIndex;
    int newest = (tempHistoryIndex - 1 + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;

    float dTemp = tempHistory[newest] - tempHistory[oldest];
    float dTime = (tempHistoryTimes[newest] - tempHistoryTimes[oldest]) / 1000.0;

    if (dTime > 0.1) {
        tempRatePerSec = dTemp / dTime;
    }

    // Estimate time to target (only useful when heating)
    if (tempRatePerSec > 0.1 && boilerTemp < targetTemp) {
        estimatedTimeToTarget = (targetTemp - boilerTemp) / tempRatePerSec;
    } else {
        estimatedTimeToTarget = -1.0;
    }
}

// ============ HEATER CONTROL ============
void updateHeaterControl() {
    // Bang-bang control with hysteresis + predictive cutoff

    // Predictive cutoff: if heating fast and close to target, cut early
    if (heaterOn && tempRatePerSec > 2.0) {
        float predictedTemp = boilerTemp + tempRatePerSec * 2.0;  // Where we'll be in 2 seconds
        if (predictedTemp > targetTemp + tempHysteresis) {
            setRelay(RELAY_BOILER, false, "Boiler (predictive cutoff)");
            heaterOn = false;
            return;
        }
    }

    if (boilerTemp < (targetTemp - tempHysteresis)) {
        if (!heaterOn) {
            setRelay(RELAY_BOILER, true, "Boiler");
            heaterOn = true;
        }
    } else if (boilerTemp > (targetTemp + tempHysteresis)) {
        if (heaterOn) {
            setRelay(RELAY_BOILER, false, "Boiler");
            heaterOn = false;
        }
    }
}

// ============ RELAY CONTROL ============
void setRelay(int pin, bool on, const char* name) {
    digitalWrite(pin, on ? RELAY_ON : RELAY_OFF);
    Serial.print(name);
    Serial.print(": ");
    Serial.println(on ? "ON" : "OFF");
}

void toggleRelayDirect(int relayNum) {
    const int pins[] = {0, RELAY_PUMP, RELAY_BOILER, RELAY_SOLENOID, RELAY_WARMER};
    const char* names[] = {"", "Pump", "Boiler", "Solenoid", "Warmer"};
    bool* states[] = {nullptr, &pumpOn, &heaterOn, &solenoidOn, &warmerOn};

    if (relayNum < 1 || relayNum > 4) return;

    *states[relayNum] = !(*states[relayNum]);
    digitalWrite(pins[relayNum], *states[relayNum] ? RELAY_ON : RELAY_OFF);
    Serial.print("Relay ");
    Serial.print(relayNum);
    Serial.print(" (");
    Serial.print(names[relayNum]);
    Serial.print("): ");
    Serial.println(*states[relayNum] ? "ON" : "OFF");
}

// ============ EXPLICIT RELAY STATE CONTROL ============
void setAllRelays(bool pump, bool boiler, bool solenoid, bool warmer, const char* context) {
    Serial.print("[");
    Serial.print(context);
    Serial.print("] Relays: P=");
    Serial.print(pump ? "ON" : "off");
    Serial.print(" B=");
    Serial.print(boiler ? "ON" : "off");
    Serial.print(" S=");
    Serial.print(solenoid ? "ON" : "off");
    Serial.print(" W=");
    Serial.println(warmer ? "ON" : "off");

    digitalWrite(RELAY_PUMP, pump ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_BOILER, boiler ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_SOLENOID, solenoid ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_WARMER, warmer ? RELAY_ON : RELAY_OFF);

    pumpOn = pump;
    heaterOn = boiler;
    solenoidOn = solenoid;
    warmerOn = warmer;
}

void allRelaysOff() {
    setAllRelays(false, false, false, false, "ALL OFF");
    brewingActive = false;
}

// ============ TIME-SINCE-LAST-BREW ============
unsigned long timeSinceLastBrewSec() {
    if (!hasBrewedBefore) return 999999;
    return (millis() - lastBrewCompleteTime) / 1000;
}

bool shouldSkipPressurize() {
    return timeSinceLastBrewSec() < skipPrimeThresholdSec;
}

// ============ BREW CYCLE STATE MACHINE ============

void startBrewCycle() {
    if (brewState != IDLE) {
        Serial.println("Already running - use STOP first");
        return;
    }

    Serial.println("=== STARTING BREW SEQUENCE ===");
    Serial.print("Target temp: ");
    Serial.print(useFahrenheit ? toFahrenheit(targetTemp) : targetTemp, 1);
    Serial.println(useFahrenheit ? "F" : "C");

    if (hasBrewedBefore) {
        unsigned long sec = timeSinceLastBrewSec();
        Serial.print("Time since last brew: ");
        if (sec < 60) { Serial.print(sec); Serial.println("s"); }
        else { Serial.print(sec / 60); Serial.println("m"); }
    }

    totalVolume = 0;
    flowPulseCount = 0;
    brewingActive = true;
    effectivePreheatTime = -1;

    goToStep(TEMP_CHECK);
}

void goToStep(BrewState step) {
    brewState = step;
    stepStartTime = millis();

    Serial.print(">>> STEP ");
    Serial.print((int)step);
    Serial.print(": ");
    Serial.println(stepNames[step]);

    // Set relays based on step
    // RULE: Solenoid mirrors pump (it's the RO water gate)
    switch (step) {
        case IDLE:
            setAllRelays(false, false, false, false, stepNames[step]);
            brewingActive = false;
            effectivePreheatTime = -1;
            break;
        case DONE:
            setAllRelays(false, false, false, false, stepNames[step]);
            brewingActive = false;
            lastBrewCompleteTime = millis();
            hasBrewedBefore = true;
            Serial.println("=== BREW COMPLETE ===");
            Serial.print("Volume: ");
            Serial.print(totalVolume, 1);
            Serial.println(" mL");
            Serial.print("Final temp: ");
            Serial.print(useFahrenheit ? toFahrenheit(boilerTemp) : boilerTemp, 1);
            Serial.println(useFahrenheit ? "F" : "C");
            break;
        case TEMP_CHECK:
            setAllRelays(false, false, false, false, stepNames[step]);
            Serial.print("Current temp: ");
            Serial.print(useFahrenheit ? toFahrenheit(boilerTemp) : boilerTemp, 1);
            Serial.println(useFahrenheit ? "F" : "C");
            break;
        case PRESSURIZE:
            // Quick burst to prime lines from RO system
            setAllRelays(true, false, true, false, stepNames[step]);
            Serial.println("Pressurizing lines from RO...");
            break;
        case PREHEAT: {
            setAllRelays(false, true, false, false, stepNames[step]);  // Heater ON only
            // Adaptive preheat: reduce time if recent brew
            unsigned long sinceLast = timeSinceLastBrewSec();
            if (hasBrewedBefore && sinceLast < 300) {
                float reduction = 1.0 - ((float)sinceLast / 300.0);
                int reducedTime = (int)(stepTimes[PREHEAT] * (1.0 - reduction * 0.7));
                if (reducedTime < 3) reducedTime = 3;
                effectivePreheatTime = reducedTime;
                Serial.print("Adaptive preheat: ");
                Serial.print(reducedTime);
                Serial.print("s (was ");
                Serial.print(stepTimes[PREHEAT]);
                Serial.print("s, last brew ");
                Serial.print(sinceLast);
                Serial.println("s ago)");
            } else {
                effectivePreheatTime = -1;
            }
            break;
        }
        case PRIME:
            // Pump + Heater + Solenoid ON (pop the pod)
            setAllRelays(true, true, true, false, stepNames[step]);
            lastPumpToggle = millis();
            pumpPulseState = true;
            break;
        case PAUSE:
            // Heater ON, Pump+Solenoid OFF
            setAllRelays(false, true, false, false, stepNames[step]);
            Serial.print("Temp after prime: ");
            Serial.print(useFahrenheit ? toFahrenheit(boilerTemp) : boilerTemp, 1);
            Serial.println(useFahrenheit ? "F" : "C");
            break;
        case BREW:
            // Pump + Heater + Solenoid ON (main extraction)
            setAllRelays(true, true, true, false, stepNames[step]);
            brewStartTime = millis();
            lastPumpToggle = millis();
            pumpPulseState = true;
            break;
        case COOLDOWN:
            // Pump + Solenoid ON, Heater OFF (gradient cool)
            setAllRelays(true, false, true, false, stepNames[step]);
            break;
    }
}

void nextStep() {
    if (brewState < DONE) {
        goToStep((BrewState)(brewState + 1));
    } else {
        goToStep(IDLE);
    }
}

void prevStep() {
    if (brewState > IDLE) {
        goToStep((BrewState)(brewState - 1));
    }
}

void updateBrewCycle() {
    unsigned long now = millis();
    unsigned long stepElapsed = now - stepStartTime;
    unsigned long stepTimeMs = stepTimes[brewState] * 1000UL;

    // Use effective preheat time if set
    if (brewState == PREHEAT && effectivePreheatTime > 0) {
        stepTimeMs = effectivePreheatTime * 1000UL;
    }

    switch (brewState) {
        case IDLE:
            break;

        case TEMP_CHECK:
            if (stepElapsed >= stepTimeMs) {
                nextStep();
            }
            break;

        case PRESSURIZE:
            if (shouldSkipPressurize()) {
                Serial.println("Recent brew - skipping pressurize (system still primed)");
                nextStep();
            } else if (stepElapsed >= stepTimeMs) {
                nextStep();
            }
            break;

        case PREHEAT:
            updateHeaterControl();
            if (boilerTemp >= targetTemp || stepElapsed >= stepTimeMs) {
                Serial.print("Preheat done. Temp: ");
                Serial.print(useFahrenheit ? toFahrenheit(boilerTemp) : boilerTemp, 1);
                Serial.println(useFahrenheit ? "F" : "C");
                nextStep();
            }
            break;

        case PRIME:
            if (softPumpMode) {
                unsigned long pumpElapsed = now - lastPumpToggle;
                if (pumpPulseState && pumpElapsed >= PUMP_ON_MS) {
                    digitalWrite(RELAY_PUMP, RELAY_OFF);
                    digitalWrite(RELAY_SOLENOID, RELAY_OFF);  // Mirror pump
                    pumpPulseState = false;
                    lastPumpToggle = now;
                } else if (!pumpPulseState && pumpElapsed >= PUMP_OFF_MS) {
                    digitalWrite(RELAY_PUMP, RELAY_ON);
                    digitalWrite(RELAY_SOLENOID, RELAY_ON);   // Mirror pump
                    pumpPulseState = true;
                    lastPumpToggle = now;
                }
            }
            updateHeaterControl();
            if (stepElapsed >= stepTimeMs) {
                nextStep();
            }
            break;

        case PAUSE:
            updateHeaterControl();
            if (stepElapsed >= stepTimeMs) {
                Serial.print("Temp after pause: ");
                Serial.print(useFahrenheit ? toFahrenheit(boilerTemp) : boilerTemp, 1);
                Serial.println(useFahrenheit ? "F" : "C");
                nextStep();
            }
            break;

        case BREW:
            if (softPumpMode) {
                unsigned long pumpElapsed = now - lastPumpToggle;
                if (pumpPulseState && pumpElapsed >= PUMP_ON_MS) {
                    digitalWrite(RELAY_PUMP, RELAY_OFF);
                    digitalWrite(RELAY_SOLENOID, RELAY_OFF);  // Mirror pump
                    pumpPulseState = false;
                    lastPumpToggle = now;
                } else if (!pumpPulseState && pumpElapsed >= PUMP_OFF_MS) {
                    digitalWrite(RELAY_PUMP, RELAY_ON);
                    digitalWrite(RELAY_SOLENOID, RELAY_ON);   // Mirror pump
                    pumpPulseState = true;
                    lastPumpToggle = now;
                }
            }
            updateHeaterControl();
            if (stepElapsed >= stepTimeMs) {
                nextStep();
            }
            break;

        case COOLDOWN:
            if (stepElapsed >= stepTimeMs) {
                nextStep();
            }
            break;

        case DONE:
            if (stepElapsed >= 2000) {
                goToStep(IDLE);
            }
            break;
    }
}

void stopBrew() {
    Serial.println("!!! STOP - All OFF !!!");
    setAllRelays(false, false, false, false, "STOP");
    brewState = IDLE;
    brewingActive = false;
    preheating = false;
    effectivePreheatTime = -1;
}

void abortBrew(const char* reason) {
    Serial.print("!!! ABORT: ");
    Serial.println(reason);
    setAllRelays(false, false, false, false, "ABORT");
    brewState = IDLE;
    brewingActive = false;
    preheating = false;
    effectivePreheatTime = -1;
}

// ============ COMMAND HANDLING ============
void handleCommand(char cmd) {
    switch (cmd) {
        case 'h': case 'H': case '?':
            printHelp();
            break;

        case 's': case 'S':
            printStatus();
            break;

        case 'b': case 'B':
            if (brewState == IDLE) {
                startBrewCycle();
            } else {
                Serial.println("Already running - use X to stop, N for next, V for back");
            }
            break;

        case 'n': case 'N':
            nextStep();
            break;

        case 'v': case 'V':
            prevStep();
            break;

        case 'p': case 'P':
            pumpOn = !pumpOn;
            setRelay(RELAY_PUMP, pumpOn, "Pump");
            break;

        case 'o': case 'O':
            solenoidOn = !solenoidOn;
            setRelay(RELAY_SOLENOID, solenoidOn, "Solenoid");
            break;

        case 'w': case 'W':
            warmerOn = !warmerOn;
            setRelay(RELAY_WARMER, warmerOn, "Cup Warmer");
            break;

        case '+':
            targetTemp += 1;
            if (targetTemp > 100) targetTemp = 100;
            Serial.print("Target temp: ");
            Serial.print(targetTemp);
            Serial.println(" C");
            break;

        case '-':
            targetTemp -= 1;
            if (targetTemp < 50) targetTemp = 50;
            Serial.print("Target temp: ");
            Serial.print(targetTemp);
            Serial.println(" C");
            break;

        case 'r': case 'R':
            totalVolume = 0;
            flowPulseCount = 0;
            Serial.println("Flow counter reset");
            break;

        case 'x': case 'X':
            allRelaysOff();
            brewState = IDLE;
            effectivePreheatTime = -1;
            Serial.println("!!! EMERGENCY STOP - ALL OFF !!!");
            break;

        case 'c': case 'C':
            Serial.println("Retrying WiFi...");
            apMode = false;
            wifiSetupDone = false;
            wifiConnecting = false;
            wifiNetworkIndex = 0;
            wifiRetryCount = 0;
            WiFi.disconnect();
            WiFi.mode(WIFI_STA);
            break;

        case 't': case 'T':
            testMode = !testMode;
            Serial.print("Test mode: ");
            Serial.println(testMode ? "ON (15s preheat timer)" : "OFF (real temp sensing)");
            break;

        case 'k': case 'K':
            softPumpMode = !softPumpMode;
            Serial.print("Soft pump mode: ");
            Serial.println(softPumpMode ? "ON (pulsing)" : "OFF (continuous)");
            break;

        case 'f': case 'F':
            flowSensorEnabled = !flowSensorEnabled;
            Serial.print("Flow sensor: ");
            Serial.println(flowSensorEnabled ? "ENABLED" : "DISABLED (bypass)");
            break;

        case 'l': case 'L':
            if (hasBrewedBefore) {
                unsigned long sec = timeSinceLastBrewSec();
                Serial.print("Last brew: ");
                if (sec < 60) { Serial.print(sec); Serial.println("s ago"); }
                else if (sec < 3600) { Serial.print(sec / 60); Serial.println("m ago"); }
                else { Serial.print(sec / 3600); Serial.println("h ago"); }
            } else {
                Serial.println("No brews yet this session");
            }
            break;

        case '1': case '2': case '3': case '4':
            toggleRelayDirect(cmd - '0');
            break;

        case '\n': case '\r': case ' ':
            break;

        default:
            if (cmd >= 32 && cmd <= 126) {
                Serial.print("Unknown command: '");
                Serial.print(cmd);
                Serial.println("' - press H for help");
            }
            break;
    }
}

// ============ STATUS ============
void printStatus() {
    Serial.println("\n========== STATUS ==========");
    Serial.print("Boiler Temp:  ");
    Serial.print(boilerTemp, 1);
    Serial.print(" C (target: ");
    Serial.print(targetTemp, 1);
    Serial.println(" C)");

    Serial.print("Temp Rate:    ");
    if (tempRatePerSec > 0) Serial.print("+");
    Serial.print(tempRatePerSec, 2);
    Serial.println(" C/sec");

    if (estimatedTimeToTarget > 0) {
        Serial.print("ETA to target: ");
        Serial.print((int)estimatedTimeToTarget);
        Serial.println(" sec");
    }

    Serial.print("Flow Rate:    ");
    Serial.print(flowRate, 2);
    Serial.print(" mL/sec");
    if (!flowSensorEnabled) Serial.print(" [DISABLED]");
    Serial.println();

    Serial.print("Total Volume: ");
    Serial.print(totalVolume, 1);
    Serial.println(" mL");

    Serial.print("Flow Pulses:  ");
    Serial.println(flowPulseCount);

    Serial.println();
    Serial.print("Pump:     "); Serial.println(pumpOn ? "ON" : "OFF");
    Serial.print("Boiler:   "); Serial.println(heaterOn ? "ON" : "OFF");
    Serial.print("Solenoid: "); Serial.println(solenoidOn ? "ON" : "OFF");
    Serial.print("Warmer:   "); Serial.println(warmerOn ? "ON" : "OFF");

    Serial.print("State:    ["); Serial.print((int)brewState); Serial.print("] ");
    Serial.println(stepNames[brewState]);

    if (hasBrewedBefore) {
        unsigned long sec = timeSinceLastBrewSec();
        Serial.print("Last brew: ");
        if (sec < 60) { Serial.print(sec); Serial.println("s ago"); }
        else if (sec < 3600) { Serial.print(sec / 60); Serial.println("m ago"); }
        else { Serial.print(sec / 3600); Serial.println("h ago"); }
    }
    Serial.println("============================\n");
}

void printHelp() {
    Serial.println("BrewForge Commands:");
    Serial.println("  H     Help");
    Serial.println("  S     Status (temp, flow, relays, rate)");
    Serial.println("  B     Start Brew Cycle");
    Serial.println("  N     Next step");
    Serial.println("  V     Previous (back) step");
    Serial.println("  P     Toggle Pump");
    Serial.println("  O     Toggle Solenoid");
    Serial.println("  W     Toggle Cup Warmer");
    Serial.println("  +/-   Adjust target temp +/-1C");
    Serial.println("  R     Reset flow counter");
    Serial.println("  X     EMERGENCY STOP");
    Serial.println("  C     Retry WiFi connection");
    Serial.println("  T     Toggle test mode (skip temp check)");
    Serial.println("  K     Toggle soft pump mode (pulse)");
    Serial.println("  F     Toggle flow sensor bypass");
    Serial.println("  L     Show last brew time");
    Serial.println("  1-4   Direct relay toggle");
    Serial.println();
    Serial.println("Brew button (GP7) or BOOTSEL starts/stops brew");
    Serial.print("Web UI: http://");
    if (apMode) {
        Serial.println(WiFi.softAPIP());
    } else {
        Serial.print(WiFi.localIP());
        Serial.println(" | http://brewforge.local");
    }
    Serial.println();
}

// ============ WIFI & WEB SERVER ============

int wifiNetworkIndex = 0;
unsigned long wifiConnectStart = 0;
bool wifiConnecting = false;
bool wifiSetupDone = false;
int wifiRetryCount = 0;

void startAPMode() {
    Serial.println("Starting AP mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    apMode = true;
    wifiSetupDone = true;

    Serial.println("=================================");
    Serial.print("AP Mode: ");
    Serial.println(AP_SSID);
    Serial.print("Password: ");
    Serial.println(AP_PASS);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("=================================");

    setupWebServer();
}

void startWiFiConnect() {
    if (wifiNetworkIndex >= NUM_NETWORKS) {
        wifiNetworkIndex = 0;
        wifiRetryCount++;
        if (wifiRetryCount > 2) {
            Serial.println("WiFi failed - switching to AP mode");
            startAPMode();
            return;
        }
    }

    Serial.print("Trying WiFi: ");
    Serial.println(WIFI_NETWORKS[wifiNetworkIndex].ssid);
    WiFi.begin(WIFI_NETWORKS[wifiNetworkIndex].ssid, WIFI_NETWORKS[wifiNetworkIndex].pass);
    wifiConnectStart = millis();
    wifiConnecting = true;
}

void updateWiFi() {
    if (wifiSetupDone) return;

    if (!wifiConnecting) {
        if (millis() > 3000) {
            startWiFiConnect();
        }
        return;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("Network: ");
        Serial.println(WIFI_NETWORKS[wifiNetworkIndex].ssid);
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        wifiSetupDone = true;
        wifiConnecting = false;
        setupWebServer();
        return;
    }

    unsigned long timeout = 5000 + (wifiRetryCount * 3000);
    if (timeout > 15000) timeout = 15000;

    if (millis() - wifiConnectStart > timeout) {
        Serial.print(" timeout (");
        Serial.print(timeout / 1000);
        Serial.println("s)");
        WiFi.disconnect();
        wifiConnecting = false;
        wifiNetworkIndex++;
        delay(500);
        startWiFiConnect();
    }
}

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    Serial.println("WiFi will connect in background...");
}

float toFahrenheit(float c) {
    return c * 9.0 / 5.0 + 32.0;
}

void saveSettings() {
    SavedSettings settings;
    settings.magic = EEPROM_MAGIC;
    for (int i = 0; i < 9; i++) {
        settings.stepTimes[i] = (uint8_t)stepTimes[i];
    }
    settings.targetTemp = targetTemp;
    settings.useFahrenheit = useFahrenheit;
    settings.softPumpMode = softPumpMode;
    settings.flowSensorEnabled = flowSensorEnabled;
    settings.skipPrimeThresholdSec = skipPrimeThresholdSec;

    EEPROM.put(0, settings);
    bool ok = EEPROM.commit();

    Serial.println("=== SETTINGS SAVED ===");
    Serial.print("EEPROM commit: ");
    Serial.println(ok ? "OK" : "FAILED!");
    Serial.print("Times: ");
    for (int i = 1; i <= 7; i++) {
        Serial.print(stepTimes[i]);
        Serial.print(" ");
    }
    Serial.println();
    Serial.print("Target temp: ");
    Serial.println(targetTemp);
}

void loadSettings() {
    SavedSettings settings;
    EEPROM.get(0, settings);

    Serial.print("EEPROM magic: 0x");
    Serial.println(settings.magic, HEX);

    if (settings.magic == EEPROM_MAGIC) {
        Serial.println("=== LOADING SAVED SETTINGS ===");
        for (int i = 0; i < 9; i++) {
            stepTimes[i] = settings.stepTimes[i];
        }
        targetTemp = settings.targetTemp;
        useFahrenheit = settings.useFahrenheit;
        softPumpMode = settings.softPumpMode;
        flowSensorEnabled = settings.flowSensorEnabled;
        skipPrimeThresholdSec = settings.skipPrimeThresholdSec;

        Serial.print("Loaded times: ");
        for (int i = 1; i <= 7; i++) {
            Serial.print(stepTimes[i]);
            Serial.print(" ");
        }
        Serial.println();
        Serial.print("Target temp: ");
        Serial.println(targetTemp);
    } else {
        Serial.println("No saved settings (magic mismatch), using defaults.");
    }
}

// ============ LEARN BREW FUNCTIONS ============

void startRecording() {
    if (isPlaying) return;
    allRelaysOff();
    recordedCount = 0;
    isRecording = true;
    recordStartTime = millis();
    Serial.println("=== RECORDING STARTED ===");
    Serial.println("Toggle relays to record your brew sequence.");
}

void stopRecording() {
    if (!isRecording) return;
    isRecording = false;
    Serial.println("=== RECORDING STOPPED ===");
    Serial.print("Recorded ");
    Serial.print(recordedCount);
    Serial.println(" actions.");
    allRelaysOff();
}

void recordAction(int relay, bool state) {
    if (!isRecording || recordedCount >= MAX_RECORDED_ACTIONS) return;

    unsigned long now = millis();
    unsigned long elapsed = now - recordStartTime;

    if (recordedCount > 0) {
        unsigned long prevTime = recordedActions[recordedCount - 1].timeMs;
        if (elapsed - prevTime < SIMULTANEOUS_THRESHOLD_MS) {
            elapsed = prevTime;
        }
    }

    recordedActions[recordedCount].timeMs = elapsed;
    recordedActions[recordedCount].relay = relay;
    recordedActions[recordedCount].state = state;
    recordedCount++;

    const char* relayNames[] = {"Pump", "Boiler", "Solenoid", "Warmer"};
    Serial.print("REC [");
    Serial.print(elapsed / 1000.0, 1);
    Serial.print("s] ");
    Serial.print(relayNames[relay]);
    Serial.println(state ? " ON" : " OFF");
}

void startPlayback() {
    if (isRecording || recordedCount == 0) return;
    isPlaying = true;
    playStartTime = millis();
    playIndex = 0;
    Serial.println("=== PLAYBACK STARTED ===");
    Serial.print("Playing ");
    Serial.print(recordedCount);
    Serial.println(" recorded actions.");
}

void updatePlayback() {
    if (!isPlaying) return;

    unsigned long elapsed = millis() - playStartTime;

    while (playIndex < recordedCount && recordedActions[playIndex].timeMs <= elapsed) {
        RecordedAction& action = recordedActions[playIndex];

        const int relayPins[] = {RELAY_PUMP, RELAY_BOILER, RELAY_SOLENOID, RELAY_WARMER};
        const char* relayNames[] = {"Pump", "Boiler", "Solenoid", "Warmer"};
        bool* relayStates[] = {&pumpOn, &heaterOn, &solenoidOn, &warmerOn};

        *relayStates[action.relay] = action.state;
        digitalWrite(relayPins[action.relay], action.state ? RELAY_ON : RELAY_OFF);

        Serial.print("PLAY [");
        Serial.print(action.timeMs / 1000.0, 1);
        Serial.print("s] ");
        Serial.print(relayNames[action.relay]);
        Serial.println(action.state ? " ON" : " OFF");

        playIndex++;
    }

    if (playIndex >= recordedCount) {
        isPlaying = false;
        Serial.println("=== PLAYBACK COMPLETE ===");
    }
}

void saveRecording() {
    int addr = sizeof(SavedSettings);
    EEPROM.put(addr, recordedCount);
    addr += sizeof(recordedCount);

    for (int i = 0; i < recordedCount; i++) {
        EEPROM.put(addr, recordedActions[i]);
        addr += sizeof(RecordedAction);
    }

    EEPROM.commit();
    Serial.println("Recording SAVED to flash!");
}

void loadRecording() {
    int addr = sizeof(SavedSettings);

    int count;
    EEPROM.get(addr, count);
    addr += sizeof(count);

    if (count > 0 && count <= MAX_RECORDED_ACTIONS) {
        recordedCount = count;
        for (int i = 0; i < recordedCount; i++) {
            EEPROM.get(addr, recordedActions[i]);
            addr += sizeof(RecordedAction);
        }
        Serial.print("Loaded ");
        Serial.print(recordedCount);
        Serial.println(" recorded actions.");
    } else {
        recordedCount = 0;
        Serial.println("No recorded sequence found.");
    }
}

String getStatusJson() {
    unsigned long stepElapsed = (millis() - stepStartTime) / 1000;
    unsigned int stepTime = stepTimes[brewState];

    if (brewState == PREHEAT && effectivePreheatTime > 0) {
        stepTime = effectivePreheatTime;
    }

    String json = "{";
    json += "\"temp\":" + String(boilerTemp, 1) + ",";
    json += "\"tempF\":" + String(toFahrenheit(boilerTemp), 1) + ",";
    json += "\"target\":" + String(targetTemp, 1) + ",";
    json += "\"targetF\":" + String(toFahrenheit(targetTemp), 1) + ",";
    json += "\"useF\":" + String(useFahrenheit ? "true" : "false") + ",";
    json += "\"flow\":" + String(flowRate, 2) + ",";
    json += "\"volume\":" + String(totalVolume, 1) + ",";
    json += "\"state\":\"" + String(stepNames[brewState]) + "\",";
    json += "\"step\":" + String((int)brewState) + ",";
    json += "\"stepElapsed\":" + String(stepElapsed) + ",";
    json += "\"stepTime\":" + String(stepTime) + ",";
    json += "\"times\":[";
    for (int i = 0; i < 9; i++) {
        json += String(stepTimes[i]);
        if (i < 8) json += ",";
    }
    json += "],";
    json += "\"pump\":" + String(pumpOn ? "true" : "false") + ",";
    json += "\"boiler\":" + String(heaterOn ? "true" : "false") + ",";
    json += "\"solenoid\":" + String(solenoidOn ? "true" : "false") + ",";
    json += "\"warmer\":" + String(warmerOn ? "true" : "false") + ",";
    json += "\"minTemp\":" + String(MIN_BREW_TEMP, 0) + ",";
    json += "\"wifiConnected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"ssid\":\"" + WiFi.SSID() + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    json += "\"uptime\":" + String(millis() / 1000) + ",";
    json += "\"freeHeap\":" + String(rp2040.getFreeHeap()) + ",";
    json += "\"pollInterval\":" + String(pollInterval) + ",";
    json += "\"testMode\":" + String(testMode ? "true" : "false") + ",";
    json += "\"softPump\":" + String(softPumpMode ? "true" : "false") + ",";
    json += "\"flowEnabled\":" + String(flowSensorEnabled ? "true" : "false") + ",";
    json += "\"recording\":" + String(isRecording ? "true" : "false") + ",";
    json += "\"playing\":" + String(isPlaying ? "true" : "false") + ",";
    json += "\"recCount\":" + String(recordedCount) + ",";
    json += "\"tempRate\":" + String(tempRatePerSec, 2) + ",";
    json += "\"etaTarget\":" + String(estimatedTimeToTarget, 0) + ",";
    json += "\"lastBrewSec\":" + String(timeSinceLastBrewSec()) + ",";
    json += "\"hasBrewedBefore\":" + String(hasBrewedBefore ? "true" : "false") + ",";
    json += "\"extButton\":" + String(!digitalRead(BREW_BUTTON) ? "true" : "false");
    json += "}";
    return json;
}

// Poll interval in ms
unsigned int pollInterval = 500;

// HTML stored in PROGMEM (flash)
const char MAIN_PAGE[] PROGMEM = R"rawliteral(<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><meta charset="UTF-8"><title>BrewForge</title><style>
*{box-sizing:border-box}body{font-family:Arial;background:#0d1117;color:#c9d1d9;margin:0;padding:10px}
.h{display:flex;align-items:center;gap:10px;margin-bottom:10px}.logo{width:40px;height:40px}h1{margin:0;font-size:20px;color:#58a6ff}
.c{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:12px;margin:8px 0}
.t{font-size:48px;font-weight:bold;color:#f78166;text-align:center}.ti{color:#8b949e;text-align:center}
.sb{display:flex;align-items:center;justify-content:space-between;gap:8px}
.st{font-size:16px;font-weight:bold;padding:6px 12px;border-radius:16px}
.st.IDLE{background:#238636;color:#fff}.st.TEMP_CHECK{background:#d29922;color:#000}
.st.PRESSURIZE{background:#1f6feb;color:#fff}.st.PREHEAT{background:#d29922;color:#000}
.st.PRIME{background:#f78166;color:#000}.st.PAUSE{background:#8957e5;color:#fff}
.st.BREW{background:#f78166;color:#000}.st.COOLDOWN{background:#79c0ff;color:#000}
.st.DONE{background:#238636;color:#fff}
.tm{font-size:20px;font-weight:bold;color:#58a6ff}
.rl{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
.rb{padding:12px;border-radius:6px;border:2px solid #30363d;cursor:pointer;font-size:13px;font-weight:600;text-align:center}
.rb.off{background:#21262d;color:#8b949e}.rb.on{background:#238636;color:#fff;border-color:#2ea043}
.bb{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.bn{padding:14px;border:none;border-radius:8px;font-size:16px;font-weight:bold;cursor:pointer}
.bg{background:#238636;color:#fff}.br{background:#da3633;color:#fff}
.ss{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.se{background:#21262d;border-radius:6px;padding:10px;text-align:center}
.sl{font-size:11px;color:#8b949e}.sv{font-size:18px;font-weight:bold;color:#58a6ff}
.ab{width:32px;height:32px;border-radius:50%;border:none;font-size:16px;font-weight:bold;cursor:pointer;margin:4px}
.am{background:#f85149;color:#fff}.ap{background:#238636;color:#fff}
.fi{display:flex;justify-content:space-around;text-align:center}
.fv{font-size:20px;font-weight:bold;color:#79c0ff}.fl{font-size:11px;color:#8b949e}
.rt{background:#30363d;color:#8b949e;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;font-size:11px}
.ib{background:#58a6ff;color:#0d1117;border:none;padding:8px 16px;border-radius:6px;cursor:pointer;font-weight:bold;font-size:12px}
.tu{cursor:pointer;text-decoration:underline}
.tr{font-size:13px;color:#8b949e;text-align:center;margin-top:4px}
</style></head><body>
<div class="h"><svg class="logo" viewBox="0 0 100 100"><defs><linearGradient id="s" x1="0%" y1="100%" x2="0%" y2="0%"><stop offset="0%" stop-color="#8b949e" stop-opacity="0.8"/><stop offset="100%" stop-color="#8b949e" stop-opacity="0"/></linearGradient><linearGradient id="u" x1="0%" y1="0%" x2="0%" y2="100%"><stop offset="0%" stop-color="#f78166"/><stop offset="100%" stop-color="#da3633"/></linearGradient></defs><path d="M35 35Q32 25 38 15" stroke="url(#s)" stroke-width="3" fill="none"/><path d="M50 30Q47 20 53 10" stroke="url(#s)" stroke-width="3" fill="none"/><path d="M65 35Q62 25 68 15" stroke="url(#s)" stroke-width="3" fill="none"/><path d="M20 45L25 85Q27 92 35 92L65 92Q73 92 75 85L80 45Z" fill="url(#u)"/><path d="M80 50Q95 50 95 65Q95 80 80 80" stroke="#f78166" stroke-width="6" fill="none"/><ellipse cx="50" cy="48" rx="28" ry="6" fill="#3d2817"/></svg><h1>BrewForge</h1><button class="ib" onclick="showInfo()" style="margin-left:auto">Info</button></div>
<div class="c"><div class="t" id="temp">--</div><div class="ti">Target: <span id="target">--</span><span id="unit" class="tu" onclick="toggleUnit()">C</span></div><div class="tr" id="tempRate"></div><div class="tr" id="lastBrew" style="font-size:11px">Last brew: never</div></div>
<div class="c"><div class="sb"><span class="st IDLE" id="state">IDLE</span><span class="tm" id="timer"></span><span id="extBtn" style="color:#8b949e;font-size:11px">BTN: ready</span></div>
<div class="rl"><button class="rb off" id="pump" onclick="T('pump')">Pump (GP4)</button><button class="rb off" id="boiler" onclick="T('boiler')">Boiler (GP5)</button><button class="rb off" id="solenoid" onclick="T('solenoid')">Solenoid (GP2)</button><button class="rb off" id="warmer" onclick="T('warmer')">Warmer (GP3)</button></div></div>
<div class="c bb"><button class="bn" style="background:#8b949e" onclick="P()">&lt; BACK</button><button class="bn bg" onclick="B()">BREW</button><button class="bn" style="background:#58a6ff" onclick="NX()">NEXT &gt;</button><button class="bn br" onclick="S()">STOP</button></div>
<div class="c" style="display:flex;gap:10px;justify-content:center"><button class="bn" id="btnRec" style="background:#da3633" onclick="doRec()">REC</button><button class="bn" id="btnPlay" style="background:#238636" onclick="doPlay()">PLAY</button><button class="rt" onclick="doSaveRec()">Save Seq</button><span id="recStatus" style="color:#8b949e;font-size:11px;align-self:center"></span></div>
<div class="c ss"><div class="se"><div class="sl">Target (<span id="tul">C</span>)</div><div class="sv" id="tv">93</div><button class="ab am" onclick="A('temp',-5)">-</button><button class="ab ap" onclick="A('temp',5)">+</button></div></div>
<div class="c" style="font-size:11px"><table style="width:100%;text-align:center;border-collapse:collapse"><tr style="color:#8b949e"><td>CHECK</td><td>PRESS</td><td>HEAT</td><td>PRIME</td><td>PAUSE</td><td>BREW</td><td>COOL</td></tr><tr><td><span id="t1">1</span>s</td><td><span id="t2">2</span>s</td><td><span id="t3">15</span>s</td><td><span id="t4">4</span>s</td><td><span id="t5">1</span>s</td><td><span id="t6">15</span>s</td><td><span id="t7">3</span>s</td></tr><tr><td><button class="rt" onclick="ST(1,-1)">-</button><button class="rt" onclick="ST(1,1)">+</button></td><td><button class="rt" onclick="ST(2,-1)">-</button><button class="rt" onclick="ST(2,1)">+</button></td><td><button class="rt" onclick="ST(3,-5)">-</button><button class="rt" onclick="ST(3,5)">+</button></td><td><button class="rt" onclick="ST(4,-1)">-</button><button class="rt" onclick="ST(4,1)">+</button></td><td><button class="rt" onclick="ST(5,-1)">-</button><button class="rt" onclick="ST(5,1)">+</button></td><td><button class="rt" onclick="ST(6,-5)">-</button><button class="rt" onclick="ST(6,5)">+</button></td><td><button class="rt" onclick="ST(7,-1)">-</button><button class="rt" onclick="ST(7,1)">+</button></td></tr></table></div>
<div class="c"><div class="fi"><div><div class="fv" id="flow">0</div><div class="fl">mL/s</div></div><div><div class="fv" id="vol">0</div><div class="fl">mL</div></div></div><center><button class="rt" onclick="fetch('/reset')">Reset</button></center></div>
<div class="c" style="font-size:12px;display:flex;gap:15px;justify-content:center;align-items:center;flex-wrap:wrap"><label><input type="checkbox" id="cbSoft" onchange="fetch('/softpump')"> Soft Pump</label><label><input type="checkbox" id="cbTest" onchange="fetch('/testmode')"> Test Mode</label><label><input type="checkbox" id="cbFlow" onchange="fetch('/flowsensor')"> Flow Sensor</label><button class="ib" onclick="doSave()" style="padding:4px 12px">SAVE</button></div>
<div class="c" style="font-size:11px;color:#8b949e"><span id="wifi">WiFi: --</span> | <span id="rssi">--</span>dBm | <span id="ip">--</span></div>
<div id="infopanel" style="display:none;position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(0,0,0,0.95);padding:20px;z-index:100;overflow-y:auto">
<div style="max-width:400px;margin:auto">
<h2 style="color:#58a6ff;margin-top:0">System Info</h2>
<div id="infolock">
<p style="color:#8b949e">Enter password to access system controls:</p>
<input type="password" id="infopw" style="background:#21262d;border:1px solid #30363d;color:#c9d1d9;padding:10px;width:100%;border-radius:6px;margin-bottom:10px" placeholder="Password">
<button class="ib" onclick="checkPw()">Unlock</button>
<button class="rt" onclick="hideInfo()" style="margin-left:10px">Cancel</button>
</div>
<div id="infocontent" style="display:none">
<div class="c"><div class="sl">BrewForge - Pico 2W (RP2350)</div>
<div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px">
<div><span class="fl">Uptime:</span><br><span class="fv" id="iup">--</span></div>
<div><span class="fl">Free RAM:</span><br><span class="fv" id="iheap">--</span></div>
<div><span class="fl">WiFi SSID:</span><br><span class="fv" id="issid">--</span></div>
<div><span class="fl">Signal:</span><br><span class="fv" id="irssi">--</span>dBm</div>
<div><span class="fl">IP Address:</span><br><span class="fv" id="iip">--</span></div>
<div><span class="fl">Temp Unit:</span><br><span class="fv" id="iunit">C</span></div>
<div><span class="fl">Poll (ms):</span><br><span class="fv" id="ipoll">500</span></div>
</div></div>
<div class="c"><div class="sl">System Controls</div>
<div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px">
<button class="bn" style="background:#d29922;color:#000" onclick="sysCmd('reboot')">Reboot</button>
<button class="bn br" onclick="sysCmd('sleep')">Deep Sleep</button>
<button class="bn" style="background:#8957e5;color:#fff" onclick="sysCmd('alloff')">All Relays Off</button>
<button class="bn" style="background:#238636;color:#fff" onclick="toggleUnit()">Toggle C/F</button>
<button class="bn" style="background:#30363d;color:#c9d1d9" onclick="sysCmd('defaults')">Reset Defaults</button>
<button class="bn" style="background:#30363d;color:#c9d1d9" onclick="sysCmd('wifireset')">WiFi Reconnect</button>
<button class="bn" style="background:#21262d;color:#8b949e" onclick="adjPoll(-500)">Poll -</button>
<button class="bn" style="background:#21262d;color:#8b949e" onclick="adjPoll(500)">Poll +</button>
</div></div>
<div class="c"><div class="sl">GPIO Pins</div>
<div style="font-size:11px;color:#8b949e;margin-top:8px">
GP2: Solenoid (RO valve) | GP3: Warmer (unused)<br>
GP4: Pump | GP5: Boiler<br>
GP6: Flow Sensor | GP7: Brew Button<br>
GP26: Thermistor (ADC0)<br>
Active HIGH relays (HIGH = ON)
</div></div>
<button class="bn" style="background:#30363d;color:#c9d1d9;width:100%;margin-top:10px" onclick="hideInfo()">Close</button>
</div></div></div>
<script>
var uF=false;
function U(){fetch('/status').then(r=>r.json()).then(d=>{
uF=d.useF;var tp=uF?d.tempF:d.temp;var tg=uF?d.targetF:d.target;var u=uF?'F':'C';
document.getElementById('temp').textContent=tp.toFixed(1)+u;
document.getElementById('target').textContent=tg.toFixed(1);
document.getElementById('unit').textContent=u;
document.getElementById('tul').textContent=u;
document.getElementById('tv').textContent=tg.toFixed(0);
document.getElementById('flow').textContent=d.flow;
document.getElementById('vol').textContent=d.volume;
var s=document.getElementById('state');s.textContent='['+d.step+'] '+d.state;s.className='st '+d.state;
var t=document.getElementById('timer');t.textContent=(d.step>0&&d.stepTime>0)?d.stepElapsed+'/'+d.stepTime+'s':(d.step>0?d.stepElapsed+'s':'');
['pump','boiler','solenoid','warmer'].forEach(r=>{document.getElementById(r).className='rb '+(d[r]?'on':'off')});
document.getElementById('wifi').textContent='WiFi: '+(d.wifiConnected?d.ssid:'--');
document.getElementById('rssi').textContent=d.rssi;
document.getElementById('ip').textContent=d.ip;
document.getElementById('iup').textContent=formatUptime(d.uptime);
document.getElementById('iheap').textContent=(d.freeHeap/1024).toFixed(1)+'KB';
document.getElementById('issid').textContent=d.ssid;
document.getElementById('irssi').textContent=d.rssi;
document.getElementById('iip').textContent=d.ip;
document.getElementById('iunit').textContent=u;
document.getElementById('ipoll').textContent=d.pollInterval;
document.getElementById('cbSoft').checked=d.softPump;
document.getElementById('cbTest').checked=d.testMode;
document.getElementById('cbFlow').checked=d.flowEnabled;
for(var i=1;i<=7;i++){document.getElementById('t'+i).textContent=d.times[i]}
var br=document.getElementById('btnRec');var bp=document.getElementById('btnPlay');var rs=document.getElementById('recStatus');
br.textContent=d.recording?'STOP REC':'REC';br.style.background=d.recording?'#f85149':'#da3633';
bp.textContent=d.playing?'STOP':'PLAY';bp.style.background=d.playing?'#f85149':'#238636';
rs.textContent=d.recCount>0?d.recCount+' actions':'No seq';
var rate=d.tempRate;var rateStr='';
if(d.step>=1&&d.step<=7){rateStr=(rate>=0?'+':'')+rate.toFixed(1)+'\u00B0/s';if(d.etaTarget>0&&d.step<=3)rateStr+=' | ETA: '+Math.round(d.etaTarget)+'s'}
document.getElementById('tempRate').textContent=rateStr;
var lb=d.hasBrewedBefore?formatLastBrew(d.lastBrewSec):'never';
document.getElementById('lastBrew').textContent='Last brew: '+lb;
var eb=document.getElementById('extBtn');eb.textContent='BTN: '+(d.extButton?'PRESSED':'ready');eb.style.color=d.extButton?'#238636':'#8b949e';
}).catch(e=>{})}
function formatUptime(s){var h=Math.floor(s/3600);var m=Math.floor((s%3600)/60);var sec=s%60;return h+'h '+m+'m '+sec+'s'}
function formatLastBrew(s){if(s<60)return s+'s ago';if(s<3600)return Math.floor(s/60)+'m ago';return Math.floor(s/3600)+'h ago'}
var L=0;function T(r){if(L)return;L=1;fetch('/toggle?r='+r).then(x=>x.json()).then(d=>{L=0;['pump','boiler','solenoid','warmer'].forEach(r=>{document.getElementById(r).className='rb '+(d[r]?'on':'off')})}).catch(e=>{L=0})}
function B(){fetch('/brew')}function S(){fetch('/stop')}function NX(){fetch('/next')}function P(){fetch('/prev')}function A(w,d){fetch('/adj?what='+w+'&delta='+d)}
function ST(s,d){fetch('/steptime?s='+s+'&d='+d).then(r=>r.text()).then(v=>{document.getElementById('t'+s).textContent=v})}
function doSave(){fetch('/save').then(r=>r.text()).then(x=>{alert('Settings saved!')})}
function doRec(){fetch('/rec').then(r=>r.text()).then(x=>{alert(x);U()})}
function doPlay(){fetch('/play').then(r=>r.text()).then(x=>{alert(x);U()})}
function doSaveRec(){fetch('/saverec').then(r=>r.text()).then(x=>{alert(x)})}
function toggleUnit(){fetch('/togglef').then(r=>r.text()).then(u=>{uF=(u==='F');U()})}
function showInfo(){document.getElementById('infopanel').style.display='block';document.getElementById('infolock').style.display='block';document.getElementById('infocontent').style.display='none';document.getElementById('infopw').value='';document.getElementById('infopw').focus()}
function hideInfo(){document.getElementById('infopanel').style.display='none'}
function checkPw(){fetch('/checkpw?pw='+encodeURIComponent(document.getElementById('infopw').value)).then(r=>r.text()).then(x=>{if(x==='OK'){document.getElementById('infolock').style.display='none';document.getElementById('infocontent').style.display='block'}else{alert('Wrong password')}})}
function sysCmd(c){if(c==='reboot'&&!confirm('Reboot the Pico?'))return;if(c==='sleep'&&!confirm('Enter deep sleep? Power cycle to wake.'))return;if(c==='defaults'&&!confirm('Reset settings to defaults?'))return;fetch('/sys?cmd='+c).then(r=>r.text()).then(x=>{alert(x);if(c==='reboot'||c==='sleep')hideInfo()})}
function adjPoll(d){fetch('/poll?delta='+d).then(r=>r.text()).then(x=>{document.getElementById('ipoll').textContent=x})}
var PI=500;setInterval(U,PI);U()
</script></body></html>)rawliteral";

String getWebPage() {
    String html = FPSTR(MAIN_PAGE);
    html.replace("var PI=500", "var PI=" + String(pollInterval));
    return html;
}

void setupWebServer() {
    server.on("/", []() {
        server.send(200, "text/html", getWebPage());
    });

    server.on("/status", []() {
        server.send(200, "application/json", getStatusJson());
    });

    server.on("/brew", []() {
        if (brewState == IDLE) {
            startBrewCycle();
            server.send(200, "text/plain", "Brewing started");
        } else {
            server.send(200, "text/plain", "Already running");
        }
    });

    server.on("/stop", []() {
        stopBrew();
        server.send(200, "text/plain", "Stopped");
    });

    server.on("/next", []() {
        nextStep();
        server.send(200, "text/plain", stepNames[brewState]);
    });

    server.on("/prev", []() {
        prevStep();
        server.send(200, "text/plain", stepNames[brewState]);
    });

    server.on("/steptime", []() {
        int step = server.arg("s").toInt();
        int delta = server.arg("d").toInt();
        if (step >= 1 && step <= 7) {
            int newTime = (int)stepTimes[step] + delta;
            if (newTime < 1) newTime = 1;
            if (newTime > 120) newTime = 120;
            stepTimes[step] = newTime;
            Serial.print("Step ");
            Serial.print(stepNames[step]);
            Serial.print(" time: ");
            Serial.print(newTime);
            Serial.println("s");
        }
        server.send(200, "text/plain", String(stepTimes[step]));
    });

    server.on("/save", []() {
        saveSettings();
        server.send(200, "text/plain", "SAVED!");
    });

    server.on("/rec", []() {
        String msg;
        if (isRecording) {
            stopRecording();
            msg = "STOPPED - " + String(recordedCount) + " actions recorded";
        } else {
            startRecording();
            msg = "RECORDING - Toggle relays now!";
        }
        server.send(200, "text/plain", msg);
    });

    server.on("/play", []() {
        String msg;
        if (isPlaying) {
            isPlaying = false;
            allRelaysOff();
            msg = "STOPPED playback";
        } else if (recordedCount == 0) {
            msg = "Nothing recorded! Record first.";
        } else {
            startPlayback();
            msg = "PLAYING " + String(recordedCount) + " actions...";
        }
        server.send(200, "text/plain", msg);
    });

    server.on("/saverec", []() {
        if (recordedCount == 0) {
            server.send(200, "text/plain", "Nothing to save! Record first.");
        } else {
            saveRecording();
            server.send(200, "text/plain", "Saved " + String(recordedCount) + " actions to flash!");
        }
    });

    server.on("/toggle", []() {
        String relay = server.arg("r");
        if (relay == "pump") {
            pumpOn = !pumpOn;
            setRelay(RELAY_PUMP, pumpOn, "Pump");
            if (isRecording) recordAction(0, pumpOn);
        } else if (relay == "boiler") {
            heaterOn = !heaterOn;
            setRelay(RELAY_BOILER, heaterOn, "Boiler");
            if (isRecording) recordAction(1, heaterOn);
        } else if (relay == "solenoid") {
            solenoidOn = !solenoidOn;
            setRelay(RELAY_SOLENOID, solenoidOn, "Solenoid");
            if (isRecording) recordAction(2, solenoidOn);
        } else if (relay == "warmer") {
            warmerOn = !warmerOn;
            setRelay(RELAY_WARMER, warmerOn, "Warmer");
            if (isRecording) recordAction(3, warmerOn);
        }
        server.send(200, "application/json", getStatusJson());
    });

    server.on("/adj", []() {
        String what = server.arg("what");
        int delta = server.arg("delta").toInt();

        if (what == "temp") {
            targetTemp += delta;
            if (targetTemp < 50) targetTemp = 50;
            if (targetTemp > 100) targetTemp = 100;
            Serial.print("Target temp: ");
            Serial.print(targetTemp);
            Serial.println(" C");
        } else if (what == "time") {
            brewTimeMs += delta * 1000;
            if (brewTimeMs < 5000) brewTimeMs = 5000;
            if (brewTimeMs > 60000) brewTimeMs = 60000;
        }
        server.send(200, "text/plain", "OK");
    });

    server.on("/togglef", []() {
        useFahrenheit = !useFahrenheit;
        server.send(200, "text/plain", useFahrenheit ? "F" : "C");
    });

    server.on("/softpump", []() {
        softPumpMode = !softPumpMode;
        Serial.print("Soft pump mode: ");
        Serial.println(softPumpMode ? "ON" : "OFF");
        server.send(200, "text/plain", softPumpMode ? "on" : "off");
    });

    server.on("/testmode", []() {
        testMode = !testMode;
        Serial.print("Test mode: ");
        Serial.println(testMode ? "ON" : "OFF");
        server.send(200, "text/plain", testMode ? "on" : "off");
    });

    server.on("/flowsensor", []() {
        flowSensorEnabled = !flowSensorEnabled;
        Serial.print("Flow sensor: ");
        Serial.println(flowSensorEnabled ? "ENABLED" : "DISABLED");
        server.send(200, "text/plain", flowSensorEnabled ? "on" : "off");
    });

    server.on("/reset", []() {
        totalVolume = 0;
        flowPulseCount = 0;
        Serial.println("Flow counter reset");
        server.send(200, "text/plain", "OK");
    });

    server.on("/poll", []() {
        int delta = server.arg("delta").toInt();
        pollInterval += delta;
        if (pollInterval < 500) pollInterval = 500;
        if (pollInterval > 10000) pollInterval = 10000;
        server.send(200, "text/plain", String(pollInterval));
    });

    server.on("/wifi", []() {
        wifiSetupDone = false;
        wifiConnecting = false;
        wifiNetworkIndex = 0;
        wifiRetryCount = 0;
        WiFi.disconnect();
        server.send(200, "text/plain", "Reconnecting...");
    });

    server.on("/checkpw", []() {
        String pw = server.arg("pw");
        if (pw == INFO_PASSWORD) {
            server.send(200, "text/plain", "OK");
        } else {
            server.send(200, "text/plain", "FAIL");
        }
    });

    server.on("/sys", []() {
        String cmd = server.arg("cmd");
        Serial.print("System command: ");
        Serial.println(cmd);

        if (cmd == "reboot") {
            server.send(200, "text/plain", "Rebooting...");
            delay(500);
            rp2040.reboot();
        } else if (cmd == "sleep") {
            server.send(200, "text/plain", "Entering deep sleep. Power cycle to wake.");
            delay(500);
            allRelaysOff();
            rp2040.reboot();
        } else if (cmd == "alloff") {
            allRelaysOff();
            server.send(200, "text/plain", "All relays OFF");
        } else if (cmd == "defaults") {
            targetTemp = 93.0;
            brewTimeMs = 25000;
            useFahrenheit = false;
            softPumpMode = false;
            flowSensorEnabled = true;
            stepTimes[1] = 1; stepTimes[2] = 2; stepTimes[3] = 15;
            stepTimes[4] = 4; stepTimes[5] = 1; stepTimes[6] = 15; stepTimes[7] = 3;
            allRelaysOff();
            server.send(200, "text/plain", "Settings reset to defaults");
        } else if (cmd == "wifireset") {
            server.send(200, "text/plain", "WiFi reconnecting...");
            wifiSetupDone = false;
            wifiConnecting = false;
            wifiNetworkIndex = 0;
            wifiRetryCount = 0;
            WiFi.disconnect();
        } else {
            server.send(400, "text/plain", "Unknown command");
        }
    });

    server.begin();
    Serial.println("Web server started");

    if (MDNS.begin("brewforge")) {
        Serial.println("mDNS: http://brewforge.local");
        MDNS.addService("http", "tcp", 80);
    }
}
