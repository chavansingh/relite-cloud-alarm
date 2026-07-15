#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>

// ====================================
// ESP Adhana Motor Updated Firmware
// One firmware image for all modules.
// Set MODULE_ID (1..30) uniquely per board.
// Each module controls only one motor relay on GPIO12.
// ====================================

// WiFi  E5573
const char* ssid = "Airtel-Hotspot-043E";
const char* password = "gjj6rday";

//const char* ssid = "AricssoIndia";
//const char* password = "0520@pwd";

// MQTT
const char* mqttBroker = "broker.hivemq.com";
const int mqttPort = 1883;
const char* mqttClientPrefix = "esp_adhana_motor_4_node";

// Node identity: change per board (1..30)
const int MODULE_ID = 25;

// Auto mode command: X2 raw ADC input below threshold enables AUTO.
// The D13 pin is no longer used for auto/manual mode selection.
const int MODE_SWITCH_PIN = 13; // retained for compatibility, not used for auto mode
const int AUTO_COMMAND_X2_CHANNEL = 2;
const int AUTO_COMMAND_X2_THRESHOLD = 2500;
const int MODE_LED_PIN = 15; // D15 mode indicator LED output
const bool MODE_LED_ON_IN_AUTO = true;

// Starter relays: D12 and D14 are timed; D27 stays ON until OFF command.
const int RELAY_PIN_12 = 12;
const int RELAY_PIN_14 = 14;
const int RELAY_PIN_27 = 27;
const bool RELAY_ACTIVE_HIGH = true;
const unsigned long STARTER_RELAY12_MS = 15000;
const unsigned long STARTER_D12_TO_D14_DELAY_MS = 10000;
const unsigned long STARTER_RELAY14_MS = 5000;

// CD4052 dual 4-channel analog multiplexer wiring:
// - X and Y (common pins) go to ESP analog inputs GPIO34/GPIO35
// - X0..X3 and Y0..Y3 are switched as channel pairs by A/B select lines
// - A/B select -> GPIO33/GPIO32
const int SENSOR_SEL_A_PIN = 33;
const int SENSOR_SEL_B_PIN = 32;
const int SENSOR_X_PIN = 34; // CD4052 X common -> raw/diagnostic input (ADC1 pin on ESP32)
const int SENSOR_Y_PIN = 35; // CD4052 Y common -> measurement input (voltage on Y0/Y1/Y2, current on Y3)
const int CURRENT_Y_CHANNEL = 3; // current sensor input from CD4052 Y3
const int SENSOR_CHANNEL = 1; // 0..3 channel pair: 0(X0/Y0), 1(X1/Y1), 2(X2/Y2), 3(X3/Y3)
const int VOLTAGE_R_Y_CHANNEL = 0; // R-phase voltage from Y0
const int VOLTAGE_Y_Y_CHANNEL = 1; // Y-phase voltage from Y1
const int VOLTAGE_B_Y_CHANNEL = 2; // B-phase voltage from Y2
const int X1_FAULT_THRESHOLD = 2500;
const int X3_RUNNING_THRESHOLD = 2000;
// Threshold used to decide hardware AUTO/MANUAL from X0 averaged reading
const int MODE_X0_THRESHOLD = 1500;

// If you have wired X/Y reversed for this board, set this to true to swap
// the logical X/Y used by the code without changing the hardware pin defs.
const bool SWAP_XY = true;

// Logical pin aliases (use these throughout the code). Do not change below.
const int SENSOR_X_LOGICAL_PIN = SWAP_XY ? SENSOR_Y_PIN : SENSOR_X_PIN;
const int SENSOR_Y_LOGICAL_PIN = SWAP_XY ? SENSOR_X_PIN : SENSOR_Y_PIN;

// ADC scaling (tune these with calibration measurements)
const float ADC_REF_V = 3.3f;
const float ADC_MAX = 4095.0f;
const float VOLTAGE_SCALE = 100.0f; // Raw sensor scaling before calibration
const float CURRENT_SCALE = 100.0f;  // Increased 10x per request
const float VOLTAGE_CAL_LOW_MEASURED = 0.8f;
const float VOLTAGE_CAL_LOW_ACTUAL = 150.0f;
const float VOLTAGE_CAL_HIGH_MEASURED = 10.4f;
const float VOLTAGE_CAL_HIGH_ACTUAL = 443.0f;
const float VOLTAGE_OUTPUT_FACTOR = 0.70f;

// Telemetry defaults
const float POWER_FACTOR = 0.98f;

// Publish intervals
const unsigned long TELEMETRY_INTERVAL_MS = 5000;
const unsigned long STATUS_HEARTBEAT_MS = 15000;
const unsigned long CD4052_TEST_PRINT_MS = 3000;
const unsigned long AUTO_START_DELAY_MS = 30000;
const unsigned long AUTO_CHECK_INTERVAL_MS = 5000;
const unsigned long SCHEDULE_CHECK_INTERVAL_MS = 30000;
const int SCHEDULE_COUNT = 3;
const long NTP_GMT_OFFSET_SEC = 19800; // UTC+5:30
const int NTP_DAYLIGHT_OFFSET_SEC = 0;
const char* NTP_SERVER1 = "pool.ntp.org";
const char* NTP_SERVER2 = "time.nist.gov";
const size_t TELEMETRY_JSON_CAPACITY = 1024;
const uint16_t MQTT_BUFFER_SIZE = 2048;

// Set true to validate ADC2 reads (e.g., GPIO27) without WiFi/MQTT interference.
const bool ADC2_TEST_MODE = false;

WiFiClient motorControllerClient;
PubSubClient mqttClient(motorControllerClient);

String topicControl;
String topicStatus;
String topicTelemetry;
String topicAllControl;
String topicAllData;
String moduleId4;

unsigned long lastTelemetryMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastCd4052PrintMs = 0;

bool relayStateOn = false;
bool pendingPublishStatus = false;
bool starterSequenceActive = false;
unsigned long starterSequenceStartMs = 0;
bool manualRunLatchActive = false;
bool lastVoltageOk = true;
bool autoModeEnabled = true;
bool modeCommandOverrideActive = false;
bool lastHardwareAutoMode = true;
unsigned long bootMs = 0;
unsigned long lastAutoCheckMs = 0;
unsigned long lastScheduleCheckMs = 0;

const unsigned long MODE_DECISION_WINDOW_MS = 30000;
const unsigned long MODE_SAMPLE_INTERVAL_MS = 1000;
unsigned long lastModeSampleMs = 0;
unsigned long modeWindowStartMs = 0;
long modeDecisionSum = 0;
int modeDecisionCount = 0;
int modeDecisionRaw = 0;
bool modeDecisionReady = false;
bool modeDecisionHardwareAutoMode = true;
bool x1DecisionVoltageOk = true;

struct ScheduleEntry {
  uint8_t onHour;
  uint8_t onMinute;
  uint8_t offHour;
  uint8_t offMinute;
  bool enabled;
  int lastTriggeredDayOfYearOn;
  int lastTriggeredDayOfYearOff;
};

ScheduleEntry scheduleEntries[SCHEDULE_COUNT];
Preferences schedulePreferences;
Preferences sequencePreferences;
Preferences controlPreferences;

unsigned long starterRelay12Ms = STARTER_RELAY12_MS;
unsigned long starterD12ToD14DelayMs = STARTER_D12_TO_D14_DELAY_MS;
unsigned long starterRelay14Ms = STARTER_RELAY14_MS;

int readAveragedAdc(int pin, int samples) {
  long total = 0;
  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delayMicroseconds(600);
  }
  return (int)(total / samples);
}

void selectSensorChannel(int channel) {
  const int safeChannel = constrain(channel, 0, 3);
  // CD4052 select map: AB=00->ch0, 01->ch1, 10->ch2, 11->ch3
  digitalWrite(SENSOR_SEL_A_PIN, (safeChannel & 0x01) ? HIGH : LOW);
  digitalWrite(SENSOR_SEL_B_PIN, (safeChannel & 0x02) ? HIGH : LOW);
  delayMicroseconds(80);
}

float readVoltageFromYChannel(int yChannel) {
  selectSensorChannel(yChannel);
  analogRead(SENSOR_Y_LOGICAL_PIN); // discard first sample after channel switch
  delayMicroseconds(200);
  const int raw = readAveragedAdc(SENSOR_Y_LOGICAL_PIN, 500);
  const float adcVoltage = (raw * ADC_REF_V) / ADC_MAX;
  const float sensedVoltage = adcVoltage * VOLTAGE_SCALE;

  if (sensedVoltage <= 0.0f) {
    return 0.0f;
  }

  if (sensedVoltage <= VOLTAGE_CAL_LOW_MEASURED) {
    return sensedVoltage * (VOLTAGE_CAL_LOW_ACTUAL / VOLTAGE_CAL_LOW_MEASURED) * VOLTAGE_OUTPUT_FACTOR;
  }

  return (VOLTAGE_CAL_LOW_ACTUAL +
          ((sensedVoltage - VOLTAGE_CAL_LOW_MEASURED) *
           (VOLTAGE_CAL_HIGH_ACTUAL - VOLTAGE_CAL_LOW_ACTUAL) /
           (VOLTAGE_CAL_HIGH_MEASURED - VOLTAGE_CAL_LOW_MEASURED))) * VOLTAGE_OUTPUT_FACTOR;
}

float readCurrentFromSensor() {
  selectSensorChannel(CURRENT_Y_CHANNEL);
  analogRead(SENSOR_Y_LOGICAL_PIN); // discard first sample after channel switch
  delayMicroseconds(200);
  const int raw = readAveragedAdc(SENSOR_Y_LOGICAL_PIN, 500);
  const float adcVoltage = (raw * ADC_REF_V) / ADC_MAX;
  return adcVoltage * CURRENT_SCALE;
}

int readSensorChannelRaw(int channel, int samples = 16) {
  selectSensorChannel(channel);
  analogRead(SENSOR_X_LOGICAL_PIN); // discard first sample after channel switch
  delayMicroseconds(200);
  return readAveragedAdc(SENSOR_X_LOGICAL_PIN, samples);
}

bool isRunningFromX3Raw(int x3Raw) {
  return x3Raw < X3_RUNNING_THRESHOLD;
}

bool isVoltageOkFromX1Raw(int x1Raw) {
  return x1Raw < X1_FAULT_THRESHOLD;
}

void resetModeDecisionWindow() {
  modeWindowStartMs = millis();
  lastModeSampleMs = modeWindowStartMs;
  modeDecisionSum = 0;
  modeDecisionCount = 0;
  modeDecisionReady = false;
}

void updateModeDecisionWindow() {
  const unsigned long now = millis();
  if (now - lastModeSampleMs < MODE_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastModeSampleMs = now;

  const int sample = readSensorChannelRaw(0, 50);
  modeDecisionSum += sample;
  modeDecisionCount++;

  if (now - modeWindowStartMs < MODE_DECISION_WINDOW_MS) {
    return;
  }

  if (modeDecisionCount <= 0) {
    modeWindowStartMs = now;
    return;
  }

  modeDecisionRaw = (int)(modeDecisionSum / modeDecisionCount);
  modeDecisionHardwareAutoMode = modeDecisionRaw < MODE_X0_THRESHOLD;
  x1DecisionVoltageOk = modeDecisionRaw < X1_FAULT_THRESHOLD;
  modeDecisionReady = true;

  Serial.print("[MODE] 30s avg X0=");
  Serial.print(modeDecisionRaw);
  Serial.print(" mode=");
  Serial.print(modeDecisionHardwareAutoMode ? "AUTO" : "MANUAL");
  Serial.println();

  modeDecisionSum = 0;
  modeDecisionCount = 0;
  modeWindowStartMs = now;
}

bool getHardwareModeFromModeDecision() {
  if (!modeDecisionReady) {
    return autoModeEnabled;
  }
  return modeDecisionHardwareAutoMode;
}

bool getVoltageOkFromModeDecision() {
  if (!modeDecisionReady) {
    return lastVoltageOk;
  }
  return x1DecisionVoltageOk;
}

bool readHardwareAutoMode() {
  // Use immediate X0 reading only for startup if the 30s decision window is not yet ready.
  if (modeDecisionReady) {
    return getHardwareModeFromModeDecision();
  }

  const int x0Raw = readSensorChannelRaw(0, 50);
  return x0Raw < MODE_X0_THRESHOLD;
}

void updateModeIndicator() {
  const bool ledOn = MODE_LED_ON_IN_AUTO ? autoModeEnabled : !autoModeEnabled;
  digitalWrite(MODE_LED_PIN, ledOn ? HIGH : LOW);
}

void syncHardwareMode() {
  if (!modeDecisionReady) {
    return;
  }

  const bool hardwareAutoMode = getHardwareModeFromModeDecision();

  // If dashboard command set the mode, keep it until the hardware input is physically toggled.
  if (modeCommandOverrideActive) {
    // Detect a physical toggle on the hardware input compared to last observed hardware state.
    if (hardwareAutoMode != lastHardwareAutoMode) {
      // Hardware was toggled; clear dashboard override and adopt hardware mode.
      modeCommandOverrideActive = false;
      autoModeEnabled = hardwareAutoMode;
      if (autoModeEnabled) {
        lastAutoCheckMs = 0;
      }
      updateModeIndicator();
      pendingPublishStatus = true;
      Serial.println("[MODE] Hardware toggle detected - dashboard override cleared");
      Serial.print("[MODE] Now using hardware input mode: ");
      Serial.println(autoModeEnabled ? "AUTO" : "MANUAL");
    }
    lastHardwareAutoMode = hardwareAutoMode;
    return;
  }

  if (hardwareAutoMode == autoModeEnabled) {
    lastHardwareAutoMode = hardwareAutoMode;
    return;
  }

  autoModeEnabled = hardwareAutoMode;
  if (autoModeEnabled) {
    lastAutoCheckMs = 0;
  }
  updateModeIndicator();
  pendingPublishStatus = true;
  Serial.print("[MODE] Hardware input set mode to ");
  Serial.println(autoModeEnabled ? "AUTO" : "MANUAL");
  lastHardwareAutoMode = hardwareAutoMode;
}

void printCd4052AllValues() {
  int xValues[4];
  int yValues[4];

  for (int ch = 0; ch < 4; ch++) {
    selectSensorChannel(ch);
    analogRead(SENSOR_X_LOGICAL_PIN); // discard first sample after channel switch
    analogRead(SENSOR_Y_LOGICAL_PIN);
    delayMicroseconds(200);
    xValues[ch] = readAveragedAdc(SENSOR_X_LOGICAL_PIN, 8);
    yValues[ch] = readAveragedAdc(SENSOR_Y_LOGICAL_PIN, 8);
  }

  Serial.print("[CD4052 TEST] X0=");
  Serial.print(xValues[0]);
  Serial.print(", X1=");
  Serial.print(xValues[1]);
  Serial.print(", X2=");
  Serial.print(xValues[2]);
  Serial.print(", X3=");
  Serial.print(xValues[3]);
  Serial.print(" | Y0=");
  Serial.print(yValues[0]);
  Serial.print(", Y1=");
  Serial.print(yValues[1]);
  Serial.print(", Y2=");
  Serial.print(yValues[2]);
  Serial.print(", Y3=");
  Serial.println(yValues[3]);
}

String formatMotorId4(int id) {
  char buf[5];
  snprintf(buf, sizeof(buf), "%04d", id);
  return String(buf);
}

bool parseOnCommand(const String& msg) {
  String value = msg;
  value.trim();
  value.toUpperCase();
  return (value == "ON" || value == "1" || value == "HIGH" || value == "START");
}

bool parseOffCommand(const String& msg) {
  String value = msg;
  value.trim();
  value.toUpperCase();
  return (value == "OFF" || value == "0" || value == "LOW" || value == "STOP");
}

void writeRelayPin(int pin, bool turnOn) {
  const int level = turnOn
    ? (RELAY_ACTIVE_HIGH ? HIGH : LOW)
    : (RELAY_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(pin, level);
}

bool isRelayPinOn(int pin) {
  const int level = digitalRead(pin);
  return RELAY_ACTIVE_HIGH ? (level == HIGH) : (level == LOW);
}

void updateStarterRelays() {
  if (!relayStateOn) {
    writeRelayPin(RELAY_PIN_12, false);
    writeRelayPin(RELAY_PIN_14, false);
    writeRelayPin(RELAY_PIN_27, false);
    starterSequenceActive = false;
    return;
  }

  writeRelayPin(RELAY_PIN_27, true);

  if (!starterSequenceActive) {
    writeRelayPin(RELAY_PIN_12, false);
    writeRelayPin(RELAY_PIN_14, false);
    return;
  }

  const unsigned long elapsed = millis() - starterSequenceStartMs;
  if (elapsed < starterRelay12Ms) {
    writeRelayPin(RELAY_PIN_12, true);
    writeRelayPin(RELAY_PIN_14, false);
  } else if (elapsed < (starterRelay12Ms + starterD12ToD14DelayMs)) {
    writeRelayPin(RELAY_PIN_12, false);
    writeRelayPin(RELAY_PIN_14, false);
  } else if (elapsed < (starterRelay12Ms + starterD12ToD14DelayMs + starterRelay14Ms)) {
    writeRelayPin(RELAY_PIN_12, false);
    writeRelayPin(RELAY_PIN_14, true);
  } else {
    writeRelayPin(RELAY_PIN_12, false);
    writeRelayPin(RELAY_PIN_14, false);
    starterSequenceActive = false;
  }
}

void startStarterOnSequence() {
  relayStateOn = true;
  starterSequenceActive = true;
  starterSequenceStartMs = millis();
  updateStarterRelays();
}

void stopStarter() {
  relayStateOn = false;
  starterSequenceActive = false;
  updateStarterRelays();
}

void setDefaultSchedule() {
  for (int i = 0; i < SCHEDULE_COUNT; i++) {
    scheduleEntries[i].enabled = false;
    scheduleEntries[i].onHour = 6 + i * 6;
    scheduleEntries[i].onMinute = 0;
    scheduleEntries[i].offHour = scheduleEntries[i].onHour + 1;
    if (scheduleEntries[i].offHour > 23) {
      scheduleEntries[i].offHour = 23;
    }
    scheduleEntries[i].offMinute = 0;
    scheduleEntries[i].lastTriggeredDayOfYearOn = -1;
    scheduleEntries[i].lastTriggeredDayOfYearOff = -1;
  }
}

bool saveScheduleToPreferences() {
  DynamicJsonDocument doc(512);
  JsonArray scheduleArray = doc.createNestedArray("schedule");

  for (int i = 0; i < SCHEDULE_COUNT; i++) {
    JsonObject entry = scheduleArray.createNestedObject();
    entry["enabled"] = scheduleEntries[i].enabled;
    entry["on_hour"] = scheduleEntries[i].onHour;
    entry["on_minute"] = scheduleEntries[i].onMinute;
    entry["off_hour"] = scheduleEntries[i].offHour;
    entry["off_minute"] = scheduleEntries[i].offMinute;
  }

  char buffer[512];
  const size_t len = serializeJson(doc, buffer, sizeof(buffer));
  if (len == 0 || len >= sizeof(buffer) - 1) {
    return false;
  }

  schedulePreferences.begin("schedule", false);
  const bool ok = schedulePreferences.putString("schedule_data", buffer) > 0;
  schedulePreferences.end();
  return ok;
}

bool saveStarterSequenceToPreferences() {
  DynamicJsonDocument doc(256);
  JsonObject seq = doc.createNestedObject("starter_sequence");
  seq["relay12_ms"] = starterRelay12Ms;
  seq["d12_to_d14_delay_ms"] = starterD12ToD14DelayMs;
  seq["relay14_ms"] = starterRelay14Ms;

  char buffer[256];
  const size_t len = serializeJson(doc, buffer, sizeof(buffer));
  if (len == 0 || len >= sizeof(buffer) - 1) {
    return false;
  }

  sequencePreferences.begin("starter", false);
  const bool ok = sequencePreferences.putString("sequence_data", buffer) > 0;
  sequencePreferences.end();
  return ok;
}

void loadStarterSequenceFromPreferences() {
  starterRelay12Ms = STARTER_RELAY12_MS;
  starterD12ToD14DelayMs = STARTER_D12_TO_D14_DELAY_MS;
  starterRelay14Ms = STARTER_RELAY14_MS;

  sequencePreferences.begin("starter", true);
  const String stored = sequencePreferences.getString("sequence_data", "");
  sequencePreferences.end();

  if (stored.length() == 0) {
    return;
  }

  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, stored);
  if (err) {
    Serial.print("[SEQUENCE] load failed: ");
    Serial.println(err.c_str());
    return;
  }

  if (!doc.containsKey("starter_sequence")) {
    return;
  }

  JsonObjectConst seq = doc["starter_sequence"].as<JsonObjectConst>();
  starterRelay12Ms = (unsigned long)constrain((long)(seq["relay12_ms"] | (long)starterRelay12Ms), 1000, 60000);
  starterD12ToD14DelayMs = (unsigned long)constrain((long)(seq["d12_to_d14_delay_ms"] | (long)starterD12ToD14DelayMs), 0, 60000);
  starterRelay14Ms = (unsigned long)constrain((long)(seq["relay14_ms"] | (long)starterRelay14Ms), 1000, 60000);
}

void saveModeOverrideToPreferences() {
  controlPreferences.begin("control", false);
  controlPreferences.putBool("mode_override", modeCommandOverrideActive);
  controlPreferences.putBool("auto_mode", autoModeEnabled);
  controlPreferences.end();
}

void loadModeOverrideFromPreferences() {
  controlPreferences.begin("control", true);
  modeCommandOverrideActive = controlPreferences.getBool("mode_override", false);
  autoModeEnabled = controlPreferences.getBool("auto_mode", autoModeEnabled);
  controlPreferences.end();
  if (modeCommandOverrideActive) {
    updateModeIndicator();
    pendingPublishStatus = true;
    Serial.println("[MODE] Restored dashboard mode override from prefs");
  }
}

bool parseStarterSequenceCommand(const JsonDocument& doc) {
  if (!doc.containsKey("starter_sequence")) {
    return false;
  }

  JsonObjectConst seq = doc["starter_sequence"].as<JsonObjectConst>();
  const long relay12 = seq["relay12_ms"] | (long)starterRelay12Ms;
  const long delayMs = seq["d12_to_d14_delay_ms"] | (long)starterD12ToD14DelayMs;
  const long relay14 = seq["relay14_ms"] | (long)starterRelay14Ms;

  starterRelay12Ms = (unsigned long)constrain(relay12, 1000, 60000);
  starterD12ToD14DelayMs = (unsigned long)constrain(delayMs, 0, 60000);
  starterRelay14Ms = (unsigned long)constrain(relay14, 1000, 60000);

  if (saveStarterSequenceToPreferences()) {
    pendingPublishStatus = true;
    Serial.println("[SEQUENCE] Starter sequence updated from MQTT");
    return true;
  }

  Serial.println("[SEQUENCE] Failed to save starter sequence from MQTT");
  return true;
}

void loadScheduleFromPreferences() {
  setDefaultSchedule();
  schedulePreferences.begin("schedule", true);
  const String stored = schedulePreferences.getString("schedule_data", "");
  schedulePreferences.end();

  if (stored.length() == 0) {
    return;
  }

  DynamicJsonDocument doc(512);
  DeserializationError err = deserializeJson(doc, stored);
  if (err) {
    Serial.print("[SCHEDULE] load failed: ");
    Serial.println(err.c_str());
    return;
  }

  if (!doc.containsKey("schedule")) {
    return;
  }

  JsonArray scheduleArray = doc["schedule"].as<JsonArray>();
  int index = 0;
  for (JsonVariant item : scheduleArray) {
    if (index >= SCHEDULE_COUNT) {
      break;
    }
    scheduleEntries[index].enabled = item["enabled"] | false;
    scheduleEntries[index].onHour = constrain(item["on_hour"] | scheduleEntries[index].onHour, 0, 23);
    scheduleEntries[index].onMinute = constrain(item["on_minute"] | scheduleEntries[index].onMinute, 0, 59);
    scheduleEntries[index].offHour = constrain(item["off_hour"] | scheduleEntries[index].offHour, 0, 23);
    scheduleEntries[index].offMinute = constrain(item["off_minute"] | scheduleEntries[index].offMinute, 0, 59);
    scheduleEntries[index].lastTriggeredDayOfYearOn = -1;
    scheduleEntries[index].lastTriggeredDayOfYearOff = -1;
    index++;
  }
}

void applyScheduleLogic() {
  const unsigned long nowMs = millis();
  if (nowMs - lastScheduleCheckMs < SCHEDULE_CHECK_INTERVAL_MS) {
    return;
  }
  lastScheduleCheckMs = nowMs;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return;
  }

  const int dayOfYear = timeinfo.tm_yday;
  for (int i = 0; i < SCHEDULE_COUNT; i++) {
    ScheduleEntry& entry = scheduleEntries[i];
    if (!entry.enabled) {
      continue;
    }

    if (entry.onHour == timeinfo.tm_hour && entry.onMinute == timeinfo.tm_min && entry.lastTriggeredDayOfYearOn != dayOfYear) {
      startStarterOnSequence();
      Serial.printf("[SCHEDULE] %02d:%02d -> ON\n", entry.onHour, entry.onMinute);
      entry.lastTriggeredDayOfYearOn = dayOfYear;
      pendingPublishStatus = true;
    }

    if (entry.offHour == timeinfo.tm_hour && entry.offMinute == timeinfo.tm_min && entry.lastTriggeredDayOfYearOff != dayOfYear) {
      if (manualRunLatchActive) {
        Serial.printf("[SCHEDULE] %02d:%02d -> OFF skipped (manual latch active)\n", entry.offHour, entry.offMinute);
      } else {
        stopStarter();
        Serial.printf("[SCHEDULE] %02d:%02d -> OFF\n", entry.offHour, entry.offMinute);
      }
      entry.lastTriggeredDayOfYearOff = dayOfYear;
      pendingPublishStatus = true;
    }
  }
}

void applyAutoModeLogic() {
  if (!autoModeEnabled || !modeDecisionReady) {
    return;
  }

  const unsigned long now = millis();
  if (now - bootMs < AUTO_START_DELAY_MS) {
    return;
  }

  if (x1DecisionVoltageOk && !relayStateOn) {
    startStarterOnSequence();
    pendingPublishStatus = true;
    Serial.println("[AUTO] Voltage OK after 30s average, starter ON");
  }

  if (!x1DecisionVoltageOk && relayStateOn) {
    stopStarter();
    pendingPublishStatus = true;
    Serial.println("[AUTO] Voltage FAULT after 30s average, starter forced OFF");
  }
}

void applyX1FaultSafety() {
  if (!modeDecisionReady) {
    return;
  }

  if (x1DecisionVoltageOk != lastVoltageOk) {
    pendingPublishStatus = true;
    Serial.print("[X1] Voltage state changed: ");
    Serial.print(x1DecisionVoltageOk ? "Voltage OK" : "Faulty");
    Serial.print(" (30s avg X0=");
    Serial.print(modeDecisionRaw);
    Serial.println(")");
    lastVoltageOk = x1DecisionVoltageOk;
  }
}

void publishStatus(bool retained) {
  const int x3Raw = readSensorChannelRaw(3, 16);
  const bool x3Running = isRunningFromX3Raw(x3Raw);
  const char* statusText = x3Running ? "ON" : "OFF";

  StaticJsonDocument<512> doc;
  doc["status"] = statusText;
  doc["state"] = statusText;
  doc["relay_on"] = x3Running;
  doc["motor"] = x3Running;
  doc["io12_status"] = x3Running;
  doc["mode"] = autoModeEnabled ? "AUTO" : "MANUAL";
  doc["mode_auto"] = autoModeEnabled;
  doc["manual_latch"] = manualRunLatchActive;
  doc["mode_source"] = modeCommandOverrideActive ? "DASHBOARD" : "X0";
  doc["mode_override"] = modeCommandOverrideActive;
  doc["mode_x0_raw"] = modeDecisionReady ? modeDecisionRaw : readSensorChannelRaw(0, 50);
  doc["module_id"] = MODULE_ID;
  doc["module_id4"] = moduleId4;

  JsonObject starterConfig = doc.createNestedObject("starter_sequence");
  starterConfig["relay12_ms"] = starterRelay12Ms;
  starterConfig["d12_to_d14_delay_ms"] = starterD12ToD14DelayMs;
  starterConfig["relay14_ms"] = starterRelay14Ms;

  JsonArray scheduleArray = doc.createNestedArray("schedule");
  for (int i = 0; i < SCHEDULE_COUNT; i++) {
    JsonObject entry = scheduleArray.createNestedObject();
    entry["enabled"] = scheduleEntries[i].enabled;
    entry["on_hour"] = scheduleEntries[i].onHour;
    entry["on_minute"] = scheduleEntries[i].onMinute;
    entry["off_hour"] = scheduleEntries[i].offHour;
    entry["off_minute"] = scheduleEntries[i].offMinute;
  }

  char payload[512];
  const size_t payloadLen = serializeJson(doc, payload, sizeof(payload));
  if (payloadLen == 0 || payloadLen >= sizeof(payload) - 1) {
    Serial.println("[MQTT] ERROR: status serialize failed/too large");
    return;
  }

  mqttClient.publish(topicStatus.c_str(), payload, retained);
}

void publishTelemetry() {
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] WARN: skip telemetry publish, MQTT disconnected");
    return;
  }

  const int x1Raw = readSensorChannelRaw(1, 16);
  const bool voltageOk = isVoltageOkFromX1Raw(x1Raw);
  const bool faultActive = !voltageOk;

  const int x3Raw = readSensorChannelRaw(3, 16);
  const bool x3Running = isRunningFromX3Raw(x3Raw);

  const float voltageR = readVoltageFromYChannel(VOLTAGE_R_Y_CHANNEL);
  const float voltageY = readVoltageFromYChannel(VOLTAGE_Y_Y_CHANNEL);
  const float voltageB = readVoltageFromYChannel(VOLTAGE_B_Y_CHANNEL);
  const float voltage = (voltageR + voltageY + voltageB) / 3.0f;
  float current = readCurrentFromSensor();
  if (!x3Running && current < 0.25f) {
    current = 0.0f;
  }

  const float totalPhaseVoltage = voltageR + voltageY + voltageB;
  const float power = totalPhaseVoltage * current * POWER_FACTOR;

  // Keep telemetry compact to reduce publish failures on unstable links.
  StaticJsonDocument<TELEMETRY_JSON_CAPACITY> doc;
  doc["module_id"] = MODULE_ID;
  doc["module_id4"] = moduleId4;
  doc["motor"] = x3Running;
  doc["io12_status"] = x3Running;
  doc["x1"] = x1Raw;
  doc["x3"] = x3Raw;
  doc["x3_running"] = x3Running;
  doc["voltage_ok"] = voltageOk;
  doc["fault"] = faultActive;
  doc["voltage_status"] = voltageOk ? "Voltage OK" : "Faulty";
  doc["mode"] = autoModeEnabled ? "AUTO" : "MANUAL";
  doc["mode_auto"] = autoModeEnabled;
  doc["manual_latch"] = manualRunLatchActive;
  doc["voltage_r"] = voltageR;
  doc["voltage_y"] = voltageY;
  doc["voltage_b"] = voltageB;
  doc["voltage"] = voltage;
  doc["current"] = current;
  doc["power"] = power;
  doc["timestamp_ms"] = millis();

  char payload[TELEMETRY_JSON_CAPACITY];
  const size_t payloadLen = serializeJson(doc, payload, sizeof(payload));
  if (payloadLen == 0 || payloadLen >= sizeof(payload) - 1) {
    Serial.print("[MQTT] ERROR: telemetry serialize failed/too large, len=");
    Serial.println((unsigned int)payloadLen);
    return;
  }

  const bool telemetryOk = mqttClient.publish(topicTelemetry.c_str(), payload, false);
  const bool allDataOk = mqttClient.publish(topicAllData.c_str(), payload, false);
  if (!telemetryOk || !allDataOk) {
    Serial.print("[MQTT] ERROR: telemetry publish failed, state=");
    Serial.print(mqttClient.state());
    Serial.print(", len=");
    Serial.println((unsigned int)payloadLen);
  }
}

void handleControlMessage(const String& msg) {
  String value = msg;
  value.trim();
  value.toUpperCase();

  if (value == "AUTO" || value == "MODE_AUTO") {
    autoModeEnabled = true;
    manualRunLatchActive = false;
    modeCommandOverrideActive = true;
    lastAutoCheckMs = 0;
    updateModeIndicator();
    pendingPublishStatus = true;
    saveModeOverrideToPreferences();
    Serial.println("[CTRL] Mode set to AUTO from dashboard (override active until hardware input toggle)");
    return;
  }

  if (value == "MANUAL" || value == "MODE_MANUAL") {
    autoModeEnabled = false;
    modeCommandOverrideActive = true;
    updateModeIndicator();
    pendingPublishStatus = true;
    saveModeOverrideToPreferences();
    Serial.println("[CTRL] Mode set to MANUAL from dashboard (override active until hardware input toggle)");
    return;
  }

  if (parseOnCommand(msg)) {
    if (autoModeEnabled) {
      Serial.println("[CTRL] ON ignored in AUTO mode. Switch to MANUAL for button control.");
      return;
    }
    manualRunLatchActive = true;
    startStarterOnSequence();
    pendingPublishStatus = true;
    Serial.println("[CTRL] MANUAL ON: latched ON until manual OFF. STARTER ON sequence: D12(15s) -> wait(10s) -> D14(5s), D27 latched ON");
    return;
  }

  if (parseOffCommand(msg)) {
    manualRunLatchActive = false;
    stopStarter();
    pendingPublishStatus = true;
    Serial.println("[CTRL] MANUAL OFF: latch cleared, D12/D14/D27 OFF");
    return;
  }

  if (value == "TOGGLE") {
    if (autoModeEnabled) {
      Serial.println("[CTRL] TOGGLE ignored in AUTO mode. Switch to MANUAL for button control.");
      return;
    }
    if (relayStateOn) {
      manualRunLatchActive = false;
      stopStarter();
    } else {
      manualRunLatchActive = true;
      startStarterOnSequence();
    }
    pendingPublishStatus = true;
    Serial.println("[CTRL] STARTER TOGGLE");
  }
}

bool parseScheduleCommand(const JsonDocument& doc) {
  if (!doc.containsKey("schedule")) {
    return false;
  }

  JsonArrayConst scheduleArray = doc["schedule"].as<JsonArrayConst>();
  if (scheduleArray.size() == 0) {
    return false;
  }

  for (int i = 0; i < SCHEDULE_COUNT; i++) {
    scheduleEntries[i].enabled = false;
    scheduleEntries[i].onHour = 6 + i * 6;
    scheduleEntries[i].onMinute = 0;
    scheduleEntries[i].offHour = scheduleEntries[i].onHour + 1;
    if (scheduleEntries[i].offHour > 23) {
      scheduleEntries[i].offHour = 23;
    }
    scheduleEntries[i].offMinute = 0;
    scheduleEntries[i].lastTriggeredDayOfYearOn = -1;
    scheduleEntries[i].lastTriggeredDayOfYearOff = -1;
  }

  int index = 0;
  for (JsonVariantConst item : scheduleArray) {
    if (index >= SCHEDULE_COUNT) {
      break;
    }
    scheduleEntries[index].enabled = item["enabled"] | false;
    scheduleEntries[index].onHour = constrain(item["on_hour"] | scheduleEntries[index].onHour, 0, 23);
    scheduleEntries[index].onMinute = constrain(item["on_minute"] | scheduleEntries[index].onMinute, 0, 59);
    scheduleEntries[index].offHour = constrain(item["off_hour"] | scheduleEntries[index].offHour, 0, 23);
    scheduleEntries[index].offMinute = constrain(item["off_minute"] | scheduleEntries[index].offMinute, 0, 59);
    scheduleEntries[index].lastTriggeredDayOfYearOn = -1;
    scheduleEntries[index].lastTriggeredDayOfYearOff = -1;
    index++;
  }

  if (saveScheduleToPreferences()) {
    pendingPublishStatus = true;
    Serial.println("[SCHEDULE] Schedule updated from MQTT");
    return true;
  }

  Serial.println("[SCHEDULE] Failed to save schedule from MQTT");
  return true;
}

bool parseTimeCommand(const JsonDocument& doc) {
  if (!doc.containsKey("time")) {
    return false;
  }

  const char* timeText = doc["time"];
  if (!timeText) {
    return false;
  }

  int hour = 0;
  int minute = 0;
  if (sscanf(timeText, "%d:%d", &hour, &minute) != 2) {
    return false;
  }

  hour = constrain(hour, 0, 23);
  minute = constrain(minute, 0, 59);

  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  timeinfo.tm_hour = hour;
  timeinfo.tm_min = minute;
  timeinfo.tm_sec = 0;
  time_t setTime = mktime(&timeinfo);
  struct timeval tv = { setTime, 0 };
  settimeofday(&tv, NULL);
  pendingPublishStatus = true;
  Serial.printf("[TIME] Set clock to %02d:%02d from MQTT\n", hour, minute);
  return true;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr(topic);
  String msg;
  msg.reserve(length);

  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  DynamicJsonDocument doc(512);
  DeserializationError err = deserializeJson(doc, msg);
  if (!err) {
    bool parsed = false;
    parsed |= parseScheduleCommand(doc);
    parsed |= parseStarterSequenceCommand(doc);
    parsed |= parseTimeCommand(doc);
    if (parsed) {
      return;
    }
  }

  if (topicStr == topicControl || topicStr == topicAllControl) {
    handleControlMessage(msg);
  }
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("[WiFi] Connecting");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" connected");
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
    setupTime();
    return true;
  }

  Serial.println(" failed");
  return false;
}

void setupTime() {
  configTime(NTP_GMT_OFFSET_SEC, NTP_DAYLIGHT_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5000)) {
    char timeBuffer[64];
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.print("[TIME] NTP synced: ");
    Serial.println(timeBuffer);
  } else {
    Serial.println("[TIME] NTP sync failed");
  }
}

bool reconnectMqtt() {
  if (millis() - lastMqttReconnectAttempt < 5000) {
    return false;
  }

  lastMqttReconnectAttempt = millis();

  uint8_t mac[6];
  WiFi.macAddress(mac);
  String clientId = String(mqttClientPrefix) + "_" + String(MODULE_ID) + "_" +
                    String(mac[4], HEX) + String(mac[5], HEX) + "_" +
                    String(random(0xffff), HEX);

  Serial.print("[MQTT] Connecting...");
  if (!mqttClient.connect(clientId.c_str())) {
    Serial.print(" failed, state=");
    Serial.println(mqttClient.state());
    return false;
  }

  Serial.println(" connected");

  mqttClient.subscribe(topicControl.c_str(), 1);
  mqttClient.subscribe(topicAllControl.c_str(), 1);

  publishStatus(true);
  publishTelemetry();

  Serial.println("[MQTT] Subscriptions ready");
  Serial.println("  Control: " + topicControl);
  Serial.println("  Group  : " + topicAllControl);
  Serial.println("  Status : " + topicStatus);
  Serial.println("  Data   : " + topicTelemetry);
  return true;
}

void setupTopics() {
  moduleId4 = formatMotorId4(MODULE_ID);
  topicControl = "relite/motor/" + moduleId4 + "/control";
  topicStatus = "relite/motor/" + moduleId4 + "/status";
  topicTelemetry = "relite/motor/" + moduleId4 + "/telemetry";
  topicAllControl = "relite/motor/all/control";
  topicAllData = "relite/sensors/all";
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("====================================");
  Serial.println(" ESP Adhana Motor Updated");
  Serial.println(" One motor per board on GPIO12");
  Serial.println("====================================");

  setupTopics();
  bootMs = millis();

  pinMode(RELAY_PIN_12, OUTPUT);
  pinMode(RELAY_PIN_14, OUTPUT);
  pinMode(RELAY_PIN_27, OUTPUT);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLDOWN);
  pinMode(MODE_LED_PIN, OUTPUT);
  pinMode(SENSOR_SEL_A_PIN, OUTPUT);
  pinMode(SENSOR_SEL_B_PIN, OUTPUT);
  digitalWrite(SENSOR_SEL_A_PIN, LOW);
  digitalWrite(SENSOR_SEL_B_PIN, LOW);

  autoModeEnabled = readHardwareAutoMode();
  lastHardwareAutoMode = autoModeEnabled;
  modeCommandOverrideActive = false;
  updateModeIndicator();

  loadScheduleFromPreferences();
  loadStarterSequenceFromPreferences();
  loadModeOverrideFromPreferences();

  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_X_LOGICAL_PIN, ADC_11db);
  analogSetPinAttenuation(SENSOR_Y_LOGICAL_PIN, ADC_11db);

  stopStarter();
  resetModeDecisionWindow();

  Serial.print("[Node] MODULE_ID: ");
  Serial.println(MODULE_ID);
  Serial.print("[Node] Starter relays GPIO: ");
  Serial.print(RELAY_PIN_12);
  Serial.print(", ");
  Serial.print(RELAY_PIN_14);
  Serial.print(", ");
  Serial.println(RELAY_PIN_27);
  Serial.print("[Node] CD4052 A/B: GPIO");
  Serial.print(SENSOR_SEL_A_PIN);
  Serial.print("/");
  Serial.println(SENSOR_SEL_B_PIN);
  Serial.print("[Node] CD4052 X/Y: GPIO");
  Serial.print(SENSOR_X_LOGICAL_PIN);
  Serial.print("/");
  Serial.println(SENSOR_Y_LOGICAL_PIN);
  Serial.println("[Node] Current input: CD4052 Y3 via SENSOR_Y_LOGICAL_PIN");
  Serial.print("[Node] Mode LED GPIO: ");
  Serial.print(MODE_LED_PIN);
  Serial.print(" (ON means ");
  Serial.print(MODE_LED_ON_IN_AUTO ? "AUTO" : "MANUAL");
  Serial.println(")");
    if (SENSOR_X_LOGICAL_PIN == 25 || SENSOR_X_LOGICAL_PIN == 26 || SENSOR_X_LOGICAL_PIN == 27 ||
      SENSOR_X_LOGICAL_PIN == 14 || SENSOR_X_LOGICAL_PIN == 13 || SENSOR_X_LOGICAL_PIN == 12 ||
      SENSOR_X_LOGICAL_PIN == 15 || SENSOR_X_LOGICAL_PIN == 2 || SENSOR_X_LOGICAL_PIN == 4 || SENSOR_X_LOGICAL_PIN == 0) {
    Serial.println("[WARN] SENSOR X logical pin is ADC2. ESP32 ADC2 reads are unreliable while WiFi is active.");
    Serial.println("[WARN] Keep X on ADC1 pin (GPIO32-39) for stable values with MQTT/WiFi.");
  }

  randomSeed(micros());

  if (ADC2_TEST_MODE) {
    Serial.println("[MODE] ADC2_TEST_MODE enabled. WiFi/MQTT disabled for stable ADC2 reads.");
    return;
  }

  connectWiFi();

  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(15);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);

  reconnectMqtt();
}

void loop() {
  if (ADC2_TEST_MODE) {
    const unsigned long now = millis();
    if (now - lastCd4052PrintMs >= CD4052_TEST_PRINT_MS) {
      lastCd4052PrintMs = now;
      printCd4052AllValues();
    }
    delay(20);
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  updateModeDecisionWindow();
  syncHardwareMode();

  if (!mqttClient.connected()) {
    reconnectMqtt();
  } else {
    mqttClient.loop();
    if (pendingPublishStatus) {
      pendingPublishStatus = false;
      publishStatus(true);
    }
  }

  applyX1FaultSafety();
  applyScheduleLogic();
  applyAutoModeLogic();

  updateStarterRelays();

  const unsigned long now = millis();

  if (now - lastCd4052PrintMs >= CD4052_TEST_PRINT_MS) {
    lastCd4052PrintMs = now;
    printCd4052AllValues();
  }

  if (mqttClient.connected() && (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)) {
    lastTelemetryMs = now;
    publishTelemetry();
  }

  if (mqttClient.connected() && (now - lastHeartbeatMs >= STATUS_HEARTBEAT_MS)) {
    lastHeartbeatMs = now;
    publishStatus(true);
  }

  delay(20);
}

