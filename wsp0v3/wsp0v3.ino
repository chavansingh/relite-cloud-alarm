#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h> 
#include <time.h>
#include <Wire.h>
#include <RTClib.h>

#define USE_BLUETOOTH_TIME_DISPLAY 0

#if USE_BLUETOOTH_TIME_DISPLAY
#include <BluetoothSerial.h>
#endif

// ====================================
// ESP Adhana Motor Updated Firmware
// One firmware image for all modules.
// Set MODULE_ID (1..30) uniquely per board.
// Each module controls only one motor relay on GPIO12.
// ====================================

// WiFi profile 1 (fixed in firmware)
const char* WIFI_FIXED_SSID = "Airtel-Hotspot-21A0";
const char* WIFI_FIXED_PASSWORD = "3ay7tn96";

// MQTT
const char* mqttBroker = "broker.hivemq.com";
const int mqttPort = 1883;
const char* mqttClientPrefix = "esp_adhana_motor_updated_node";

// Node identity: change per board (1..30)
const int MODULE_ID = 9;


// AUTO is dashboard-controlled only; hardware sensor values do not switch mode.
// The D13 pin is no longer used for auto/manual mode selection.
const int MODE_LED_PIN = 15; // D15 mode indicator LED output
const bool MODE_LED_ON_IN_AUTO = true;
const int NET_LED_PIN = 2; // D2 network status LED output
const bool NET_LED_ON_WHEN_CONNECTED = true;

// Starter relays: D12 and D14 are timed; D27 stays ON until OFF command.
const int RELAY_PIN_12 = 12;
const int RELAY_PIN_14 = 14;
const int RELAY_PIN_27 = 27;
const bool RELAY_ACTIVE_HIGH = true;
const unsigned long STARTER_RELAY12_MS = 30000;
const unsigned long STARTER_D12_TO_D14_DELAY_MS = 0;
const unsigned long STARTER_RELAY14_MS = 30000;

// CD4052 dual 4-channel analog multiplexer wiring:
// - X and Y (common pins) go to ESP analog inputs GPIO34/GPIO35
// - X0..X3 and Y0..Y3 are switched as channel pairs by A/B select lines
// - A/B select -> GPIO33/GPIO32
const int SENSOR_SEL_A_PIN = 33;
const int SENSOR_SEL_B_PIN = 32;
const int SENSOR_X_PIN = 34; // CD4052 X common -> raw/diagnostic input (ADC1 pin on ESP32)
const int SENSOR_Y_PIN = 35; // CD4052 Y common -> measurement input (voltage on Y0/Y1/Y2, current on Y3)
const int CURRENT_Y_CHANNEL = 3; // current sensor input from CD4052 Y3
const int VOLTAGE_R_Y_CHANNEL = 0; // R-phase voltage from Y0
const int VOLTAGE_Y_Y_CHANNEL = 1; // Y-phase voltage from Y1
const int VOLTAGE_B_Y_CHANNEL = 2; // B-phase voltage from Y2
const int X1_FAULT_THRESHOLD = 2500;
const int X1_PHASE_OK_THRESHOLD = 1700;
const int X0_POWER_SUPPLY_CUT_THRESHOLD = 1700;
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
// Phase-wise voltage calibration using field measurements:
// Actual 0V -> measured offsets differ per phase.
// Actual 250V -> measured span differs per phase.
const float VOLTAGE_R_MEASURED_AT_0V = 0.0f;
const float VOLTAGE_R_MEASURED_AT_250V = 165.0f;
const float VOLTAGE_Y_MEASURED_AT_0V = 0.0f;
const float VOLTAGE_Y_MEASURED_AT_250V = 167.0f;
const float VOLTAGE_B_MEASURED_AT_0V = 0.0f;
const float VOLTAGE_B_MEASURED_AT_250V = 167.0f;
const float VOLTAGE_CAL_ACTUAL_SPAN_V = 250.0f;
const float MOTOR_ON_CURRENT_THRESHOLD_A = 1.0f;
const float CURRENT_OFFSET_A = 0.0f;
const float CURRENT_GAIN = 10.0f;
// Current calibration using field measurements:
// shown 0A -> actual 0A, shown 7A -> actual 10A, shown 164A -> actual 50A.
const float CURRENT_MEASURED_AT_0A = 0.0f;
const float CURRENT_MEASURED_AT_10A = 7.0f;
const float CURRENT_MEASURED_AT_50A = 164.0f;
const float LOW_VOLTAGE_THRESHOLD_DEFAULT_V = 150.0f;
const float UNBALANCE_VOLTAGE_THRESHOLD_DEFAULT_PCT = 15.0f;
const float DRY_RUN_SET_DEFAULT_A = 1.0f;
const float OVERLOAD_SET_DEFAULT_A = 12.0f;

// Telemetry defaults
const float POWER_FACTOR = 0.98f;

// Publish intervals
const unsigned long TELEMETRY_INTERVAL_MS = 5000;
const unsigned long STATUS_HEARTBEAT_MS = 15000;
const unsigned long CD4052_TEST_PRINT_MS = 3000;
const unsigned long AUTO_START_DELAY_MS = 30000;
const unsigned long AUTO_RESTART_AFTER_CLEAR_MS = 20000;
const unsigned long SCHEDULE_CHECK_INTERVAL_MS = 1000;
const unsigned long ALARM_STATE_PERSIST_INTERVAL_MS = 60000;
const unsigned long NTP_RESYNC_INTERVAL_MS = 3600000; // 1 hour
const unsigned long NTP_RETRY_WHEN_UNSYNCED_MS = 30000;
const unsigned long BLUETOOTH_TIME_BROADCAST_MS = 10000;
const unsigned long SERIAL_TIME_PRINT_MS = 10000;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 10000;
const unsigned long WIFI_RECONFIG_GUARD_MS = 1200;
const unsigned long RTC_SNAPSHOT_SAVE_MS = 300000; // 5 minutes
const unsigned long DS1307_SYNC_INTERVAL_MS = 60000;
const unsigned long DS1307_RESTORE_INTERVAL_MS = 60000;
const unsigned long PHASE_WINDOW_VOLTAGE_MS = 30000;
const unsigned long PHASE_WINDOW_WAVEFORM_MS = 30000;
const unsigned long PHASE_WINDOW_TOTAL_MS = PHASE_WINDOW_VOLTAGE_MS + PHASE_WINDOW_WAVEFORM_MS;
const unsigned long PHASE_SAMPLE_INTERVAL_MS = 4;
const unsigned long DRY_RUN_STARTUP_IGNORE_MS = 30000; // 30 seconds
const unsigned long OVERLOAD_STARTUP_IGNORE_MS = 35000; // 35 seconds
const unsigned long PROTECTION_FAULT_TRIP_DELAY_MS = 10000;
const unsigned long OFF_CURRENT_SETTLE_MS = 30000; // 30 seconds after D27 OFF
const float OFF_CURRENT_ZERO_THRESHOLD_A = 0.25f;
const float MANUAL_TIMER_COUNT_CURRENT_THRESHOLD_A = 2.0f;
const unsigned long MANUAL_TIMER_PERSIST_INTERVAL_MS = 60000;
const int SCHEDULE_COUNT = 3;
const int DS1307_SDA_PIN = 21;
const int DS1307_SCL_PIN = 22;
const long NTP_GMT_OFFSET_SEC = 19800; // UTC+5:30
const int NTP_DAYLIGHT_OFFSET_SEC = 0;
const char* NTP_SERVER1 = "pool.ntp.org";
const char* NTP_SERVER2 = "time.nist.gov";
const size_t STATUS_JSON_CAPACITY = 3072;
const size_t TELEMETRY_JSON_CAPACITY = 1280;
const uint16_t MQTT_BUFFER_SIZE = 4096;
const int SWITCH_HISTORY_MAX = 8;

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
unsigned long lastSerialTimePrintMs = 0;
unsigned long lastWiFiConnectAttemptMs = 0;
unsigned long lastRtcSnapshotSaveMs = 0;
unsigned long lastDs1307SyncMs = 0;
unsigned long lastDs1307RestoreMs = 0;
#if USE_BLUETOOTH_TIME_DISPLAY
unsigned long lastBluetoothTimeMs = 0;
#endif

bool relayStateOn = false;
bool pendingPublishStatus = false;
bool autoAlarmLatched = false;
bool autoRestartAfterFault = false;
unsigned long autoRestartClearSinceMs = 0;
String autoRestartFaultReason = "";
bool alarmWindowActive = false;
bool alarmWindowStartPending = false;
bool alarmWindowManualOffUntilEnd = false;
unsigned long alarmWindowCompensationMs = 0;
unsigned long alarmCompensationLastTickMs = 0;
unsigned long lastAlarmStatePersistMs = 0;
uint32_t alarmWindowLastEpoch = 0;
bool alarmWindowRecoveryPending = false;
String pendingAlarmStateReason = "";
bool dryRunAutoRestartUsed = false;
bool overloadAutoRestartUsed = false;
bool dryRunAutoRestartLocked = false;
bool overloadAutoRestartLocked = false;
bool starterSequenceActive = false;
unsigned long starterSequenceStartMs = 0;
bool lastVoltageOk = true;
bool autoModeEnabled = false;
bool modeCommandOverrideActive = false;
bool lastHardwareAutoMode = true;
bool lastWiFiConnected = false;
bool ds1307Available = false;
String currentTimeSource = "UNSYNCED";
unsigned long phaseWindowStartMs = 0;
unsigned long lastPhaseSampleMs = 0;
float phaseVoltageSumR = 0.0f;
float phaseVoltageSumY = 0.0f;
float phaseVoltageSumB = 0.0f;
unsigned long phaseVoltageSamples = 0;
long phaseWaveIntegralR = 0;
long phaseWaveIntegralY = 0;
long phaseWaveIntegralB = 0;
unsigned long phaseWaveSamples = 0;
String phaseSequenceEstimate = "UNKNOWN";
long phaseSequenceConfidence = 0;
String phaseSequenceSet = "RYB";
unsigned long offCurrentMonitorStartMs = 0;
bool offCurrentTripRaised = false;
unsigned long bootMs = 0;
unsigned long lastScheduleCheckMs = 0;
unsigned long lastNtpSyncAttemptMs = 0;
bool rtcTimeValid = false;
float lowVoltageThresholdV = LOW_VOLTAGE_THRESHOLD_DEFAULT_V;
float unbalanceVoltageThresholdPct = UNBALANCE_VOLTAGE_THRESHOLD_DEFAULT_PCT;
float dryRunSetA = DRY_RUN_SET_DEFAULT_A;
float overloadSetA = OVERLOAD_SET_DEFAULT_A;
bool voltageProtectionEnabled = true;
bool currentProtectionEnabled = true;
bool ignoredFaultLowVoltage = false;
bool ignoredFaultUnbalanceVoltage = false;
bool ignoredFaultCurrent = false;
bool ignoredFaultPhase = false;
bool manualTimedOnActive = false;
uint32_t manualTimedOnEndEpoch = 0;
uint32_t manualTimedOnDurationSec = 0;
uint32_t manualTimedOnRemainingSec = 0;
unsigned long manualTimerLastTickMs = 0;
unsigned long manualTimerLastPersistMs = 0;
bool manualTimerPausedByCurrent = false;
bool manualTimerFaultRestartPending = false;
unsigned long manualTimerFaultClearSinceMs = 0;
String manualTimerFaultReason = "";
unsigned long motorLastStartMs = 0;
String pendingProtectionFaultReason = "";
unsigned long pendingProtectionFaultSinceMs = 0;

struct SwitchHistoryEntry {
  bool valid;
  bool on;
  uint32_t epoch;
  String reason;
};

SwitchHistoryEntry switchHistory[SWITCH_HISTORY_MAX];
bool switchHistoryStateKnown = false;
bool switchHistoryLastState = false;
String pendingSwitchEventReason = "";

const unsigned long MODE_DECISION_WINDOW_MS = 30000;
const unsigned long MODE_SAMPLE_INTERVAL_MS = 1000;
const uint8_t MODE_TOGGLE_CONFIRM_WINDOWS = 2;
unsigned long lastModeSampleMs = 0;
unsigned long modeWindowStartMs = 0;
long modeDecisionSum = 0;
int modeDecisionCount = 0;
int modeDecisionRaw = 0;
bool modeDecisionReady = false;
bool modeDecisionHardwareAutoMode = true;
bool x1DecisionVoltageOk = true;
unsigned long modeDecisionVersion = 0;
unsigned long lastHandledModeDecisionVersion = 0;
bool pendingHardwareModeChangeActive = false;
bool pendingHardwareModeCandidate = true;
uint8_t pendingHardwareModeConfirmCount = 0;

#if USE_BLUETOOTH_TIME_DISPLAY
BluetoothSerial SerialBT;
#endif

RTC_DS1307 externalRtc;

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
Preferences rtcPreferences;

void syncDs1307FromSystemTime(bool forceSync);
void applyManualModeTransition(const String& stopReason);
void startStarterOnSequenceWithReason(const String& reason);
void stopStarterWithReason(const String& reason);
float readCurrentFromSensor(int* rawOut = nullptr, float* adcVoltageOut = nullptr);

unsigned long starterRelay12Ms = STARTER_RELAY12_MS;
unsigned long starterD12ToD14DelayMs = STARTER_D12_TO_D14_DELAY_MS;
unsigned long starterRelay14Ms = STARTER_RELAY14_MS;

uint32_t getManualTimedOnDurationFromMinutes(uint32_t minutes) {
  if (minutes == 10 || minutes == 20 || minutes == 40 || minutes == 60 || minutes == 120 || minutes == 180) {
    return minutes * 60UL;
  }
  return 0;
}

uint16_t getScheduleDurationMinutes(const ScheduleEntry& entry) {
  const int onTotal = ((int)entry.onHour * 60) + (int)entry.onMinute;
  const int offTotal = ((int)entry.offHour * 60) + (int)entry.offMinute;
  int diff = (offTotal - onTotal + (24 * 60)) % (24 * 60);
  if (diff <= 0) {
    diff = 60;
  }
  return (uint16_t)diff;
}

void applyScheduleDurationToEntry(ScheduleEntry& entry, uint16_t durationMinutes) {
  if (durationMinutes == 0) {
    durationMinutes = 60;
  }
  const int onTotal = ((int)entry.onHour * 60) + (int)entry.onMinute;
  const int offTotal = (onTotal + (int)durationMinutes) % (24 * 60);
  entry.offHour = (uint8_t)(offTotal / 60);
  entry.offMinute = (uint8_t)(offTotal % 60);
}

void saveManualTimedOnToPreferences() {
  controlPreferences.begin("control", false);
  controlPreferences.putBool("mt_on", manualTimedOnActive);
  controlPreferences.putULong("mt_end", manualTimedOnEndEpoch);
  controlPreferences.putULong("mt_dur", manualTimedOnDurationSec);
  controlPreferences.putULong("mt_rem", manualTimedOnRemainingSec);
  controlPreferences.end();
}

uint32_t getManualTimedOnRemainingSec() {
  if (!manualTimedOnActive) {
    return 0;
  }
  return manualTimedOnRemainingSec;
}

void clearManualTimedOnState(bool save = true) {
  manualTimedOnActive = false;
  manualTimedOnEndEpoch = 0;
  manualTimedOnDurationSec = 0;
  manualTimedOnRemainingSec = 0;
  manualTimerLastTickMs = 0;
  manualTimerPausedByCurrent = false;
  manualTimerFaultRestartPending = false;
  manualTimerFaultClearSinceMs = 0;
  manualTimerFaultReason = "";
  if (save) {
    saveManualTimedOnToPreferences();
  }
}

void loadManualTimedOnFromPreferences() {
  controlPreferences.begin("control", true);
  manualTimedOnActive = controlPreferences.getBool("mt_on", false);
  manualTimedOnEndEpoch = controlPreferences.getULong("mt_end", 0);
  manualTimedOnDurationSec = controlPreferences.getULong("mt_dur", 0);
  manualTimedOnRemainingSec = controlPreferences.getULong("mt_rem", 0);
  controlPreferences.end();

  if (!manualTimedOnActive) {
    manualTimedOnEndEpoch = 0;
    manualTimedOnDurationSec = 0;
    manualTimedOnRemainingSec = 0;
    return;
  }

  const time_t nowEpoch = time(nullptr);
  if (manualTimedOnRemainingSec == 0) {
    if (nowEpoch > 0 && manualTimedOnEndEpoch > 0 && (uint32_t)nowEpoch < manualTimedOnEndEpoch) {
      manualTimedOnRemainingSec = manualTimedOnEndEpoch - (uint32_t)nowEpoch;
    } else {
      manualTimedOnRemainingSec = manualTimedOnDurationSec;
    }
  }

  if (manualTimedOnRemainingSec == 0) {
    clearManualTimedOnState(true);
    return;
  }

  if (nowEpoch > 0) {
    manualTimedOnEndEpoch = (uint32_t)nowEpoch + manualTimedOnRemainingSec;
  } else {
    manualTimedOnEndEpoch = 0;
  }

  manualTimerLastTickMs = millis();
  manualTimerPausedByCurrent = true;
  manualTimerFaultRestartPending = false;
  manualTimerFaultClearSinceMs = 0;
  manualTimerFaultReason = "";
  manualTimerLastPersistMs = millis();
}

bool parseManualTimedDurationToken(const String& token, uint32_t& durationSec) {
  String t = token;
  t.trim();
  t.toUpperCase();
  t.replace(" ", "");

  if (t == "10" || t == "10M" || t == "10MIN" || t == "10MINUTE" || t == "10MINUTES") {
    durationSec = 10UL * 60UL;
    return true;
  }
  if (t == "20" || t == "20M" || t == "20MIN" || t == "20MINUTE" || t == "20MINUTES") {
    durationSec = 20UL * 60UL;
    return true;
  }
  if (t == "40" || t == "40M" || t == "40MIN" || t == "40MINUTE" || t == "40MINUTES") {
    durationSec = 40UL * 60UL;
    return true;
  }
  if (t == "1H" || t == "1HR" || t == "1HOUR") {
    durationSec = 60UL * 60UL;
    return true;
  }
  if (t == "2H" || t == "2HR" || t == "2HOUR") {
    durationSec = 120UL * 60UL;
    return true;
  }
  if (t == "3H" || t == "3HR" || t == "3HOUR") {
    durationSec = 180UL * 60UL;
    return true;
  }

  return false;
}

bool parseManualTimedCommandText(const String& msg, uint32_t& durationSec) {
  String value = msg;
  value.trim();
  value.toUpperCase();
  value.replace(" ", "");

  const char* prefixes[] = {
    "MANUAL_ON_",
    "MANUALON_",
    "MANUAL_TIMER_",
    "MANUALTIME_"
  };

  for (size_t i = 0; i < (sizeof(prefixes) / sizeof(prefixes[0])); i++) {
    const String prefix = String(prefixes[i]);
    if (value.startsWith(prefix)) {
      return parseManualTimedDurationToken(value.substring(prefix.length()), durationSec);
    }
  }

  return false;
}

bool startManualTimedOn(uint32_t durationSec, const String& reason) {
  if (!(durationSec == 600UL || durationSec == 1200UL || durationSec == 2400UL ||
        durationSec == 3600UL || durationSec == 7200UL || durationSec == 10800UL)) {
    return false;
  }

  modeCommandOverrideActive = true;
  if (autoModeEnabled) {
    applyManualModeTransition("AUTO_OFF");
  }
  autoModeEnabled = false;
  updateModeIndicator();

  manualTimedOnActive = true;
  manualTimedOnDurationSec = durationSec;
  manualTimedOnRemainingSec = durationSec;
  const time_t nowEpoch = time(nullptr);
  if (nowEpoch > 0) {
    manualTimedOnEndEpoch = (uint32_t)nowEpoch + manualTimedOnRemainingSec;
  } else {
    manualTimedOnEndEpoch = 0;
  }
  manualTimerLastTickMs = millis();
  manualTimerLastPersistMs = millis();
  manualTimerPausedByCurrent = true;
  manualTimerFaultRestartPending = false;
  manualTimerFaultClearSinceMs = 0;
  manualTimerFaultReason = "";
  saveModeOverrideToPreferences();
  saveManualTimedOnToPreferences();

  if (!relayStateOn) {
    startStarterOnSequenceWithReason(reason);
  }

  pendingProtectionFaultReason = "";
  pendingProtectionFaultSinceMs = 0;
  pendingPublishStatus = true;

  Serial.print("[MANUAL TIMER] ON for ");
  Serial.print((unsigned long)(durationSec / 60UL));
  Serial.println(" minutes");
  return true;
}

void cancelManualTimedOn(bool stopMotor, const String& reason) {
  if (stopMotor && relayStateOn) {
    stopStarterWithReason(reason);
  }
  clearManualTimedOnState(true);
  pendingProtectionFaultReason = "";
  pendingProtectionFaultSinceMs = 0;
  pendingPublishStatus = true;
}

void applyManualTimedOnLogic() {
  if (!manualTimedOnActive) {
    return;
  }

  const unsigned long nowMs = millis();
  if (manualTimerLastTickMs == 0) {
    manualTimerLastTickMs = nowMs;
  }

  if (!relayStateOn) {
    if (!manualTimerFaultRestartPending) {
      manualTimerLastTickMs = nowMs;
      return;
    }

    String blockReason = "";
    if (isAutoStartBlockedByProtection(blockReason)) {
      manualTimerFaultClearSinceMs = 0;
      return;
    }

    if (isCurrentTripReason(manualTimerFaultReason) && !isCurrentFaultClearForAutoRestart(manualTimerFaultReason)) {
      manualTimerFaultClearSinceMs = 0;
      return;
    }

    if (manualTimerFaultClearSinceMs == 0) {
      manualTimerFaultClearSinceMs = nowMs;
      Serial.print("[MANUAL TIMER] Fault cleared, waiting ");
      Serial.print(AUTO_RESTART_AFTER_CLEAR_MS / 1000);
      Serial.println("s before restart");
      return;
    }

    if (nowMs - manualTimerFaultClearSinceMs < AUTO_RESTART_AFTER_CLEAR_MS) {
      return;
    }

    startStarterOnSequenceWithReason("MANUAL_TIMER_RESTART");
    manualTimerFaultRestartPending = false;
    manualTimerFaultClearSinceMs = 0;
    manualTimerFaultReason = "";
    manualTimerPausedByCurrent = true;
    manualTimerLastTickMs = millis();
    pendingPublishStatus = true;
    Serial.println("[MANUAL TIMER] Restarted after fault clear");
    return;
  }

  float currentA = readCurrentFromSensor();
  if (currentA < 0.25f) {
    currentA = 0.0f;
  }

  if (currentA <= MANUAL_TIMER_COUNT_CURRENT_THRESHOLD_A) {
    manualTimerPausedByCurrent = true;
    manualTimerLastTickMs = nowMs;
    return;
  }

  manualTimerPausedByCurrent = false;
  const unsigned long elapsedMs = nowMs - manualTimerLastTickMs;
  if (elapsedMs < 1000) {
    return;
  }

  uint32_t elapsedSec = (uint32_t)(elapsedMs / 1000UL);
  if (elapsedSec > manualTimedOnRemainingSec) {
    elapsedSec = manualTimedOnRemainingSec;
  }
  manualTimedOnRemainingSec -= elapsedSec;
  manualTimerLastTickMs += elapsedSec * 1000UL;

  const time_t nowEpoch = time(nullptr);
  if (nowEpoch > 0) {
    manualTimedOnEndEpoch = (uint32_t)nowEpoch + manualTimedOnRemainingSec;
  } else {
    manualTimedOnEndEpoch = 0;
  }

  if (nowMs - manualTimerLastPersistMs >= MANUAL_TIMER_PERSIST_INTERVAL_MS || manualTimedOnRemainingSec == 0) {
    saveManualTimedOnToPreferences();
    manualTimerLastPersistMs = nowMs;
  }

  if (manualTimedOnRemainingSec > 0) {
    return;
  }

  Serial.println("[MANUAL TIMER] Time completed, switching OFF");
  cancelManualTimedOn(true, "MANUAL_TIMER_OFF");
}

void resetSwitchHistory() {
  for (int i = 0; i < SWITCH_HISTORY_MAX; i++) {
    switchHistory[i].valid = false;
    switchHistory[i].on = false;
    switchHistory[i].epoch = 0;
    switchHistory[i].reason = "";
  }
}

void saveSwitchHistoryToPreferences() {
  StaticJsonDocument<1024> doc;
  JsonArray arr = doc.createNestedArray("events");

  for (int i = 0; i < SWITCH_HISTORY_MAX; i++) {
    if (!switchHistory[i].valid) {
      continue;
    }
    JsonObject item = arr.createNestedObject();
    item["state"] = switchHistory[i].on ? "ON" : "OFF";
    item["time_unix"] = switchHistory[i].epoch;
    if (switchHistory[i].reason.length() > 0) {
      item["reason"] = switchHistory[i].reason;
    }
  }

  char buffer[1024];
  const size_t len = serializeJson(doc, buffer, sizeof(buffer));
  if (len == 0 || len >= sizeof(buffer) - 1) {
    return;
  }

  controlPreferences.begin("control", false);
  controlPreferences.putString("sw_hist", buffer);
  controlPreferences.end();
}

void loadSwitchHistoryFromPreferences() {
  resetSwitchHistory();

  controlPreferences.begin("control", true);
  const String stored = controlPreferences.getString("sw_hist", "");
  controlPreferences.end();

  if (stored.length() == 0) {
    return;
  }

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, stored);
  if (err || !doc.containsKey("events")) {
    return;
  }

  JsonArray arr = doc["events"].as<JsonArray>();
  int index = 0;
  for (JsonVariant v : arr) {
    if (index >= SWITCH_HISTORY_MAX) {
      break;
    }
    const char* state = v["state"] | "";
    const uint32_t epoch = v["time_unix"] | 0;
    const char* reason = v["reason"] | "";
    if (epoch == 0) {
      continue;
    }
    String s = String(state);
    s.toUpperCase();
    switchHistory[index].valid = true;
    switchHistory[index].on = (s == "ON" || s == "1");
    switchHistory[index].epoch = epoch;
    switchHistory[index].reason = String(reason);
    index++;
  }

  if (switchHistory[0].valid) {
    switchHistoryStateKnown = true;
    switchHistoryLastState = switchHistory[0].on;
  }
}

void pushSwitchHistoryEvent(bool isOn, time_t nowEpoch, const String& reason = "") {
  if (nowEpoch <= 0) {
    nowEpoch = time(nullptr);
  }
  if (nowEpoch <= 0) {
    nowEpoch = millis() / 1000;
  }

  if (switchHistory[0].valid && switchHistory[0].on == isOn) {
    return;
  }

  for (int i = SWITCH_HISTORY_MAX - 1; i > 0; i--) {
    switchHistory[i] = switchHistory[i - 1];
  }

  switchHistory[0].valid = true;
  switchHistory[0].on = isOn;
  switchHistory[0].epoch = (uint32_t)nowEpoch;
  switchHistory[0].reason = reason;

  switchHistoryStateKnown = true;
  switchHistoryLastState = isOn;
  saveSwitchHistoryToPreferences();
}

void updateSwitchHistoryFromState(bool isOn, time_t nowEpoch, const String& reason = "") {
  if (!switchHistoryStateKnown) {
    switchHistoryStateKnown = true;
    switchHistoryLastState = isOn;
    if (!switchHistory[0].valid) {
      pushSwitchHistoryEvent(isOn, nowEpoch, reason);
    }
    return;
  }

  if (isOn == switchHistoryLastState) {
    return;
  }

  switchHistoryLastState = isOn;
  pushSwitchHistoryEvent(isOn, nowEpoch, reason);
}

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

  // Keep the original voltage conversion first, then apply phase correction
  // using your measured displayed readings.
  float legacyVoltage = 0.0f;
  if (sensedVoltage <= VOLTAGE_CAL_LOW_MEASURED) {
    legacyVoltage = sensedVoltage * (VOLTAGE_CAL_LOW_ACTUAL / VOLTAGE_CAL_LOW_MEASURED) * VOLTAGE_OUTPUT_FACTOR;
  } else {
    legacyVoltage = (VOLTAGE_CAL_LOW_ACTUAL +
                    ((sensedVoltage - VOLTAGE_CAL_LOW_MEASURED) *
                    (VOLTAGE_CAL_HIGH_ACTUAL - VOLTAGE_CAL_LOW_ACTUAL) /
                    (VOLTAGE_CAL_HIGH_MEASURED - VOLTAGE_CAL_LOW_MEASURED))) * VOLTAGE_OUTPUT_FACTOR;
  }

  float measuredAt0 = VOLTAGE_Y_MEASURED_AT_0V;
  float measuredAt250 = VOLTAGE_Y_MEASURED_AT_250V;

  if (yChannel == VOLTAGE_R_Y_CHANNEL) {
    measuredAt0 = VOLTAGE_R_MEASURED_AT_0V;
    measuredAt250 = VOLTAGE_R_MEASURED_AT_250V;
  } else if (yChannel == VOLTAGE_B_Y_CHANNEL) {
    measuredAt0 = VOLTAGE_B_MEASURED_AT_0V;
    measuredAt250 = VOLTAGE_B_MEASURED_AT_250V;
  }

  const float measuredSpan = measuredAt250 - measuredAt0;
  if (measuredSpan < 1.0f) {
    return 0.0f;
  }

  float calibratedVoltage = (legacyVoltage - measuredAt0) * (VOLTAGE_CAL_ACTUAL_SPAN_V / measuredSpan);
  if (calibratedVoltage < 0.0f) {
    calibratedVoltage = 0.0f;
  }
  return calibratedVoltage;
}

float calibrateCurrentReading(float sensedCurrentA) {
  float measuredCurrentA = (sensedCurrentA - CURRENT_OFFSET_A) * CURRENT_GAIN;
  if (measuredCurrentA <= CURRENT_MEASURED_AT_0A) {
    return 0.0f;
  }

  float currentA = 0.0f;
  if (measuredCurrentA <= CURRENT_MEASURED_AT_10A) {
    currentA = measuredCurrentA * (10.0f / CURRENT_MEASURED_AT_10A);
  } else {
    const float spanMeasured = CURRENT_MEASURED_AT_50A - CURRENT_MEASURED_AT_10A;
    if (spanMeasured < 0.1f) {
      return 0.0f;
    }
    currentA = 10.0f + ((measuredCurrentA - CURRENT_MEASURED_AT_10A) * (40.0f / spanMeasured));
  }

  if (currentA < 0.0f) {
    currentA = 0.0f;
  }
  return currentA;
}

float readCurrentFromSensor(int* rawOut, float* adcVoltageOut) {
  selectSensorChannel(CURRENT_Y_CHANNEL);
  analogRead(SENSOR_Y_LOGICAL_PIN); // discard first sample after channel switch
  delayMicroseconds(200);
  const int raw = readAveragedAdc(SENSOR_Y_LOGICAL_PIN, 500);
  const float adcVoltage = (raw * ADC_REF_V) / ADC_MAX;
  if (rawOut != nullptr) {
    *rawOut = raw;
  }
  if (adcVoltageOut != nullptr) {
    *adcVoltageOut = adcVoltage;
  }
  return calibrateCurrentReading(adcVoltage * CURRENT_SCALE);
}

void resetPhaseWindow(unsigned long nowMs) {
  phaseWindowStartMs = nowMs;
  lastPhaseSampleMs = nowMs;
  phaseVoltageSumR = 0.0f;
  phaseVoltageSumY = 0.0f;
  phaseVoltageSumB = 0.0f;
  phaseVoltageSamples = 0;
  phaseWaveIntegralR = 0;
  phaseWaveIntegralY = 0;
  phaseWaveIntegralB = 0;
  phaseWaveSamples = 0;
}

void finalizePhaseWindow() {
  const long score = (phaseWaveIntegralR + phaseWaveIntegralY + phaseWaveIntegralB);
  phaseSequenceConfidence = labs(score);

  if (phaseWaveSamples < 200 || phaseSequenceConfidence < 200) {
    phaseSequenceEstimate = "UNKNOWN";
    return;
  }

  // Approximation: sign of summed phase-integral trend maps sequence direction.
  phaseSequenceEstimate = (score >= 0) ? "RYB" : "YBR";
}

String normalizePhaseSequenceLabel(const String& input, bool allowAuto = false) {
  String value = input;
  value.trim();
  value.toUpperCase();

  if (value == "RYB") {
    return "RYB";
  }
  if (value == "YBR" || value == "RBY") {
    return "YBR";
  }
  if (value == "BRY") {
    return "BRY";
  }
  if (value == "UNKNOWN" || value == "UNK" || value == "UNKNOW") {
    return "UNKNOWN";
  }
  return String("");
}

String getDisplayPhaseSequence() {
  return phaseSequenceSet;
}

String getPhaseSequenceSource() {
  return "DASHBOARD";
}

bool isPhaseSequenceStable() {
  return phaseSequenceEstimate != "UNKNOWN" && phaseSequenceConfidence >= 200;
}

void updatePhaseSequenceEstimator() {
  const unsigned long nowMs = millis();
  if (phaseWindowStartMs == 0) {
    resetPhaseWindow(nowMs);
  }

  if (nowMs - lastPhaseSampleMs < PHASE_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastPhaseSampleMs = nowMs;

  const unsigned long elapsed = nowMs - phaseWindowStartMs;

  if (elapsed < PHASE_WINDOW_VOLTAGE_MS) {
    phaseVoltageSumR += readVoltageFromYChannel(VOLTAGE_R_Y_CHANNEL);
    phaseVoltageSumY += readVoltageFromYChannel(VOLTAGE_Y_Y_CHANNEL);
    phaseVoltageSumB += readVoltageFromYChannel(VOLTAGE_B_Y_CHANNEL);
    phaseVoltageSamples++;
    return;
  }

  if (elapsed < PHASE_WINDOW_TOTAL_MS) {
    // Use raw waveform around ADC midpoint for approximate phase-order trend.
    selectSensorChannel(VOLTAGE_R_Y_CHANNEL);
    const int rRaw = readAveragedAdc(SENSOR_Y_LOGICAL_PIN, 2);
    selectSensorChannel(VOLTAGE_Y_Y_CHANNEL);
    const int yRaw = readAveragedAdc(SENSOR_Y_LOGICAL_PIN, 2);
    selectSensorChannel(VOLTAGE_B_Y_CHANNEL);
    const int bRaw = readAveragedAdc(SENSOR_Y_LOGICAL_PIN, 2);

    phaseWaveIntegralR += (long)(rRaw - 2048);
    phaseWaveIntegralY += (long)(yRaw - 2048);
    phaseWaveIntegralB += (long)(bRaw - 2048);
    phaseWaveSamples++;
    return;
  }

  finalizePhaseWindow();
  resetPhaseWindow(nowMs);
}

int readSensorChannelRaw(int channel, int samples = 16) {
  selectSensorChannel(channel);
  analogRead(SENSOR_X_LOGICAL_PIN); // discard first sample after channel switch
  delayMicroseconds(200);
  return readAveragedAdc(SENSOR_X_LOGICAL_PIN, samples);
}

bool isRunningFromCurrent(float currentA) {
  return currentA > MOTOR_ON_CURRENT_THRESHOLD_A;
}

bool isVoltageOkFromX1Raw(int x1Raw) {
  return x1Raw < X1_FAULT_THRESHOLD;
}

bool isPhaseOkFromX1Raw(int x1Raw) {
  return x1Raw < X1_PHASE_OK_THRESHOLD;
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
  modeDecisionHardwareAutoMode = false;
  x1DecisionVoltageOk = modeDecisionRaw < X1_FAULT_THRESHOLD;
  modeDecisionReady = true;
  modeDecisionVersion++;

  Serial.print("[MODE] 30s avg X0=");
  Serial.print(modeDecisionRaw);
  Serial.print(" mode=");
  Serial.print("IGNORED");
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
  // Hardware sensor values never drive AUTO/MANUAL mode.
  return false;
}

void updateModeIndicator() {
  const bool ledOn = MODE_LED_ON_IN_AUTO ? autoModeEnabled : !autoModeEnabled;
  digitalWrite(MODE_LED_PIN, ledOn ? HIGH : LOW);
}

void updateNetIndicator(bool wifiConnected) {
  const bool ledOn = NET_LED_ON_WHEN_CONNECTED ? wifiConnected : !wifiConnected;
  digitalWrite(NET_LED_PIN, ledOn ? HIGH : LOW);
}

bool isCurrentTripReason(const String& reason) {
  return reason == "DRY_RUN_TRIP" || reason == "OL_TRIP";
}

bool isTripRestartUsedForReason(const String& reason) {
  if (reason == "DRY_RUN_TRIP") {
    return dryRunAutoRestartUsed;
  }
  if (reason == "OL_TRIP") {
    return overloadAutoRestartUsed;
  }
  return false;
}

void markTripRestartUsedForReason(const String& reason) {
  if (reason == "DRY_RUN_TRIP") {
    dryRunAutoRestartUsed = true;
  } else if (reason == "OL_TRIP") {
    overloadAutoRestartUsed = true;
  }
}

void lockoutTripAutoRestartForReason(const String& reason) {
  if (reason == "DRY_RUN_TRIP") {
    dryRunAutoRestartLocked = true;
  } else if (reason == "OL_TRIP") {
    overloadAutoRestartLocked = true;
  }
}

bool isAnyCurrentTripAutoRestartLocked() {
  return dryRunAutoRestartLocked || overloadAutoRestartLocked;
}

bool isCurrentFaultClearForAutoRestart(const String& reason) {
  float currentA = readCurrentFromSensor();
  if (currentA < 0.25f) {
    currentA = 0.0f;
  }

  if (reason == "OL_TRIP") {
    return currentA < overloadSetA;
  }
  if (reason == "DRY_RUN_TRIP") {
    return currentA < dryRunSetA;
  }
  return true;
}

void applyManualModeTransition(const String& stopReason) {
  const bool wasAutoMode = autoModeEnabled;
  autoModeEnabled = false;
  autoAlarmLatched = false;
  autoRestartAfterFault = false;
  autoRestartClearSinceMs = 0;
  autoRestartFaultReason = "";
  alarmWindowStartPending = false;
  dryRunAutoRestartUsed = false;
  overloadAutoRestartUsed = false;
  dryRunAutoRestartLocked = false;
  overloadAutoRestartLocked = false;
  updateModeIndicator();

  if (wasAutoMode && relayStateOn) {
    stopStarterWithReason(stopReason);
  }

  pendingPublishStatus = true;
}

String getProtectionFaultReason(bool lowVoltageActive, bool unbalanceVoltageActive, bool dryRunActive, bool overloadActive, bool phaseFaultActive) {
  if (voltageProtectionEnabled && !ignoredFaultLowVoltage && lowVoltageActive) {
    return "LOW_VOLTAGE_TRIP";
  }
  if (voltageProtectionEnabled && !ignoredFaultUnbalanceVoltage && unbalanceVoltageActive) {
    return "UB_TRIP";
  }
  if (voltageProtectionEnabled && !ignoredFaultPhase && phaseFaultActive) {
    return "SEQ_TRIP";
  }
  if (currentProtectionEnabled && !ignoredFaultCurrent && dryRunActive) {
    return "DRY_RUN_TRIP";
  }
  if (currentProtectionEnabled && !ignoredFaultCurrent && overloadActive) {
    return "OL_TRIP";
  }
  return "";
}

void syncHardwareMode() {
  // AUTO/MANUAL mode is controlled only by dashboard commands.
  pendingHardwareModeChangeActive = false;
  pendingHardwareModeConfirmCount = 0;
  lastHandledModeDecisionVersion = modeDecisionVersion;
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
  return (value == "ON" || value == "1" || value == "HIGH" || value == "START" || value == "ALARM_ON" || value == "E_ALARM_ON" || value == "D_ALARM_ON");
}

bool parseOffCommand(const String& msg) {
  String value = msg;
  value.trim();
  value.toUpperCase();
  return (value == "OFF" || value == "0" || value == "LOW" || value == "STOP" || value == "ALARM_OFF" || value == "E_ALARM_OFF" || value == "D_ALARM_OFF");
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
  motorLastStartMs = starterSequenceStartMs;
  offCurrentMonitorStartMs = 0;
  offCurrentTripRaised = false;
  const String reason = pendingSwitchEventReason.length() > 0
    ? pendingSwitchEventReason
    : "SYSTEM_ON";
  pendingSwitchEventReason = "";
  pushSwitchHistoryEvent(true, time(nullptr), reason);
  updateStarterRelays();
}

void stopStarter() {
  relayStateOn = false;
  starterSequenceActive = false;
  offCurrentMonitorStartMs = millis();
  offCurrentTripRaised = false;
  const String reason = pendingSwitchEventReason.length() > 0
    ? pendingSwitchEventReason
    : "SYSTEM_OFF";
  pendingSwitchEventReason = "";
  pushSwitchHistoryEvent(false, time(nullptr), reason);
  updateStarterRelays();
}

void startStarterOnSequenceWithReason(const String& reason) {
  pendingSwitchEventReason = reason;
  if (reason == "E_ALARM_ON" || reason == "D_ALARM_ON") {
    pendingAlarmStateReason = "E_ALARM_ON";
  }
  startStarterOnSequence();
}

void stopStarterWithReason(const String& reason) {
  pendingSwitchEventReason = reason;
  if (reason == "E_ALARM_OFF" || reason == "D_ALARM_OFF") {
    pendingAlarmStateReason = "E_ALARM_OFF";
  }
  stopStarter();
}

String buildStatusText(bool autoMode, bool d27On, bool runningByCurrent) {
  String status = autoMode ? "AUTO " : "MANUAL ";
  // ON/OFF state is decided only by D27 feedback level.
  status += "STATE ";
  status += d27On ? "ON " : "OFF ";
  status += "D27 ";
  status += d27On ? "ON " : "OFF ";
  // Running condition is decided from measured motor current.
  status += "RUN ";
  status += runningByCurrent ? "STARTED" : "STOPPED";
  return status;
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
    entry["duration_min"] = getScheduleDurationMinutes(scheduleEntries[i]);
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

void loadStarterSequenceFromPreferences() {
  // Starter sequence is fixed in firmware; runtime updates are disabled.
  starterRelay12Ms = STARTER_RELAY12_MS;
  starterD12ToD14DelayMs = STARTER_D12_TO_D14_DELAY_MS;
  starterRelay14Ms = STARTER_RELAY14_MS;
}

void saveModeOverrideToPreferences() {
  controlPreferences.begin("control", false);
  controlPreferences.putBool("mode_override", modeCommandOverrideActive);
  controlPreferences.putBool("auto_mode", autoModeEnabled);
  controlPreferences.end();
}

void saveLowVoltageThresholdToPreferences() {
  controlPreferences.begin("control", false);
  controlPreferences.putFloat("low_v_thr", lowVoltageThresholdV);
  controlPreferences.end();
}

void saveCurrentThresholdsToPreferences() {
  controlPreferences.begin("control", false);
  controlPreferences.putFloat("dry_set_a", dryRunSetA);
  controlPreferences.putFloat("ovl_set_a", overloadSetA);
  controlPreferences.end();
}

void saveFaultConfigToPreferences() {
  controlPreferences.begin("control", false);
  controlPreferences.putFloat("unbal_pct", unbalanceVoltageThresholdPct);
  controlPreferences.putBool("v_protect", voltageProtectionEnabled);
  controlPreferences.putBool("c_protect", currentProtectionEnabled);
  controlPreferences.putBool("ign_lv", ignoredFaultLowVoltage);
  controlPreferences.putBool("ign_ub", ignoredFaultUnbalanceVoltage);
  controlPreferences.putBool("ign_c", ignoredFaultCurrent);
  controlPreferences.putBool("ign_p", ignoredFaultPhase);
  controlPreferences.end();
}

void loadModeOverrideFromPreferences() {
  controlPreferences.begin("control", true);
  modeCommandOverrideActive = true;
  autoModeEnabled = false;
  lowVoltageThresholdV = controlPreferences.getFloat("low_v_thr", LOW_VOLTAGE_THRESHOLD_DEFAULT_V);
  unbalanceVoltageThresholdPct = controlPreferences.getFloat("unbal_pct", UNBALANCE_VOLTAGE_THRESHOLD_DEFAULT_PCT);
  dryRunSetA = controlPreferences.getFloat("dry_set_a", DRY_RUN_SET_DEFAULT_A);
  overloadSetA = controlPreferences.getFloat("ovl_set_a", OVERLOAD_SET_DEFAULT_A);
  voltageProtectionEnabled = controlPreferences.getBool("v_protect", true);
  currentProtectionEnabled = controlPreferences.getBool("c_protect", true);
  const bool legacyIgnoredVoltage = controlPreferences.getBool("ign_v", false);
  ignoredFaultLowVoltage = controlPreferences.getBool("ign_lv", legacyIgnoredVoltage);
  ignoredFaultUnbalanceVoltage = controlPreferences.getBool("ign_ub", legacyIgnoredVoltage);
  ignoredFaultCurrent = controlPreferences.getBool("ign_c", false);
  ignoredFaultPhase = controlPreferences.getBool("ign_p", false);
  controlPreferences.end();
  lowVoltageThresholdV = constrain(lowVoltageThresholdV, 100.0f, 300.0f);
  unbalanceVoltageThresholdPct = constrain(unbalanceVoltageThresholdPct, 1.0f, 50.0f);
  dryRunSetA = constrain(dryRunSetA, 0.1f, 20.0f);
  overloadSetA = constrain(overloadSetA, 0.5f, 40.0f);
  if (overloadSetA <= dryRunSetA) {
    overloadSetA = dryRunSetA + 0.5f;
  }
  updateModeIndicator();
  pendingPublishStatus = true;
  Serial.println("[MODE] Restored dashboard-controlled mode from prefs");

  Serial.print("[VOLTAGE] Low-voltage threshold restored: ");
  Serial.println(lowVoltageThresholdV, 1);
  Serial.print("[CURRENT] Dry-run set: ");
  Serial.print(dryRunSetA, 2);
  Serial.print("A | Overload set: ");
  Serial.print(overloadSetA, 2);
  Serial.println("A");
  Serial.print("[FAULT CFG] Unbalance threshold: ");
  Serial.print(unbalanceVoltageThresholdPct, 1);
  Serial.print("% | V-protect=");
  Serial.print(voltageProtectionEnabled ? "ON" : "OFF");
  Serial.print(" LV-ignore=");
  Serial.print(ignoredFaultLowVoltage ? "1" : "0");
  Serial.print(" UB-ignore=");
  Serial.print(ignoredFaultUnbalanceVoltage ? "1" : "0");
  Serial.print(" C-protect=");
  Serial.print(currentProtectionEnabled ? "ON" : "OFF");
  Serial.print(" | IGN V/C/P=");
  Serial.print(ignoredFaultCurrent ? "1" : "0");
  Serial.print("/");
  Serial.println(ignoredFaultPhase ? "1" : "0");
  Serial.print("[WiFi] Primary profile (fixed): ");
  Serial.println(WIFI_FIXED_SSID);
  Serial.print("[WiFi] Primary password (fixed): ");
  Serial.println(WIFI_FIXED_PASSWORD);
}

void savePhaseSequenceToPreferences() {
  sequencePreferences.begin("phase", false);
  sequencePreferences.putString("phase_set", phaseSequenceSet);
  sequencePreferences.end();
}

void loadPhaseSequenceFromPreferences() {
  sequencePreferences.begin("phase", true);
  const String loaded = sequencePreferences.getString("phase_set", "RYB");
  sequencePreferences.end();

  const String normalized = normalizePhaseSequenceLabel(loaded, false);
  phaseSequenceSet = normalized.length() > 0 ? normalized : "RYB";
  Serial.print("[PHASE] Restored dashboard phase set: ");
  Serial.println(phaseSequenceSet);
}

bool parseStarterSequenceCommand(const JsonDocument& doc) {
  if (!doc.containsKey("starter_sequence")) {
    return false;
  }

  starterRelay12Ms = STARTER_RELAY12_MS;
  starterD12ToD14DelayMs = STARTER_D12_TO_D14_DELAY_MS;
  starterRelay14Ms = STARTER_RELAY14_MS;
  pendingPublishStatus = true;
  Serial.println("[SEQUENCE] Ignored MQTT starter_sequence: fixed by firmware");
  return true;
}

bool parsePhaseSequenceCommand(const JsonDocument& doc) {
  String requested = "";

  if (doc.containsKey("phase_sequence_set")) {
    requested = doc["phase_sequence_set"].as<String>();
  } else if (doc.containsKey("phase_sequence")) {
    requested = doc["phase_sequence"].as<String>();
  } else if (doc.containsKey("phase_override")) {
    requested = doc["phase_override"].as<String>();
  } else {
    return false;
  }

  const String normalized = normalizePhaseSequenceLabel(requested, false);
  if (normalized.length() == 0) {
    Serial.print("[PHASE] Invalid phase_sequence_set command: ");
    Serial.println(requested);
    return true;
  }

  if (phaseSequenceSet == normalized) {
    return true;
  }

  phaseSequenceSet = normalized;
  savePhaseSequenceToPreferences();
  pendingPublishStatus = true;
  Serial.print("[PHASE] Phase sequence set by dashboard: ");
  Serial.println(phaseSequenceSet);
  return true;
}

bool parseLowVoltageThresholdCommand(const JsonDocument& doc) {
  if (!(doc.containsKey("low_voltage_threshold_v") || doc.containsKey("low_voltage_threshold") || doc.containsKey("low_voltage_set"))) {
    return false;
  }

  float requested = lowVoltageThresholdV;
  if (doc.containsKey("low_voltage_threshold_v")) {
    requested = doc["low_voltage_threshold_v"].as<float>();
  } else if (doc.containsKey("low_voltage_threshold")) {
    requested = doc["low_voltage_threshold"].as<float>();
  } else if (doc.containsKey("low_voltage_set")) {
    requested = doc["low_voltage_set"].as<float>();
  }

  if (!(requested >= 100.0f && requested <= 300.0f)) {
    Serial.print("[VOLTAGE] Invalid low-voltage threshold: ");
    Serial.println(requested, 1);
    return true;
  }

  if (fabsf(lowVoltageThresholdV - requested) < 0.1f) {
    return true;
  }

  lowVoltageThresholdV = requested;
  saveLowVoltageThresholdToPreferences();
  pendingPublishStatus = true;
  Serial.print("[VOLTAGE] Low-voltage threshold set by dashboard: ");
  Serial.println(lowVoltageThresholdV, 1);
  return true;
}

bool parseCurrentProtectionCommand(const JsonDocument& doc) {
  if (!(doc.containsKey("dry_run_set_a") || doc.containsKey("dry_run_set") ||
        doc.containsKey("overload_set_a") || doc.containsKey("overload_set"))) {
    return false;
  }

  float requestedDry = dryRunSetA;
  float requestedOverload = overloadSetA;
  bool updated = false;

  if (doc.containsKey("dry_run_set_a")) {
    requestedDry = doc["dry_run_set_a"].as<float>();
    updated = true;
  } else if (doc.containsKey("dry_run_set")) {
    requestedDry = doc["dry_run_set"].as<float>();
    updated = true;
  }

  if (doc.containsKey("overload_set_a")) {
    requestedOverload = doc["overload_set_a"].as<float>();
    updated = true;
  } else if (doc.containsKey("overload_set")) {
    requestedOverload = doc["overload_set"].as<float>();
    updated = true;
  }

  if (!updated) {
    return true;
  }

  if (!(requestedDry >= 0.1f && requestedDry <= 20.0f) || !(requestedOverload >= 0.5f && requestedOverload <= 40.0f) || requestedOverload <= requestedDry) {
    Serial.print("[CURRENT] Invalid dry/overload set values: dry=");
    Serial.print(requestedDry, 2);
    Serial.print(" overload=");
    Serial.println(requestedOverload, 2);
    return true;
  }

  if (fabsf(dryRunSetA - requestedDry) < 0.05f && fabsf(overloadSetA - requestedOverload) < 0.05f) {
    return true;
  }

  dryRunSetA = requestedDry;
  overloadSetA = requestedOverload;
  saveCurrentThresholdsToPreferences();
  pendingPublishStatus = true;
  Serial.print("[CURRENT] Dry-run set updated: ");
  Serial.print(dryRunSetA, 2);
  Serial.print("A | Overload set: ");
  Serial.print(overloadSetA, 2);
  Serial.println("A");
  return true;
}

bool parseFaultConfigCommand(const JsonDocument& doc) {
  bool updated = false;

  if (doc.containsKey("unbalance_voltage_threshold_percent") || doc.containsKey("unbalance_voltage_fault_percent") || doc.containsKey("unbalance_set_percent")) {
    float requested = unbalanceVoltageThresholdPct;
    if (doc.containsKey("unbalance_voltage_threshold_percent")) {
      requested = doc["unbalance_voltage_threshold_percent"].as<float>();
    } else if (doc.containsKey("unbalance_voltage_fault_percent")) {
      requested = doc["unbalance_voltage_fault_percent"].as<float>();
    } else {
      requested = doc["unbalance_set_percent"].as<float>();
    }

    if (requested >= 1.0f && requested <= 50.0f) {
      if (fabsf(unbalanceVoltageThresholdPct - requested) >= 0.1f) {
        unbalanceVoltageThresholdPct = requested;
        updated = true;
      }
    }
  }

  if (doc.containsKey("voltage_protection_enabled")) {
    const bool requested = doc["voltage_protection_enabled"].as<bool>();
    if (requested != voltageProtectionEnabled) {
      voltageProtectionEnabled = requested;
      updated = true;
    }
  }

  if (doc.containsKey("ignored_fault_low_voltage")) {
    const bool requested = doc["ignored_fault_low_voltage"].as<bool>();
    if (requested != ignoredFaultLowVoltage) {
      ignoredFaultLowVoltage = requested;
      updated = true;
    }
  }

  if (doc.containsKey("ignored_fault_unbalance_voltage")) {
    const bool requested = doc["ignored_fault_unbalance_voltage"].as<bool>();
    if (requested != ignoredFaultUnbalanceVoltage) {
      ignoredFaultUnbalanceVoltage = requested;
      updated = true;
    }
  }

  if (doc.containsKey("current_protection_enabled")) {
    const bool requested = doc["current_protection_enabled"].as<bool>();
    if (requested != currentProtectionEnabled) {
      currentProtectionEnabled = requested;
      updated = true;
    }
  }

  if (doc.containsKey("ignored_fault_voltage")) {
    const bool requested = doc["ignored_fault_voltage"].as<bool>();
    if (requested != ignoredFaultLowVoltage || requested != ignoredFaultUnbalanceVoltage) {
      ignoredFaultLowVoltage = requested;
      ignoredFaultUnbalanceVoltage = requested;
      updated = true;
    }
  }

  if (doc.containsKey("ignored_fault_current")) {
    const bool requested = doc["ignored_fault_current"].as<bool>();
    if (requested != ignoredFaultCurrent) {
      ignoredFaultCurrent = requested;
      updated = true;
    }
  }

  if (doc.containsKey("ignored_fault_phase")) {
    const bool requested = doc["ignored_fault_phase"].as<bool>();
    if (requested != ignoredFaultPhase) {
      ignoredFaultPhase = requested;
      updated = true;
    }
  }

  if (doc.containsKey("ignored_faults")) {
    JsonObjectConst ignored = doc["ignored_faults"].as<JsonObjectConst>();
    if (!ignored.isNull()) {
      if (ignored.containsKey("voltage")) {
        const bool requested = ignored["voltage"].as<bool>();
        if (requested != ignoredFaultLowVoltage || requested != ignoredFaultUnbalanceVoltage) {
          ignoredFaultLowVoltage = requested;
          ignoredFaultUnbalanceVoltage = requested;
          updated = true;
        }
      }
      if (ignored.containsKey("low_voltage")) {
        const bool requested = ignored["low_voltage"].as<bool>();
        if (requested != ignoredFaultLowVoltage) {
          ignoredFaultLowVoltage = requested;
          updated = true;
        }
      }
      if (ignored.containsKey("unbalance_voltage")) {
        const bool requested = ignored["unbalance_voltage"].as<bool>();
        if (requested != ignoredFaultUnbalanceVoltage) {
          ignoredFaultUnbalanceVoltage = requested;
          updated = true;
        }
      }
      if (ignored.containsKey("current")) {
        const bool requested = ignored["current"].as<bool>();
        if (requested != ignoredFaultCurrent) {
          ignoredFaultCurrent = requested;
          updated = true;
        }
      }
      if (ignored.containsKey("phase")) {
        const bool requested = ignored["phase"].as<bool>();
        if (requested != ignoredFaultPhase) {
          ignoredFaultPhase = requested;
          updated = true;
        }
      }
    }
  }

  if (!updated) {
    return false;
  }

  saveFaultConfigToPreferences();
  pendingPublishStatus = true;
  Serial.print("[FAULT CFG] Updated: unbalance=");
  Serial.print(unbalanceVoltageThresholdPct, 1);
  Serial.print("% V-protect=");
  Serial.print(voltageProtectionEnabled ? "ON" : "OFF");
  Serial.print(" LV-ignore=");
  Serial.print(ignoredFaultLowVoltage ? "1" : "0");
  Serial.print(" UB-ignore=");
  Serial.print(ignoredFaultUnbalanceVoltage ? "1" : "0");
  Serial.print(" C-protect=");
  Serial.print(currentProtectionEnabled ? "ON" : "OFF");
  Serial.print(" IGN V/C/P=");
  Serial.print(ignoredFaultCurrent ? "1" : "0");
  Serial.print("/");
  Serial.println(ignoredFaultPhase ? "1" : "0");
  return true;
}

void saveAlarmWindowStateToPreferences() {
  controlPreferences.begin("control", false);
  controlPreferences.putBool("alarm_active", alarmWindowActive);
  controlPreferences.putBool("alarm_pending", alarmWindowStartPending);
  controlPreferences.putBool("alarm_off_latch", alarmWindowManualOffUntilEnd);
  controlPreferences.putULong("alarm_comp_ms", alarmWindowCompensationMs);
  controlPreferences.putULong("alarm_last_epoch", alarmWindowLastEpoch);
  controlPreferences.end();
}

void loadAlarmWindowStateFromPreferences() {
  controlPreferences.begin("control", true);
  alarmWindowActive = controlPreferences.getBool("alarm_active", false);
  alarmWindowStartPending = controlPreferences.getBool("alarm_pending", false);
  alarmWindowManualOffUntilEnd = controlPreferences.getBool("alarm_off_latch", false);
  alarmWindowCompensationMs = controlPreferences.getULong("alarm_comp_ms", 0);
  alarmWindowLastEpoch = controlPreferences.getULong("alarm_last_epoch", 0);
  controlPreferences.end();

  if (!alarmWindowActive) {
    alarmWindowStartPending = false;
  }

  alarmCompensationLastTickMs = millis();
  alarmWindowRecoveryPending = alarmWindowActive;
}

bool isCurrentMinuteInScheduleWindow(const ScheduleEntry& entry, int nowMinutes) {
  const int onMinutes = entry.onHour * 60 + entry.onMinute;
  const int offMinutes = entry.offHour * 60 + entry.offMinute;

  if (onMinutes == offMinutes) {
    return false;
  }

  if (onMinutes < offMinutes) {
    return nowMinutes >= onMinutes && nowMinutes < offMinutes;
  }

  // Window crosses midnight.
  return nowMinutes >= onMinutes || nowMinutes < offMinutes;
}

bool isAnyScheduleWindowActiveNow(const struct tm& timeinfo) {
  const int nowMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  for (int i = 0; i < SCHEDULE_COUNT; i++) {
    if (!scheduleEntries[i].enabled) {
      continue;
    }
    if (isCurrentMinuteInScheduleWindow(scheduleEntries[i], nowMinutes)) {
      return true;
    }
  }
  return false;
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
    if (item.containsKey("off_hour") || item.containsKey("off_minute")) {
      scheduleEntries[index].offHour = constrain(item["off_hour"] | scheduleEntries[index].offHour, 0, 23);
      scheduleEntries[index].offMinute = constrain(item["off_minute"] | scheduleEntries[index].offMinute, 0, 59);
    } else if (item.containsKey("duration_min")) {
      uint16_t durationMin = (uint16_t)(item["duration_min"] | 60);
      if (durationMin == 0) {
        durationMin = 60;
      }
      applyScheduleDurationToEntry(scheduleEntries[index], durationMin);
    }
    scheduleEntries[index].lastTriggeredDayOfYearOn = -1;
    scheduleEntries[index].lastTriggeredDayOfYearOff = -1;
    index++;
  }
}

void applyScheduleLogic() {
  if (manualTimedOnActive) {
    return;
  }

  const unsigned long nowMs = millis();
  if (nowMs - lastScheduleCheckMs < SCHEDULE_CHECK_INTERVAL_MS) {
    return;
  }
  lastScheduleCheckMs = nowMs;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return;
  }

  const unsigned long nowMsTick = millis();
  if (alarmCompensationLastTickMs == 0) {
    alarmCompensationLastTickMs = nowMsTick;
  }
  const unsigned long elapsedMs = nowMsTick - alarmCompensationLastTickMs;
  alarmCompensationLastTickMs = nowMsTick;

  if (alarmWindowRecoveryPending) {
    alarmWindowRecoveryPending = false;
    const uint32_t nowEpoch = (uint32_t)time(nullptr);
    if (alarmWindowLastEpoch > 0 && nowEpoch > alarmWindowLastEpoch) {
      const uint32_t outageSec = nowEpoch - alarmWindowLastEpoch;
      alarmWindowCompensationMs += ((unsigned long)outageSec * 1000UL);
      Serial.print("[SCHEDULE] Added reboot compensation: ");
      Serial.print((unsigned long)outageSec);
      Serial.println("s");
      saveAlarmWindowStateToPreferences();
    }
  }

  const bool scheduleActiveNow = isAnyScheduleWindowActiveNow(timeinfo);

  if (!scheduleActiveNow && alarmWindowManualOffUntilEnd) {
    alarmWindowManualOffUntilEnd = false;
    saveAlarmWindowStateToPreferences();
  }

  // Use strict schedule window only. Compensation based on X0 power-cut
  // caused extended ON periods on noisy inputs and delayed OFF actions.
  if (alarmWindowCompensationMs != 0) {
    alarmWindowCompensationMs = 0;
  }

  if (nowMsTick - lastAlarmStatePersistMs >= ALARM_STATE_PERSIST_INTERVAL_MS) {
    lastAlarmStatePersistMs = nowMsTick;
    const time_t nowEpoch = time(nullptr);
    if (nowEpoch > 0) {
      alarmWindowLastEpoch = (uint32_t)nowEpoch;
    }
    saveAlarmWindowStateToPreferences();
  }

  const bool shouldBeActive = !alarmWindowManualOffUntilEnd && scheduleActiveNow;

  if (shouldBeActive && !alarmWindowActive) {
    alarmWindowActive = true;
    alarmWindowStartPending = !relayStateOn;
    pendingAlarmStateReason = "E_ALARM_ON";
    autoAlarmLatched = false;
    autoRestartAfterFault = false;
    autoRestartClearSinceMs = 0;
    autoRestartFaultReason = "";
    saveAlarmWindowStateToPreferences();
    Serial.println("[SCHEDULE] Alarm window ACTIVE");
    pendingPublishStatus = true;
    return;
  }

  if (!shouldBeActive && alarmWindowActive) {
    alarmWindowActive = false;
    alarmWindowStartPending = false;
    pendingAlarmStateReason = "E_ALARM_OFF";
    autoAlarmLatched = false;
    autoRestartAfterFault = false;
    autoRestartClearSinceMs = 0;
    autoRestartFaultReason = "";
    saveAlarmWindowStateToPreferences();
    stopStarterWithReason("E_ALARM_OFF");
    Serial.println("[SCHEDULE] Alarm window INACTIVE");
    pendingPublishStatus = true;
  }
}

bool isAutoStartBlockedByProtection(String& reason) {
  if (!voltageProtectionEnabled && !currentProtectionEnabled) {
    reason = "";
    return false;
  }

  const float voltageR = readVoltageFromYChannel(VOLTAGE_R_Y_CHANNEL);
  const float voltageY = readVoltageFromYChannel(VOLTAGE_Y_Y_CHANNEL);
  const float voltageB = readVoltageFromYChannel(VOLTAGE_B_Y_CHANNEL);
  const float voltageAvg = (voltageR + voltageY + voltageB) / 3.0f;
  const bool lowVoltageActive = voltageAvg < lowVoltageThresholdV;
  const float maxVoltageDeviation = max(max(fabsf(voltageR - voltageAvg), fabsf(voltageY - voltageAvg)), fabsf(voltageB - voltageAvg));
  const float unbalanceVoltagePercent = voltageAvg > 0.1f ? (maxVoltageDeviation / voltageAvg) * 100.0f : 0.0f;
  const bool unbalanceVoltageActive = unbalanceVoltagePercent > unbalanceVoltageThresholdPct;
  const bool phaseUnknown = (phaseSequenceEstimate == "UNKNOWN");
  const bool phaseMismatch = (!phaseUnknown) && (phaseSequenceEstimate != phaseSequenceSet);

  if (voltageProtectionEnabled && !ignoredFaultLowVoltage && lowVoltageActive) {
    reason = "low voltage active";
    return true;
  }

  if (voltageProtectionEnabled && !ignoredFaultUnbalanceVoltage && unbalanceVoltageActive) {
    reason = "unbalance voltage active";
    return true;
  }

  if (voltageProtectionEnabled && !ignoredFaultPhase && phaseUnknown) {
    reason = "phase sequence unknown";
    return true;
  }

  if (voltageProtectionEnabled && !ignoredFaultPhase && phaseMismatch) {
    reason = "phase sequence mismatch";
    return true;
  }

  reason = "";
  return false;
}

void applyAutoModeLogic() {
  const bool alarmStartRequested = alarmWindowStartPending || alarmWindowActive;
  if (manualTimedOnActive || (!autoModeEnabled && !alarmStartRequested)) {
    return;
  }

  static String lastAutoStartBlockReason = "";

  const unsigned long now = millis();
  if (!alarmStartRequested && !modeDecisionReady) {
    return;
  }

  if (!alarmStartRequested && (now - bootMs < AUTO_START_DELAY_MS)) {
    return;
  }

  if (!relayStateOn && isAnyCurrentTripAutoRestartLocked()) {
    static bool lockoutLogged = false;
    if (!lockoutLogged) {
      Serial.println("[AUTO] Restart locked after second DRY/OVERLOAD trip. Change mode manually to reset.");
      lockoutLogged = true;
    }
    return;
  }

  if (!relayStateOn) {
    String blockReason = "";
    if (isAutoStartBlockedByProtection(blockReason)) {
      autoRestartClearSinceMs = 0;
      if (blockReason != lastAutoStartBlockReason) {
        Serial.print("[AUTO] Start blocked: ");
        Serial.println(blockReason);
        lastAutoStartBlockReason = blockReason;
      }
      return;
    }

    if (autoRestartAfterFault && isCurrentTripReason(autoRestartFaultReason)) {
      if (!isCurrentFaultClearForAutoRestart(autoRestartFaultReason)) {
        autoRestartClearSinceMs = 0;
        return;
      }
    }

    const bool needsClearDelay = autoRestartAfterFault || autoAlarmLatched;
    if (needsClearDelay) {
      if (autoRestartClearSinceMs == 0) {
        autoRestartClearSinceMs = now;
        Serial.print("[AUTO] Fault cleared, waiting ");
        Serial.print(AUTO_RESTART_AFTER_CLEAR_MS / 1000);
        Serial.println("s before restart");
        return;
      }

      if (now - autoRestartClearSinceMs < AUTO_RESTART_AFTER_CLEAR_MS) {
        return;
      }
    }

    const String startReason = alarmStartRequested
      ? "E_ALARM_ON"
      : (autoRestartAfterFault
        ? "AUTO_RESTART"
        : (autoAlarmLatched ? "E_ALARM_OFF" : "AUTO_ON"));

    if (autoRestartAfterFault && isCurrentTripReason(autoRestartFaultReason)) {
      markTripRestartUsedForReason(autoRestartFaultReason);
    }

    startStarterOnSequenceWithReason(startReason);
    alarmWindowStartPending = false;
    saveAlarmWindowStateToPreferences();
    autoAlarmLatched = false;
    autoRestartAfterFault = false;
    autoRestartClearSinceMs = 0;
    autoRestartFaultReason = "";
    pendingPublishStatus = true;
    Serial.println(alarmStartRequested ? "[ALARM] Starter ON from ESP alarm window" : "[AUTO] Voltage OK after 30s average, starter ON");
    lastAutoStartBlockReason = "";
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

void applyProtectionFaultSafety() {
  if (!relayStateOn) {
    pendingProtectionFaultReason = "";
    pendingProtectionFaultSinceMs = 0;
    return;
  }

  const bool d27On = isRelayPinOn(RELAY_PIN_27);
  if (!d27On) {
    return;
  }

  static unsigned long lastProtectionCheckMs = 0;
  const unsigned long nowMs = millis();
  if (lastProtectionCheckMs != 0 && (nowMs - lastProtectionCheckMs < 1000)) {
    return;
  }
  lastProtectionCheckMs = nowMs;

  const float voltageR = readVoltageFromYChannel(VOLTAGE_R_Y_CHANNEL);
  const float voltageY = readVoltageFromYChannel(VOLTAGE_Y_Y_CHANNEL);
  const float voltageB = readVoltageFromYChannel(VOLTAGE_B_Y_CHANNEL);
  const float voltageAvg = (voltageR + voltageY + voltageB) / 3.0f;
  const bool lowVoltageActive = voltageAvg < lowVoltageThresholdV;
  const float maxVoltageDeviation = max(max(fabsf(voltageR - voltageAvg), fabsf(voltageY - voltageAvg)), fabsf(voltageB - voltageAvg));
  const float unbalanceVoltagePercent = voltageAvg > 0.1f ? (maxVoltageDeviation / voltageAvg) * 100.0f : 0.0f;
  const bool unbalanceVoltageActive = unbalanceVoltagePercent > unbalanceVoltageThresholdPct;

  float currentA = readCurrentFromSensor();
  if (currentA < 0.25f) {
    currentA = 0.0f;
  }
  const bool dryRunActive = currentA < dryRunSetA;
  const bool overloadActive = currentA > overloadSetA;
  const bool phaseUnknown = (phaseSequenceEstimate == "UNKNOWN");
  const bool phaseMismatch = (!phaseUnknown) && (phaseSequenceEstimate != phaseSequenceSet);
  const bool phaseFaultActive = phaseUnknown || phaseMismatch;
  const bool inDryRunStartupIgnore = motorLastStartMs != 0 && (nowMs - motorLastStartMs < DRY_RUN_STARTUP_IGNORE_MS);
  const bool inOverloadStartupIgnore = motorLastStartMs != 0 && (nowMs - motorLastStartMs < OVERLOAD_STARTUP_IGNORE_MS);

  const String faultReason = getProtectionFaultReason(lowVoltageActive, unbalanceVoltageActive, (currentProtectionEnabled && !ignoredFaultCurrent && !inDryRunStartupIgnore && dryRunActive), (currentProtectionEnabled && !ignoredFaultCurrent && !inOverloadStartupIgnore && overloadActive), phaseFaultActive);

  if (faultReason.length() == 0) {
    pendingProtectionFaultReason = "";
    pendingProtectionFaultSinceMs = 0;
    return;
  }

  autoRestartClearSinceMs = 0;

  if (pendingProtectionFaultReason != faultReason) {
    pendingProtectionFaultReason = faultReason;
    pendingProtectionFaultSinceMs = nowMs;
    Serial.print("[PROTECT] Pending fault: ");
    Serial.print(faultReason);
    Serial.print(" for ");
    Serial.print(PROTECTION_FAULT_TRIP_DELAY_MS / 1000);
    Serial.println("s");
    return;
  }

  if (pendingProtectionFaultSinceMs == 0 || (nowMs - pendingProtectionFaultSinceMs < PROTECTION_FAULT_TRIP_DELAY_MS)) {
    return;
  }

  stopStarterWithReason(faultReason);
  if (manualTimedOnActive) {
    manualTimerFaultRestartPending = true;
    manualTimerFaultReason = faultReason;
    manualTimerFaultClearSinceMs = 0;
    manualTimerPausedByCurrent = true;
    manualTimerLastTickMs = millis();
    Serial.print("[MANUAL TIMER] Fault trip during timer: ");
    Serial.print(faultReason);
    Serial.println(". Will restart after clear.");
  } else if (autoModeEnabled) {
    autoRestartClearSinceMs = 0;
    if (isCurrentTripReason(faultReason) && isTripRestartUsedForReason(faultReason)) {
      lockoutTripAutoRestartForReason(faultReason);
      autoAlarmLatched = false;
      autoRestartAfterFault = false;
      autoRestartFaultReason = "";
      Serial.print("[AUTO] Restart lockout active for ");
      Serial.print(faultReason);
      Serial.println(" (second trip). Switch mode manually to reset.");
    } else {
      autoAlarmLatched = true;
      autoRestartAfterFault = true;
      autoRestartFaultReason = faultReason;
    }
  }
  pendingPublishStatus = true;
  Serial.print("[PROTECT] OFF after delay: ");
  if (faultReason == "LOW_VOLTAGE_TRIP") {
    Serial.print("Low voltage fault (avg=");
    Serial.print(voltageAvg, 1);
    Serial.print("V, set=");
    Serial.print(lowVoltageThresholdV, 1);
    Serial.println("V)");
  } else if (faultReason == "UB_TRIP") {
    Serial.print("Unbalance fault (");
    Serial.print(unbalanceVoltagePercent, 1);
    Serial.print("%, set=");
    Serial.print(unbalanceVoltageThresholdPct, 1);
    Serial.println("%)");
  } else if (faultReason == "DRY_RUN_TRIP") {
    Serial.print("Dry-run fault (I=");
    Serial.print(currentA, 2);
    Serial.print("A, set=");
    Serial.print(dryRunSetA, 2);
    Serial.println("A)");
  } else if (faultReason == "OL_TRIP") {
    Serial.print("Overload fault (I=");
    Serial.print(currentA, 2);
    Serial.print("A, set=");
    Serial.print(overloadSetA, 2);
    Serial.println("A)");
  } else if (faultReason == "SEQ_TRIP") {
    Serial.print("Phase fault (measured=");
    Serial.print(phaseSequenceEstimate);
    Serial.print(", set=");
    Serial.print(phaseSequenceSet);
    Serial.println(")");
  }
}

void applyPostOffCurrentMonitor() {
  const bool d27On = isRelayPinOn(RELAY_PIN_27);
  if (relayStateOn || d27On) {
    offCurrentMonitorStartMs = 0;
    offCurrentTripRaised = false;
    return;
  }

  if (offCurrentMonitorStartMs == 0) {
    offCurrentMonitorStartMs = millis();
  }

  if (offCurrentTripRaised) {
    return;
  }

  const unsigned long nowMs = millis();
  if (nowMs - offCurrentMonitorStartMs < OFF_CURRENT_SETTLE_MS) {
    return;
  }

  float currentA = readCurrentFromSensor();
  if (currentA < OFF_CURRENT_ZERO_THRESHOLD_A) {
    return;
  }

  offCurrentTripRaised = true;
  pushSwitchHistoryEvent(false, time(nullptr), "TRIP_ERROR_CURRENT_NOT_ZERO");
  pendingPublishStatus = true;
  Serial.print("[POST-OFF] TRIP ERROR: D27 OFF but current still ");
  Serial.print(currentA, 2);
  Serial.println("A after 30s");
}

void publishStatus(bool retained) {
  const int x0Raw = readSensorChannelRaw(0, 16);
  const int x1Raw = readSensorChannelRaw(1, 16);
  const int x3Raw = readSensorChannelRaw(3, 16);
  const bool powerSupplyOn = x0Raw < X0_POWER_SUPPLY_CUT_THRESHOLD;
  const bool powerSupplyCut = !powerSupplyOn;
  const bool phaseUnknown = (phaseSequenceEstimate == "UNKNOWN");
  const bool phaseMismatch = (!phaseUnknown) && (phaseSequenceEstimate != phaseSequenceSet);
  const bool phaseFaultActive = phaseUnknown || phaseMismatch;
  const String phaseMeasuredByX1 = phaseSequenceEstimate;
  const bool d27On = isRelayPinOn(RELAY_PIN_27);
  const float voltageR = readVoltageFromYChannel(VOLTAGE_R_Y_CHANNEL);
  const float voltageY = readVoltageFromYChannel(VOLTAGE_Y_Y_CHANNEL);
  const float voltageB = readVoltageFromYChannel(VOLTAGE_B_Y_CHANNEL);
  const float voltageAvg = (voltageR + voltageY + voltageB) / 3.0f;
  const bool lowVoltageActive = voltageAvg < lowVoltageThresholdV;
  const float maxVoltageDeviation = max(max(fabsf(voltageR - voltageAvg), fabsf(voltageY - voltageAvg)), fabsf(voltageB - voltageAvg));
  const float unbalanceVoltagePercent = voltageAvg > 0.1f ? (maxVoltageDeviation / voltageAvg) * 100.0f : 0.0f;
  const bool unbalanceVoltageActive = unbalanceVoltagePercent > unbalanceVoltageThresholdPct;
  int currentRawAdc = 0;
  float currentAdcVoltage = 0.0f;
  float currentA = readCurrentFromSensor(&currentRawAdc, &currentAdcVoltage);
  if (currentA < 0.25f) {
    currentA = 0.0f;
  }
  const bool motorRunningByCurrent = isRunningFromCurrent(currentA);
  const bool motorFeedbackOn = d27On;
  const bool dryRunActive = d27On && currentA < dryRunSetA;
  const bool overloadActive = d27On && currentA > overloadSetA;
  const String statusText = buildStatusText(autoModeEnabled, d27On, motorRunningByCurrent);
  const char* stateText = motorFeedbackOn ? "ON" : "OFF";
  const time_t nowEpoch = time(nullptr);
  updateSwitchHistoryFromState(d27On, nowEpoch);

  char localTimeBuffer[32] = "";
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 50)) {
    strftime(localTimeBuffer, sizeof(localTimeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  }

  StaticJsonDocument<STATUS_JSON_CAPACITY> doc;
  const String latestReason = switchHistory[0].valid ? switchHistory[0].reason : String("");
  const char* latestState = switchHistory[0].valid ? (switchHistory[0].on ? "ON" : "OFF") : stateText;
  const bool alarmStateActive = alarmWindowActive || alarmWindowStartPending || autoAlarmLatched || autoRestartAfterFault;
  String alarmStateText = alarmStateActive ? String("E_ALARM_ON") : String("");
  if (!alarmStateActive && pendingAlarmStateReason.length() > 0) {
    alarmStateText = pendingAlarmStateReason;
    pendingAlarmStateReason = "";
  }
  doc["status"] = stateText;
  doc["state"] = stateText;
  doc["status_text"] = statusText;
  doc["latest_reason"] = latestReason;
  doc["latest_state"] = latestState;
  doc["alarm_active"] = alarmStateActive;
  doc["alarm_state"] = alarmStateText;
  doc["relay_on"] = motorFeedbackOn;
  doc["motor"] = motorFeedbackOn;
  doc["io12_status"] = motorFeedbackOn;
  doc["motor_running_current"] = motorRunningByCurrent;
  doc["d27_status"] = d27On;
  doc["io27_status"] = d27On;
  doc["relay27_on"] = d27On;
  doc["current"] = currentA;
  doc["current_raw_adc"] = currentRawAdc;
  doc["current_adc_voltage"] = currentAdcVoltage;
  doc["motor_on_current_threshold_a"] = MOTOR_ON_CURRENT_THRESHOLD_A;
  doc["dry_run_set_a"] = dryRunSetA;
  doc["overload_set_a"] = overloadSetA;
  doc["dry_run_active"] = dryRunActive;
  doc["overload_active"] = overloadActive;
  doc["x0"] = x0Raw;
  doc["x1"] = x1Raw;
  doc["x3"] = x3Raw;
  doc["power_supply_cut"] = powerSupplyCut;
  doc["power_supply_on"] = powerSupplyOn;
  doc["x0_power_cut_threshold"] = X0_POWER_SUPPLY_CUT_THRESHOLD;
  doc["x3_power_cut_threshold"] = X0_POWER_SUPPLY_CUT_THRESHOLD;
  doc["mode"] = autoModeEnabled ? "AUTO" : "MANUAL";
  doc["mode_auto"] = autoModeEnabled;
  doc["mode_source"] = "DASHBOARD";
  doc["mode_override"] = modeCommandOverrideActive;
  doc["manual_timer_active"] = manualTimedOnActive;
  doc["manual_timer_duration_sec"] = manualTimedOnDurationSec;
  doc["manual_timer_end_unix"] = manualTimedOnEndEpoch;
  doc["manual_timer_remaining_sec"] = getManualTimedOnRemainingSec();
  doc["manual_timer_paused"] = manualTimerPausedByCurrent;
  doc["manual_timer_count_current_threshold_a"] = MANUAL_TIMER_COUNT_CURRENT_THRESHOLD_A;
  doc["manual_timer_fault_bypass"] = false;
  doc["mode_x0_raw"] = modeDecisionReady ? modeDecisionRaw : readSensorChannelRaw(0, 50);
  doc["phase_sequence"] = getDisplayPhaseSequence();
  doc["phase_sequence_measured"] = phaseMeasuredByX1;
  doc["phase_sequence_set"] = phaseSequenceSet;
  doc["phase_sequence_source"] = "X1";
  doc["phase_seq_stable"] = isPhaseSequenceStable();
  doc["voltage_r"] = voltageR;
  doc["voltage_y"] = voltageY;
  doc["voltage_b"] = voltageB;
  doc["voltage"] = voltageAvg;
  doc["low_voltage_threshold_v"] = lowVoltageThresholdV;
  doc["low_voltage_active"] = lowVoltageActive;
  doc["unbalance_voltage_threshold_percent"] = unbalanceVoltageThresholdPct;
  doc["unbalance_voltage_percent"] = unbalanceVoltagePercent;
  doc["unbalance_voltage_active"] = unbalanceVoltageActive;
  doc["voltage_protection_enabled"] = voltageProtectionEnabled;
  doc["current_protection_enabled"] = currentProtectionEnabled;
  doc["ignored_fault_low_voltage"] = ignoredFaultLowVoltage;
  doc["ignored_fault_unbalance_voltage"] = ignoredFaultUnbalanceVoltage;
  doc["ignored_fault_voltage"] = ignoredFaultLowVoltage || ignoredFaultUnbalanceVoltage;
  doc["ignored_fault_current"] = ignoredFaultCurrent;
  doc["ignored_fault_phase"] = ignoredFaultPhase;
  doc["protection_fault_pending"] = pendingProtectionFaultReason.length() > 0;
  doc["protection_fault_reason"] = pendingProtectionFaultReason;
  doc["phase_fault_active"] = phaseFaultActive;
  doc["fault_low_voltage"] = lowVoltageActive;
  doc["fault_unbalance_voltage"] = unbalanceVoltageActive;
  doc["fault_dry_run"] = dryRunActive;
  doc["fault_overload"] = overloadActive;
  doc["fault_phase"] = phaseFaultActive;

  JsonArray switchHistoryArr = doc.createNestedArray("switch_history");
  for (int i = 0; i < SWITCH_HISTORY_MAX; i++) {
    if (!switchHistory[i].valid) {
      continue;
    }
    JsonObject item = switchHistoryArr.createNestedObject();
    item["state"] = switchHistory[i].on ? "ON" : "OFF";
    item["time_unix"] = switchHistory[i].epoch;
    if (switchHistory[i].reason.length() > 0) {
      item["reason"] = switchHistory[i].reason;
    }
  }

  doc["module_id"] = MODULE_ID;
  doc["module_id4"] = moduleId4;
  doc["rtc_valid"] = isRtcTimeValid();
  doc["time_unix"] = (long)nowEpoch;
  if (localTimeBuffer[0] != '\0') {
    doc["local_time"] = localTimeBuffer;
  }

  JsonArray scheduleArray = doc.createNestedArray("schedule");
  for (int i = 0; i < SCHEDULE_COUNT; i++) {
    JsonObject entry = scheduleArray.createNestedObject();
    entry["enabled"] = scheduleEntries[i].enabled;
    entry["on_hour"] = scheduleEntries[i].onHour;
    entry["on_minute"] = scheduleEntries[i].onMinute;
    entry["duration_min"] = getScheduleDurationMinutes(scheduleEntries[i]);
    entry["off_hour"] = scheduleEntries[i].offHour;
    entry["off_minute"] = scheduleEntries[i].offMinute;
  }

  char payload[STATUS_JSON_CAPACITY];
  const size_t requiredLen = measureJson(doc);
  if (requiredLen == 0 || requiredLen >= sizeof(payload) - 1) {
    Serial.print("[MQTT] ERROR: status too large before serialize, required=");
    Serial.print((unsigned int)requiredLen);
    Serial.print(" capacity=");
    Serial.println((unsigned int)STATUS_JSON_CAPACITY);
    return;
  }

  const size_t payloadLen = serializeJson(doc, payload, sizeof(payload));
  if (payloadLen == 0 || payloadLen >= sizeof(payload) - 1) {
    Serial.println("[MQTT] ERROR: status serialize failed/too large");
    return;
  }

  const bool statusOk = mqttClient.publish(topicStatus.c_str(), payload, retained);
  if (!statusOk) {
    Serial.print("[MQTT] ERROR: status publish failed, state=");
    Serial.print(mqttClient.state());
    Serial.print(", len=");
    Serial.println((unsigned int)payloadLen);
  }
}

void publishTelemetry() {
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] WARN: skip telemetry publish, MQTT disconnected");
    return;
  }

  const int x1Raw = readSensorChannelRaw(1, 16);
  const bool phaseOkByX1 = true;
  const bool d27On = isRelayPinOn(RELAY_PIN_27);

  const int x0Raw = readSensorChannelRaw(0, 16);
  const int x3Raw = readSensorChannelRaw(3, 16);
  const bool powerSupplyOn = x0Raw < X0_POWER_SUPPLY_CUT_THRESHOLD;
  const bool powerSupplyCut = !powerSupplyOn;

  const float voltageR = readVoltageFromYChannel(VOLTAGE_R_Y_CHANNEL);
  const float voltageY = readVoltageFromYChannel(VOLTAGE_Y_Y_CHANNEL);
  const float voltageB = readVoltageFromYChannel(VOLTAGE_B_Y_CHANNEL);
  const float voltage = (voltageR + voltageY + voltageB) / 3.0f;
  const bool lowVoltageActive = voltage < lowVoltageThresholdV;
  const float maxVoltageDeviation = max(max(fabsf(voltageR - voltage), fabsf(voltageY - voltage)), fabsf(voltageB - voltage));
  const float unbalanceVoltagePercent = voltage > 0.1f ? (maxVoltageDeviation / voltage) * 100.0f : 0.0f;
  const bool unbalanceVoltageActive = unbalanceVoltagePercent > unbalanceVoltageThresholdPct;
  int currentRawAdc = 0;
  float currentAdcVoltage = 0.0f;
  float current = readCurrentFromSensor(&currentRawAdc, &currentAdcVoltage);
  const bool motorRunningByCurrent = isRunningFromCurrent(current);
  const bool dryRunActive = d27On && current < dryRunSetA;
  const bool overloadActive = d27On && current > overloadSetA;
  const bool phaseUnknown = (phaseSequenceEstimate == "UNKNOWN");
  const bool phaseMismatch = (!phaseUnknown) && (phaseSequenceEstimate != phaseSequenceSet);
  const bool phaseFaultActive = phaseUnknown || phaseMismatch;
  if (!motorRunningByCurrent && current < 0.25f) {
    current = 0.0f;
  }

  const float totalPhaseVoltage = voltageR + voltageY + voltageB;
  const float power = totalPhaseVoltage * current * POWER_FACTOR;
  const time_t nowEpoch = time(nullptr);
  updateSwitchHistoryFromState(d27On, nowEpoch);
  const String latestReason = switchHistory[0].valid ? switchHistory[0].reason : String("");
  const char* latestState = switchHistory[0].valid ? (switchHistory[0].on ? "ON" : "OFF") : (d27On ? "ON" : "OFF");
  const bool alarmStateActive = alarmWindowActive || alarmWindowStartPending || autoAlarmLatched || autoRestartAfterFault;
  String alarmStateText = alarmStateActive ? String("E_ALARM_ON") : String("");
  if (!alarmStateActive && pendingAlarmStateReason.length() > 0) {
    alarmStateText = pendingAlarmStateReason;
    pendingAlarmStateReason = "";
  }

  // Keep telemetry compact to reduce publish failures on unstable links.
  StaticJsonDocument<TELEMETRY_JSON_CAPACITY> doc;
  doc["module_id"] = MODULE_ID;
  doc["module_id4"] = moduleId4;
  doc["latest_reason"] = latestReason;
  doc["latest_state"] = latestState;
  doc["alarm_active"] = alarmStateActive;
  doc["alarm_state"] = alarmStateText;
  doc["relay_on"] = d27On;
  doc["motor"] = d27On;
  doc["io12_status"] = d27On;
  doc["d27_status"] = d27On;
  doc["io27_status"] = d27On;
  doc["relay27_on"] = d27On;
  doc["x0"] = x0Raw;
  doc["x1"] = x1Raw;
  doc["x3"] = x3Raw;
  doc["power_supply_cut"] = powerSupplyCut;
  doc["power_supply_on"] = powerSupplyOn;
  doc["x0_power_cut_threshold"] = X0_POWER_SUPPLY_CUT_THRESHOLD;
  doc["x3_power_cut_threshold"] = X0_POWER_SUPPLY_CUT_THRESHOLD;
  doc["motor_running_current"] = motorRunningByCurrent;
  doc["x3_running"] = motorRunningByCurrent;
  doc["mode"] = autoModeEnabled ? "AUTO" : "MANUAL";
  doc["mode_auto"] = autoModeEnabled;
  doc["manual_timer_active"] = manualTimedOnActive;
  doc["manual_timer_duration_sec"] = manualTimedOnDurationSec;
  doc["manual_timer_end_unix"] = manualTimedOnEndEpoch;
  doc["manual_timer_remaining_sec"] = getManualTimedOnRemainingSec();
  doc["manual_timer_paused"] = manualTimerPausedByCurrent;
  doc["manual_timer_count_current_threshold_a"] = MANUAL_TIMER_COUNT_CURRENT_THRESHOLD_A;
  doc["manual_timer_fault_bypass"] = false;
  doc["voltage_r"] = voltageR;
  doc["voltage_y"] = voltageY;
  doc["voltage_b"] = voltageB;
  doc["voltage"] = voltage;
  doc["low_voltage_threshold_v"] = lowVoltageThresholdV;
  doc["low_voltage_active"] = lowVoltageActive;
  doc["unbalance_voltage_threshold_percent"] = unbalanceVoltageThresholdPct;
  doc["unbalance_voltage_percent"] = unbalanceVoltagePercent;
  doc["unbalance_voltage_active"] = unbalanceVoltageActive;
  doc["phase_fault_active"] = phaseFaultActive;
  doc["current"] = current;
  doc["current_raw_adc"] = currentRawAdc;
  doc["current_adc_voltage"] = currentAdcVoltage;
  doc["dry_run_set_a"] = dryRunSetA;
  doc["overload_set_a"] = overloadSetA;
  doc["dry_run_active"] = dryRunActive;
  doc["overload_active"] = overloadActive;
  doc["power"] = power;
  doc["timestamp_ms"] = millis();
  doc["rtc_valid"] = isRtcTimeValid();
  doc["time_unix"] = (long)nowEpoch;
  doc["phase_sequence"] = getDisplayPhaseSequence();
  doc["phase_sequence_measured"] = phaseSequenceEstimate;
  doc["phase_sequence_set"] = phaseSequenceSet;
  doc["phase_sequence_source"] = "X1";
  doc["phase_seq_stable"] = phaseOkByX1;
  doc["protection_fault_pending"] = pendingProtectionFaultReason.length() > 0;
  doc["protection_fault_reason"] = pendingProtectionFaultReason;

  const size_t requiredLen = measureJson(doc);
  if (requiredLen == 0 || requiredLen >= TELEMETRY_JSON_CAPACITY - 1) {
    Serial.print("[MQTT] ERROR: telemetry too large before serialize, required=");
    Serial.print((unsigned int)requiredLen);
    Serial.print(" capacity=");
    Serial.println((unsigned int)TELEMETRY_JSON_CAPACITY);
    return;
  }

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

  uint32_t timedDurationSec = 0;
  if (parseManualTimedCommandText(value, timedDurationSec)) {
    if (startManualTimedOn(timedDurationSec, "MANUAL_TIMER_ON")) {
      Serial.println("[CTRL] Manual timer ON command accepted");
    }
    return;
  }

  if (value == "MANUAL_TIMER_CANCEL" || value == "MANUAL_ON_CANCEL" || value == "MANUAL_TIMER_OFF") {
    cancelManualTimedOn(true, "MANUAL_TIMER_CANCEL");
    Serial.println("[CTRL] Manual timer cancelled");
    return;
  }

  if (value == "AUTO" || value == "MODE_AUTO") {
    autoModeEnabled = true;
    modeCommandOverrideActive = true;
    updateModeIndicator();
    pendingPublishStatus = true;
    saveModeOverrideToPreferences();
    Serial.println("[CTRL] Mode set to AUTO from dashboard (override active until hardware input toggle)");
    return;
  }

  if (value == "PHASE_RYB") {
    phaseSequenceSet = "RYB";
    savePhaseSequenceToPreferences();
    pendingPublishStatus = true;
    Serial.println("[PHASE] Phase sequence set to RYB from dashboard");
    return;
  }

  if (value == "PHASE_YBR" || value == "PHASE_RBY") {
    phaseSequenceSet = "YBR";
    savePhaseSequenceToPreferences();
    pendingPublishStatus = true;
    Serial.println("[PHASE] Phase sequence set to YBR from dashboard");
    return;
  }

  if (value == "PHASE_BRY") {
    phaseSequenceSet = "BRY";
    savePhaseSequenceToPreferences();
    pendingPublishStatus = true;
    Serial.println("[PHASE] Phase sequence set to BRY from dashboard");
    return;
  }

  if (value == "PHASE_UNKNOWN") {
    phaseSequenceSet = "UNKNOWN";
    savePhaseSequenceToPreferences();
    pendingPublishStatus = true;
    Serial.println("[PHASE] Phase sequence set to UNKNOWN from dashboard");
    return;
  }

  if (value == "MANUAL" || value == "MODE_MANUAL") {
    clearManualTimedOnState(true);
    modeCommandOverrideActive = true;
    applyManualModeTransition("AUTO_OFF");
    saveModeOverrideToPreferences();
    Serial.println("[CTRL] Mode set to MANUAL from dashboard (override active until hardware input toggle)");
    return;
  }

  if (parseOnCommand(msg)) {
    if (autoModeEnabled) {
      Serial.println("[CTRL] ON ignored in AUTO mode. Switch to MANUAL for button control.");
      return;
    }
    clearManualTimedOnState(true);
    String upperMsg = String(msg);
    upperMsg.toUpperCase();
    startStarterOnSequenceWithReason(     
      upperMsg.indexOf("D_ALARM_ON") >= 0 ? "D_ALARM_ON"
      : (upperMsg.indexOf("E_ALARM_ON") >= 0 || upperMsg.indexOf("ALARM_ON") >= 0 ? "E_ALARM_ON" : "MANUAL_ON_CMD")
    );
    pendingPublishStatus = true;
    Serial.println("[CTRL] STARTER ON sequence: D12(30s) -> D14(30s), D27 latched ON");
    return;
  }

  if (parseOffCommand(msg)) {
    clearManualTimedOnState(true);
    alarmWindowActive = false;
    alarmWindowStartPending = false;
    alarmWindowManualOffUntilEnd = true;
    alarmWindowCompensationMs = 0;
    alarmWindowRecoveryPending = false;
    autoAlarmLatched = false;
    autoRestartAfterFault = false;
    autoRestartClearSinceMs = 0;
    autoRestartFaultReason = "";
    saveAlarmWindowStateToPreferences();
    String upperMsg = String(msg);
    upperMsg.toUpperCase();
    stopStarterWithReason(
      upperMsg.indexOf("D_ALARM_OFF") >= 0 ? "D_ALARM_OFF"
      : (upperMsg.indexOf("E_ALARM_OFF") >= 0 || upperMsg.indexOf("ALARM_OFF") >= 0 ? "E_ALARM_OFF" : "MANUAL_OFF_CMD")
    );
    pendingPublishStatus = true;
    Serial.println("[CTRL] STARTER OFF: D12/D14/D27 OFF, alarm window paused until current schedule window ends");
    return;
  }

  if (value == "TOGGLE") {
    if (autoModeEnabled) {
      Serial.println("[CTRL] TOGGLE ignored in AUTO mode. Switch to MANUAL for button control.");
      return;
    }
    if (relayStateOn) {
      clearManualTimedOnState(true);
      stopStarterWithReason("MANUAL_OFF_CMD");
    } else {
      clearManualTimedOnState(true);
      startStarterOnSequenceWithReason("MANUAL_ON_CMD");
    }
    pendingPublishStatus = true;
    Serial.println("[CTRL] STARTER TOGGLE");
  }
}

bool parseManualTimedOnCommand(const JsonDocument& doc) {
  bool hasTimerField = false;
  uint32_t durationSec = 0;

  if (doc.containsKey("manual_on_duration_minutes")) {
    hasTimerField = true;
    const uint32_t mins = (uint32_t)doc["manual_on_duration_minutes"].as<unsigned long>();
    durationSec = getManualTimedOnDurationFromMinutes(mins);
  } else if (doc.containsKey("manual_on_minutes")) {
    hasTimerField = true;
    const uint32_t mins = (uint32_t)doc["manual_on_minutes"].as<unsigned long>();
    durationSec = getManualTimedOnDurationFromMinutes(mins);
  } else if (doc.containsKey("manual_on_duration_min")) {
    hasTimerField = true;
    const uint32_t mins = (uint32_t)doc["manual_on_duration_min"].as<unsigned long>();
    durationSec = getManualTimedOnDurationFromMinutes(mins);
  } else if (doc.containsKey("manual_on_option")) {
    hasTimerField = true;
    const String option = doc["manual_on_option"].as<String>();
    parseManualTimedDurationToken(option, durationSec);
  }

  if (doc.containsKey("manual_timer_cancel") && doc["manual_timer_cancel"].as<bool>()) {
    cancelManualTimedOn(true, "MANUAL_TIMER_CANCEL");
    return true;
  }

  if (doc.containsKey("manual_on_cancel") && doc["manual_on_cancel"].as<bool>()) {
    cancelManualTimedOn(true, "MANUAL_TIMER_CANCEL");
    return true;
  }

  if (doc.containsKey("manual_on")) {
    hasTimerField = true;
    const bool reqOn = doc["manual_on"].as<bool>();
    if (!reqOn) {
      clearManualTimedOnState(true);
      stopStarterWithReason("MANUAL_OFF_CMD");
      pendingPublishStatus = true;
      return true;
    }
  }

  if (durationSec > 0) {
    startManualTimedOn(durationSec, "MANUAL_TIMER_ON");
    return true;
  }

  if (hasTimerField) {
    Serial.println("[MANUAL TIMER] Invalid duration, allowed: 10/20/40 min, 1/2/3 hour");
    return true;
  }

  return false;
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
    if (item.containsKey("off_hour") || item.containsKey("off_minute")) {
      scheduleEntries[index].offHour = constrain(item["off_hour"] | scheduleEntries[index].offHour, 0, 23);
      scheduleEntries[index].offMinute = constrain(item["off_minute"] | scheduleEntries[index].offMinute, 0, 59);
    } else if (item.containsKey("duration_min")) {
      uint16_t durationMin = (uint16_t)(item["duration_min"] | 60);
      if (durationMin == 0) {
        durationMin = 60;
      }
      applyScheduleDurationToEntry(scheduleEntries[index], durationMin);
    }
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
  if (setTime <= 0) {
    return false;
  }
  struct timeval tv = { setTime, 0 };
  settimeofday(&tv, NULL);
  rtcTimeValid = true;
  currentTimeSource = "MQTT_MANUAL";
  syncDs1307FromSystemTime(true);
  lastNtpSyncAttemptMs = millis();
  pendingPublishStatus = true;
  Serial.printf("[TIME] Set clock to %02d:%02d from MQTT\n", hour, minute);
  return true;
}

bool parseWiFiConfigCommand(const JsonDocument& doc) {
  const bool hasWiFiConfig =
    doc.containsKey("wifi_ssid") ||
    doc.containsKey("wifi_password") ||
    doc.containsKey("wifi_secondary_ssid") ||
    doc.containsKey("wifi_secondary_password");

  if (!hasWiFiConfig) {
    return false;
  }

  Serial.println("[WiFi] Ignored WiFi config command: primary fixed WiFi only");
  return true;
}

bool extractControlValueFromJson(const JsonVariantConst value, String& outCommand) {
  if (value.is<const char*>()) {
    outCommand = String(value.as<const char*>());
    return outCommand.length() > 0;
  }

  if (value.is<String>()) {
    outCommand = value.as<String>();
    return outCommand.length() > 0;
  }

  if (value.is<bool>()) {
    outCommand = value.as<bool>() ? "ON" : "OFF";
    return true;
  }

  if (value.is<int>() || value.is<long>() || value.is<unsigned int>() || value.is<unsigned long>()) {
    outCommand = value.as<long>() != 0 ? "ON" : "OFF";
    return true;
  }

  return false;
}

bool parseControlCommandFromJson(const JsonDocument& doc) {
  String cmd = "";
  const char* directKeys[] = {
    "command", "cmd", "control", "state", "switch", "relay", "motor", "motor_command",
    "alarm_state", "alarm", "e_alarm", "d_alarm"
  };
  const char* explicitModeKeys[] = {"mode_command", "set_mode", "mode_cmd"};

  for (size_t i = 0; i < (sizeof(directKeys) / sizeof(directKeys[0])); i++) {
    const char* key = directKeys[i];
    if (!doc.containsKey(key)) {
      continue;
    }

    if (extractControlValueFromJson(doc[key], cmd)) {
      handleControlMessage(cmd);
      return true;
    }
  }

  if (doc.containsKey("motor_on")) {
    cmd = doc["motor_on"].as<bool>() ? "ON" : "OFF";
    handleControlMessage(cmd);
    return true;
  }

  if (doc.containsKey("alarm_active")) {
    const bool active = doc["alarm_active"].as<bool>();
    cmd = active ? "E_ALARM_ON" : "E_ALARM_OFF";
    handleControlMessage(cmd);
    return true;
  }

  for (size_t i = 0; i < (sizeof(explicitModeKeys) / sizeof(explicitModeKeys[0])); i++) {
    const char* key = explicitModeKeys[i];
    if (!doc.containsKey(key)) {
      continue;
    }

    if (extractControlValueFromJson(doc[key], cmd)) {
      handleControlMessage(cmd);
      return true;
    }
  }

  if (doc.containsKey("control") && doc["control"].is<JsonObjectConst>()) {
    JsonObjectConst control = doc["control"].as<JsonObjectConst>();
    const char* nestedKeys[] = {"command", "cmd", "state", "alarm_state", "alarm", "e_alarm", "d_alarm"};
    for (size_t i = 0; i < (sizeof(nestedKeys) / sizeof(nestedKeys[0])); i++) {
      const char* key = nestedKeys[i];
      if (!control.containsKey(key)) {
        continue;
      }

      if (extractControlValueFromJson(control[key], cmd)) {
        handleControlMessage(cmd);
        return true;
      }
    }

    for (size_t i = 0; i < (sizeof(explicitModeKeys) / sizeof(explicitModeKeys[0])); i++) {
      const char* key = explicitModeKeys[i];
      if (!control.containsKey(key)) {
        continue;
      }

      if (extractControlValueFromJson(control[key], cmd)) {
        handleControlMessage(cmd);
        return true;
      }
    }

    if (control.containsKey("alarm_active")) {
      const bool active = control["alarm_active"].as<bool>();
      cmd = active ? "E_ALARM_ON" : "E_ALARM_OFF";
      handleControlMessage(cmd);
      return true;
    }
  }

  return false;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr(topic);
  String msg;
  msg.reserve(length);

  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("[MQTT RX] ");
  Serial.print(topicStr);
  Serial.print(" => ");
  if (msg.length() <= 180) {
    Serial.println(msg);
  } else {
    Serial.print(msg.substring(0, 180));
    Serial.println("...");
  }

  DynamicJsonDocument doc(512);
  DeserializationError err = deserializeJson(doc, msg);
  if (!err) {
    bool parsed = false;
    parsed |= parseScheduleCommand(doc);
    parsed |= parseStarterSequenceCommand(doc);
    parsed |= parsePhaseSequenceCommand(doc);
    parsed |= parseLowVoltageThresholdCommand(doc);
    parsed |= parseCurrentProtectionCommand(doc);
    parsed |= parseFaultConfigCommand(doc);
    parsed |= parseWiFiConfigCommand(doc);
    parsed |= parseTimeCommand(doc);
    parsed |= parseManualTimedOnCommand(doc);
    parsed |= parseControlCommandFromJson(doc);
    if (parsed) {
      return;
    }
  }

  if (topicStr == topicControl || topicStr == topicAllControl) {
    handleControlMessage(msg);
  }
}

bool connectWiFi() {
  const unsigned long nowMs = millis();
  const bool wasConnected = lastWiFiConnected;

  if (WiFi.status() == WL_CONNECTED) {
    if (!lastWiFiConnected) {
      Serial.println("[WiFi] connected");
      Serial.print("[WiFi] SSID: ");
      Serial.println(WiFi.SSID());
      Serial.print("[WiFi] IP: ");
      Serial.println(WiFi.localIP());
      setupTime();
      lastWiFiConnected = true;
      updateNetIndicator(true);
    }
    return true;
  }

  lastWiFiConnected = false;
  updateNetIndicator(false);

  if (wasConnected) {
    Serial.println("[WiFi] Connection lost, retrying fixed profile");
    lastWiFiConnectAttemptMs = 0;
  }

  if (lastWiFiConnectAttemptMs != 0 && (nowMs - lastWiFiConnectAttemptMs < WIFI_RECONNECT_INTERVAL_MS)) {
    return false;
  }

  lastWiFiConnectAttemptMs = nowMs;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(WIFI_RECONFIG_GUARD_MS);
  WiFi.begin(WIFI_FIXED_SSID, WIFI_FIXED_PASSWORD);
  Serial.print("[WiFi] reconnect start (fixed): ");
  Serial.println(WIFI_FIXED_SSID);
  return false;
}

void setupTime() {
  configTime(NTP_GMT_OFFSET_SEC, NTP_DAYLIGHT_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
  lastNtpSyncAttemptMs = millis();
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5000)) {
    char timeBuffer[64];
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.print("[TIME] NTP synced: ");
    Serial.println(timeBuffer);
    rtcTimeValid = true;
    currentTimeSource = "NTP";
    syncDs1307FromSystemTime(true);
  } else {
    Serial.println("[TIME] NTP sync failed");
    // Keep previously valid time if already restored from DS1307 or set manually.
    rtcTimeValid = isRtcTimeValid();
  }
}

bool isRtcTimeValid() {
  time_t now = time(nullptr);
  // Reject epoch-like values; 2024-01-01 is a safe lower bound for this project.
  return now > 1704067200;
}

void setupExternalRtc() {
  Wire.begin(DS1307_SDA_PIN, DS1307_SCL_PIN);
  if (!externalRtc.begin()) {
    ds1307Available = false;
    Serial.println("[RTC] DS1307 not detected on I2C");
    return;
  }

  ds1307Available = true;
  Serial.print("[RTC] DS1307 detected on SDA=");
  Serial.print(DS1307_SDA_PIN);
  Serial.print(" SCL=");
  Serial.println(DS1307_SCL_PIN);

  if (!externalRtc.isrunning()) {
    Serial.println("[RTC] DS1307 clock halted or not initialized");
    Serial.println("[RTC] Check DS1307 coin cell battery (CR2032) and module wiring");
  }
}

void restoreRtcFromDs1307() {
  if (!ds1307Available) {
    return;
  }

  const DateTime dt = externalRtc.now();
  if (dt.year() < 2024) {
    Serial.println("[RTC] DS1307 time invalid, waiting for NTP/manual set");
    return;
  }

  struct tm timeinfo;
  memset(&timeinfo, 0, sizeof(timeinfo));
  timeinfo.tm_year = dt.year() - 1900;
  timeinfo.tm_mon = dt.month() - 1;
  timeinfo.tm_mday = dt.day();
  timeinfo.tm_hour = dt.hour();
  timeinfo.tm_min = dt.minute();
  timeinfo.tm_sec = dt.second();

  const time_t epoch = mktime(&timeinfo);
  if (epoch <= 1704067200) {
    return;
  }

  struct timeval tv = { epoch, 0 };
  settimeofday(&tv, NULL);
  rtcTimeValid = true;
  currentTimeSource = "RTC_DS1307";
  Serial.println("[TIME] Restored from DS1307");
}

void syncDs1307FromSystemTime(bool forceSync = false) {
  if (!ds1307Available || !isRtcTimeValid()) {
    return;
  }

  const unsigned long nowMs = millis();
  if (!forceSync && lastDs1307SyncMs != 0 && (nowMs - lastDs1307SyncMs < DS1307_SYNC_INTERVAL_MS)) {
    return;
  }

  lastDs1307SyncMs = nowMs;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 200)) {
    return;
  }

  externalRtc.adjust(DateTime(
    timeinfo.tm_year + 1900,
    timeinfo.tm_mon + 1,
    timeinfo.tm_mday,
    timeinfo.tm_hour,
    timeinfo.tm_min,
    timeinfo.tm_sec
  ));

  const time_t nowEpoch = time(nullptr);
  Serial.print("[RTC] DS1307 synced from system time, epoch=");
  Serial.println((unsigned long)nowEpoch);
}

void restoreRtcFromPreferences() {
  if (isRtcTimeValid()) {
    return;
  }

  rtcPreferences.begin("rtc", true);
  const uint32_t savedEpoch = rtcPreferences.getULong("last_epoch", 0);
  rtcPreferences.end();

  if (savedEpoch <= 1704067200UL) {
    return;
  }

  struct timeval tv = { (time_t)savedEpoch, 0 };
  settimeofday(&tv, NULL);
  rtcTimeValid = isRtcTimeValid();
  if (rtcTimeValid) {
    currentTimeSource = "FLASH_SNAPSHOT";
    Serial.print("[TIME] RTC restored from flash snapshot: ");
    Serial.println((unsigned long)savedEpoch);
  }
}

void saveRtcSnapshotIfNeeded() {
  if (!isRtcTimeValid()) {
    return;
  }

  const unsigned long nowMs = millis();
  if (lastRtcSnapshotSaveMs != 0 && (nowMs - lastRtcSnapshotSaveMs < RTC_SNAPSHOT_SAVE_MS)) {
    return;
  }

  lastRtcSnapshotSaveMs = nowMs;
  const uint32_t nowEpoch = (uint32_t)time(nullptr);
  rtcPreferences.begin("rtc", false);
  rtcPreferences.putULong("last_epoch", nowEpoch);
  rtcPreferences.end();
}

String getLocalTimeText() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 50)) {
    return String("TIME NOT SYNCED");
  }

  char timeBuffer[32];
  strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeBuffer);
}

void printTimeToSerial() {
  Serial.print("[TIME] ");
  Serial.print(getLocalTimeText());
  Serial.print(" | Source: ");
  Serial.print(currentTimeSource);
  Serial.print(" | RTC valid: ");
  Serial.print(isRtcTimeValid() ? "YES" : "NO");

  if (ds1307Available) {
    const DateTime dt = externalRtc.now();
    char rtcBuf[24];
    snprintf(rtcBuf, sizeof(rtcBuf), "%04d-%02d-%02d %02d:%02d:%02d",
             dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
    Serial.print(" | DS1307: ");
    Serial.print(rtcBuf);
  }

  Serial.println();
}

#if USE_BLUETOOTH_TIME_DISPLAY
void broadcastTimeToBluetooth() {
  if (!SerialBT.hasClient()) {
    return;
  }

  SerialBT.print("[TIME] ");
  SerialBT.print(getLocalTimeText());
  SerialBT.print(" | RTC valid: ");
  SerialBT.println(isRtcTimeValid() ? "YES" : "NO");
}

void handleBluetoothCommand() {
  if (!SerialBT.available()) {
    return;
  }

  String msg = SerialBT.readStringUntil('\n');
  msg.trim();
  msg.toUpperCase();

  if (msg == "TIME" || msg == "GET TIME") {
    SerialBT.print("[TIME] ");
    SerialBT.println(getLocalTimeText());
    return;
  }

  if (msg == "STATUS") {
    SerialBT.print("[STATUS] ");
    SerialBT.print(autoModeEnabled ? "AUTO" : "MANUAL");
    SerialBT.print(" | RTC valid: ");
    SerialBT.println(isRtcTimeValid() ? "YES" : "NO");
    return;
  }

  SerialBT.println("Commands: TIME, STATUS");
}
#endif

void maintainTimeSync() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  const unsigned long nowMs = millis();
  const bool needsImmediateNtp = (currentTimeSource != "NTP");

  if (needsImmediateNtp) {
    if (lastNtpSyncAttemptMs != 0 && (nowMs - lastNtpSyncAttemptMs < NTP_RETRY_WHEN_UNSYNCED_MS)) {
      return;
    }
    setupTime();
    return;
  }

  if (lastNtpSyncAttemptMs != 0 && (nowMs - lastNtpSyncAttemptMs < NTP_RESYNC_INTERVAL_MS)) {
    return;
  }

  setupTime();
}

void maintainOfflineRtcSync() {
  if (WiFi.status() == WL_CONNECTED || !ds1307Available) {
    return;
  }

  const unsigned long nowMs = millis();
  if (lastDs1307RestoreMs != 0 && (nowMs - lastDs1307RestoreMs < DS1307_RESTORE_INTERVAL_MS)) {
    return;
  }

  lastDs1307RestoreMs = nowMs;
  restoreRtcFromDs1307();
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
  topicControl = "relite2/motor/" + moduleId4 + "/control";
  topicStatus = "relite2/motor/" + moduleId4 + "/status";
  topicTelemetry = "relite2/motor/" + moduleId4 + "/telemetry";
  topicAllControl = "relite2/motor/all/control";
  topicAllData = "relite2/sensors/all";
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("====================================");
  Serial.println(" ESP Adhana Motor Updated");
  Serial.println(" One motor per board on GPIO12");
  Serial.println("====================================");

  setupExternalRtc();
  restoreRtcFromDs1307();
  restoreRtcFromPreferences();
  syncDs1307FromSystemTime(true);

#if USE_BLUETOOTH_TIME_DISPLAY
  if (SerialBT.begin("ESP32-Adhana-Time")) {
    SerialBT.setTimeout(20);
    Serial.println("[BT] Bluetooth Serial started as ESP32-Adhana-Time");
  } else {
    Serial.println("[BT] Bluetooth Serial start failed");
  }
#endif

  setupTopics();
  bootMs = millis();

  pinMode(RELAY_PIN_12, OUTPUT);
  pinMode(RELAY_PIN_14, OUTPUT);
  pinMode(RELAY_PIN_27, OUTPUT);
  pinMode(MODE_LED_PIN, OUTPUT);
  pinMode(NET_LED_PIN, OUTPUT);
  pinMode(SENSOR_SEL_A_PIN, OUTPUT);
  pinMode(SENSOR_SEL_B_PIN, OUTPUT);
  digitalWrite(SENSOR_SEL_A_PIN, LOW);
  digitalWrite(SENSOR_SEL_B_PIN, LOW);
  updateNetIndicator(false);

  autoModeEnabled = false;
  lastHardwareAutoMode = true;
  modeCommandOverrideActive = true;
  updateModeIndicator();

  loadScheduleFromPreferences();
  loadStarterSequenceFromPreferences();
  loadModeOverrideFromPreferences();
  loadAlarmWindowStateFromPreferences();
  loadPhaseSequenceFromPreferences();
  loadSwitchHistoryFromPreferences();
  loadManualTimedOnFromPreferences();

  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_X_LOGICAL_PIN, ADC_11db);
  analogSetPinAttenuation(SENSOR_Y_LOGICAL_PIN, ADC_11db);

  stopStarter();
  resetModeDecisionWindow();

  if (manualTimedOnActive) {
    modeCommandOverrideActive = true;
    autoModeEnabled = false;
    updateModeIndicator();
    startStarterOnSequenceWithReason("MANUAL_TIMER_RESTORE");
    saveModeOverrideToPreferences();
    pendingPublishStatus = true;
    Serial.println("[MANUAL TIMER] Restored active manual timer from flash");
  }

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
  Serial.print("[Node] NET LED GPIO: ");
  Serial.print(NET_LED_PIN);
  Serial.print(" (ON means ");
  Serial.print(NET_LED_ON_WHEN_CONNECTED ? "WiFi connected" : "WiFi disconnected");
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

  connectWiFi();
  updatePhaseSequenceEstimator();

  rtcTimeValid = isRtcTimeValid();
  if (!rtcTimeValid || (millis() - lastNtpSyncAttemptMs >= NTP_RESYNC_INTERVAL_MS)) {
    maintainTimeSync();
  }
  maintainOfflineRtcSync();

  syncDs1307FromSystemTime();
  saveRtcSnapshotIfNeeded();

#if USE_BLUETOOTH_TIME_DISPLAY
  handleBluetoothCommand();
  if (SerialBT.hasClient() && (millis() - lastBluetoothTimeMs >= BLUETOOTH_TIME_BROADCAST_MS)) {
    lastBluetoothTimeMs = millis();
    broadcastTimeToBluetooth();
  }
#endif

  updateModeDecisionWindow();

  if (!mqttClient.connected()) {
    if (WiFi.status() == WL_CONNECTED) {
      reconnectMqtt();
    }
  } else {
    mqttClient.loop();
    if (pendingPublishStatus) {
      pendingPublishStatus = false;
      publishStatus(true);
    }
  }

  applyX1FaultSafety();
  applyManualTimedOnLogic();
  applyScheduleLogic();
  applyAutoModeLogic();
  applyProtectionFaultSafety();
  applyPostOffCurrentMonitor();

  updateStarterRelays();

  const unsigned long now = millis();

  if (now - lastSerialTimePrintMs >= SERIAL_TIME_PRINT_MS) {
    lastSerialTimePrintMs = now;
    printTimeToSerial();
  }

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

