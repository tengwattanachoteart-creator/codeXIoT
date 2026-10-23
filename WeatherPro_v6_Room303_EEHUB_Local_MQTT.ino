/*
 Weather Pro v6 - local MQTT -> Home Assistant Mosquitto -> Node-RED
 ESP8266 NodeMCU; separate project for each room. Use a 2.4GHz WiFi.
 MQTT over private LAN port 1883 (not TLS); NEVER port-forward 1883.
 Sensor 5V rail via INA219 is not the raw 18650 cell voltage.
 Deep sleep D0 GPIO16 -> RST; unplug this jumper if it interferes with uploads.
 Do not power USB and external shield 5V simultaneously without isolation.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <math.h>

// ================= USER CONFIG: EDIT THESE ONLY ===================
#define ROOM_NUMBER                  303
#define ROOM_LABEL                   "EEHUB"
const char* WIFI_SSID = "Chamber of Secrets";
const char* WIFI_PASS = "kmutt10140";
const char* MQTT_HOST = "192.168.0.3";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "active";
const char* MQTT_PASS = "learning";
// ================================================================

#define REPORT_INTERVAL_MINUTES      5 // 301 diagnostic = 1min; 302/303 = 5min
#define USE_DEEP_SLEEP              0  // all 3 stations; connect D0(GPIO16) to RST

const float TEMP_ALERT_C = 35.0f;
const float HUMIDITY_ALERT_PERCENT = 80.0f;
// Temporarily sample every ~1 min; keep SQLite raw samples, aggregate graph in 5-min bins.
const uint8_t SENSOR_INIT_ATTEMPTS = 3;
const uint8_t SENSOR_READ_ATTEMPTS = 3;
// ======================================================================

#if ROOM_NUMBER != 301 && ROOM_NUMBER != 302 && ROOM_NUMBER != 303
  #error "ROOM_NUMBER must be 301, 302, or 303"
#endif
#if REPORT_INTERVAL_MINUTES < 1 || REPORT_INTERVAL_MINUTES > 5
  #error "REPORT_INTERVAL_MINUTES must be from 1 to 5"
#endif
#if USE_DEEP_SLEEP != 0 && USE_DEEP_SLEEP != 1
  #error "USE_DEEP_SLEEP must be 0 or 1"
#endif

const uint32_t REPORT_INTERVAL_MS = (uint32_t)REPORT_INTERVAL_MINUTES * 60000UL;
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 18000UL;
const uint32_t MQTT_RETRY_INTERVAL_MS = 10000UL;
const uint32_t WIFI_RETRY_INTERVAL_MS = 15000UL;
const char* TOPIC_ROOT = "building/floor3";

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
Adafruit_BME280 bme;
Adafruit_INA219 ina(0x40);

bool ahtOK = false;
bool bmpOK = false;
bool bmeOK = false;
bool inaOK = false;
String roomName;
String baseTopic;
String topicState;
String mqttClientId;
uint32_t lastReportAt = 0;
uint32_t lastWiFiTryAt = 0;
uint32_t lastMqttTryAt = 0;
uint32_t sampleCount = 0;

struct SensorReading {
  float temperature = NAN;
  float humidity = NAN;
  float pressure = NAN;
  float voltage = NAN;        // VIN- (load side) BUS voltage, V
  float sourceVoltage = NAN;  // estimated VIN+ voltage, V
  float shuntMV = NAN;
  float currentMA = NAN;      // SIGNED; negative indicates reversed sense
  float powerMW = NAN;       // instantaneous power while ESP is awake
  float rssiDBm = NAN;
  // OK means a complete, valid MEASUREMENT, not just successful begin().
  bool ahtReadOK = false;
  bool bmpReadOK = false;
  bool bmeReadOK = false;
  bool inaReadOK = false;
  bool recoveryUsed = false;
};

String numOrNull(float n, uint8_t decimals = 2) {
  return isfinite(n) ? String(n, decimals) : String("null");
}

uint8_t readChipId(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0xD0);                 // BMP280/BME280 CHIP_ID register
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(address, (uint8_t)1) != 1) return 0;
  return (uint8_t)Wire.read();
}

// AHT20/BMP280 share the SDA/SCL bus with INA219. Sharing I2C is normal;
// devices are selected by their separate I2C addresses.
void initEnvironmentSensors() {
  ahtOK = false;
  bmpOK = false;
  bmeOK = false;

  for (uint8_t attempt = 1; attempt <= SENSOR_INIT_ATTEMPTS && !ahtOK; ++attempt) {
    ahtOK = aht.begin();
    Serial.printf("[INIT] AHT20 attempt %u/%u: %s\n", attempt,
                  SENSOR_INIT_ATTEMPTS, ahtOK ? "OK" : "FAIL");
    if (!ahtOK) delay(250);
  }

  for (uint8_t index = 0; index < 2 && !bmpOK && !bmeOK; ++index) {
    uint8_t address = (index == 0) ? 0x77 : 0x76;
    uint8_t chipId = readChipId(address);
    Serial.printf("[I2C] Chip at 0x%02X ID=0x%02X\n", address, chipId);
    if (chipId == 0x60) {
      for (uint8_t attempt = 1; attempt <= SENSOR_INIT_ATTEMPTS && !bmeOK; ++attempt) {
        bmeOK = bme.begin(address, &Wire);
        Serial.printf("[INIT] BME280 0x%02X attempt %u: %s\n", address,
                      attempt, bmeOK ? "OK" : "FAIL");
        if (!bmeOK) delay(250);
      }
    } else if (chipId == 0x58 || chipId == 0x56 || chipId == 0x57) {
      for (uint8_t attempt = 1; attempt <= SENSOR_INIT_ATTEMPTS && !bmpOK; ++attempt) {
        bmpOK = bmp.begin(address);
        Serial.printf("[INIT] BMP280 0x%02X attempt %u: %s\n", address,
                      attempt, bmpOK ? "OK" : "FAIL");
        if (!bmpOK) delay(250);
      }
    }
  }
}

void initSensors() {
  Wire.begin(D2, D1);          // SDA=D2(GPIO4), SCL=D1(GPIO5)
  Wire.setClock(100000);      // 100 kHz: conservative for shared wiring
  delay(300);                 // allow sensor supplies to settle after wake

  for (uint8_t i = 0; i < 4; ++i) {
    uint8_t addr = (i == 0) ? 0x38 : (i == 1) ? 0x76 : (i == 2) ? 0x77 : 0x40;
    Wire.beginTransmission(addr);
    bool present = (Wire.endTransmission() == 0);
    Serial.printf("[I2C SCAN] 0x%02X %s\n", addr, present ? "ACK" : "NO-ACK");
  }

  initEnvironmentSensors();
  inaOK = false;
  for (uint8_t attempt = 1; attempt <= SENSOR_INIT_ATTEMPTS && !inaOK; ++attempt) {
    inaOK = ina.begin();
    Serial.printf("[INIT] INA219 attempt %u: %s\n", attempt,
                  inaOK ? "OK" : "FAIL");
    if (!inaOK) delay(250);
  }
  if (inaOK) ina.setCalibration_32V_2A();

  Serial.printf("[I2C INIT] AHT20=%s BMP280=%s BME280=%s INA219=%s\n",
                ahtOK ? "OK" : "MISSING", bmpOK ? "OK" : "MISSING",
                bmeOK ? "OK" : "MISSING", inaOK ? "OK" : "MISSING");
}

SensorReading readSensors() {
  SensorReading r;
  // AHT20 can ACK and initialize but return invalid humidity/temperature.
  // Retry the actual MEASUREMENT, not just begin().
  if (ahtOK) {
    for (uint8_t attempt = 1; attempt <= SENSOR_READ_ATTEMPTS; ++attempt) {
      sensors_event_t humidity, temperature;
      bool eventOK = aht.getEvent(&humidity, &temperature);
      bool tempValid = eventOK && isfinite(temperature.temperature) &&
                       temperature.temperature >= -40.0f && temperature.temperature <= 85.0f;
      bool humidValid = eventOK && isfinite(humidity.relative_humidity) &&
                        humidity.relative_humidity >= 0.0f && humidity.relative_humidity <= 100.0f;
      if (tempValid) r.temperature = temperature.temperature;
      if (humidValid) r.humidity = humidity.relative_humidity;
      r.ahtReadOK = tempValid && humidValid;
      Serial.printf("[READ] AHT20 %u/%u event=%d temp=%s humidity=%s\n",
                    attempt, SENSOR_READ_ATTEMPTS, eventOK,
                    tempValid ? "OK" : "BAD", humidValid ? "OK" : "BAD");
      if (r.ahtReadOK) break;
      delay(250);
    }
  }

  if (bmeOK) {
    for (uint8_t attempt = 1; attempt <= SENSOR_READ_ATTEMPTS; ++attempt) {
      float t = bme.readTemperature();
      float h = bme.readHumidity();
      float p = bme.readPressure() / 100.0f;
      bool tv = isfinite(t) && t > -40.0f && t < 85.0f;
      bool hv = isfinite(h) && h >= 0.0f && h <= 100.0f;
      bool pv = isfinite(p) && p > 300.0f && p < 1200.0f;
      if (!isfinite(r.temperature) && tv) r.temperature = t;
      if (!isfinite(r.humidity) && hv) r.humidity = h;
      if (pv) r.pressure = p;
      r.bmeReadOK = tv && hv && pv;
      Serial.printf("[READ] BME280 %u/%u temp=%s humidity=%s pressure=%s\n",
                    attempt, SENSOR_READ_ATTEMPTS,
                    tv ? "OK" : "BAD", hv ? "OK" : "BAD", pv ? "OK" : "BAD");
      if (r.bmeReadOK) break;
      delay(250);
    }
  } else if (bmpOK) {
    for (uint8_t attempt = 1; attempt <= SENSOR_READ_ATTEMPTS; ++attempt) {
      float t = bmp.readTemperature();
      float p = bmp.readPressure() / 100.0f;
      bool tv = isfinite(t) && t > -40.0f && t < 85.0f;
      bool pv = isfinite(p) && p > 300.0f && p < 1200.0f;
      if (!isfinite(r.temperature) && tv) r.temperature = t;
      if (pv) r.pressure = p;
      r.bmpReadOK = tv && pv;
      Serial.printf("[READ] BMP280 %u/%u temp=%s pressure=%s\n",
                    attempt, SENSOR_READ_ATTEMPTS,
                    tv ? "OK" : "BAD", pv ? "OK" : "BAD");
      if (r.bmpReadOK) break;
      delay(250);
    }
  }

  if (inaOK) {
    float busV = ina.getBusVoltage_V();
    float shuntMV = ina.getShuntVoltage_mV();
    float currentMA = ina.getCurrent_mA();
    float powerMW = ina.getPower_mW();
    bool bv = isfinite(busV) && busV >= 0.0f && busV <= 26.0f;
    bool sv = isfinite(shuntMV);
    bool cv = isfinite(currentMA);
    bool pv = isfinite(powerMW);
    if (bv) r.voltage = busV;
    if (sv) r.shuntMV = shuntMV;
    if (cv) r.currentMA = currentMA;
    if (pv) r.powerMW = powerMW;
    if (bv && sv) r.sourceVoltage = busV + shuntMV / 1000.0f;
    r.inaReadOK = bv && sv && cv && pv;
    Serial.printf("[READ] INA219 voltage=%s current=%s power=%s\n",
                  bv ? "OK" : "BAD", cv ? "OK" : "BAD", pv ? "OK" : "BAD");
  }
  if (WiFi.status() == WL_CONNECTED) r.rssiDBm = WiFi.RSSI();
  return r;
}

bool ensureWiFi(uint32_t timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) return true;
  Serial.println("[WiFi] Connecting (2.4 GHz WPA2-Personal required)...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  const uint32_t startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startedAt < timeoutMs) {
    delay(250);
    yield();
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] IP: "); Serial.println(WiFi.localIP());
    return true;
  }
  Serial.println("[WiFi] FAILED. Check SSID/password, 2.4 GHz, router.");
  return false;
}

bool ensureMQTT() {
  if (mqtt.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  // Each room has a unique client ID based on the ESP chip ID.
  if (strlen(MQTT_PASS) < 8 || String(MQTT_PASS) == "YOUR_MQTT_PASSWORD") {
    Serial.println("[MQTT] Fill in MQTT password in USER CONFIG");
    return false;
  }
  const bool ok = mqtt.connect(mqttClientId.c_str(), MQTT_USER, MQTT_PASS);
  if (ok) Serial.println("[MQTT] CONNECTED to Home Assistant Mosquitto");
  else Serial.printf("[MQTT] Failed rc=%d. Check HA user/password/IP/1883.\n", mqtt.state());
  return ok;
}

String classifyAlert(const SensorReading& r) {
  if (!isfinite(r.temperature) || !isfinite(r.humidity) ||
      !isfinite(r.pressure) || !isfinite(r.currentMA))
    return "sensor_error";
  if (r.currentMA < -5.0f) return "check_ina219_direction";
  if (r.temperature > TEMP_ALERT_C && r.humidity > HUMIDITY_ALERT_PERCENT)
    return "high_temperature_and_humidity";
  if (r.temperature > TEMP_ALERT_C) return "high_temperature";
  if (r.humidity > HUMIDITY_ALERT_PERCENT) return "high_humidity";
  return "normal";
}

String makeStateJson(const SensorReading& r) {
  String json;
  json.reserve(900);
  json += "{\"room\":\"" + roomName + "\"";
  json += ",\"location\":\"" + String(ROOM_LABEL) + "\"";
  json += ",\"room_label\":\"" + String(ROOM_LABEL) + "\"";
  json += ",\"temperature\":" + numOrNull(r.temperature);
  json += ",\"humidity\":" + numOrNull(r.humidity);
  json += ",\"pressure\":" + numOrNull(r.pressure);
  json += ",\"voltage\":" + numOrNull(r.voltage);
  // The existing Weather Pro Dashboard uses 'current'/'power'.
  json += ",\"current\":" + numOrNull(r.currentMA);
  json += ",\"power\":" + numOrNull(r.powerMW);
  // The SQLite Logger uses 'current_ma'/'power_mw'/'sleep_mode'.
  json += ",\"current_ma\":" + numOrNull(r.currentMA);
  json += ",\"power_mw\":" + numOrNull(r.powerMW);
  json += ",\"source_voltage\":" + numOrNull(r.sourceVoltage);
  json += ",\"shunt_mv\":" + numOrNull(r.shuntMV);
  json += ",\"wifi_rssi\":" + numOrNull(r.rssiDBm, 0);
  const char* mode = USE_DEEP_SLEEP ? "deep_sleep" : "normal";
  json += ",\"mode\":\"" + String(mode) + "\"";
  json += ",\"sleep_mode\":\"" + String(mode) + "\"";
  json += ",\"alert\":\"" + classifyAlert(r) + "\"";
  // *_ok flags now describe VALID READINGS, not merely begin() success.
  json += ",\"aht_ok\":" + String(r.ahtReadOK ? "true" : "false");
  json += ",\"bmp_ok\":" + String(r.bmpReadOK ? "true" : "false");
  json += ",\"bme_ok\":" + String(r.bmeReadOK ? "true" : "false");
  json += ",\"ina_ok\":" + String(r.inaReadOK ? "true" : "false");
  json += ",\"aht_init_ok\":" + String(ahtOK ? "true" : "false");
  json += ",\"bmp_init_ok\":" + String(bmpOK ? "true" : "false");
  json += ",\"bme_init_ok\":" + String(bmeOK ? "true" : "false");
  json += ",\"ina_init_ok\":" + String(inaOK ? "true" : "false");
  json += ",\"sensor_recovery_used\":" + String(r.recoveryUsed ? "true" : "false");
  json += ",\"uptime_ms\":" + String(millis());
  json += ",\"sample_count\":" + String(sampleCount);
  json += "}";
  return json;
}

void publishReading() {
  SensorReading r = readSensors();
  // One bounded recovery attempt per wake. Never fabricate missing readings.
  if (!r.ahtReadOK || (!r.bmpReadOK && !r.bmeReadOK)) {
    Serial.println("[RECOVER] Environmental read failed; re-init + re-read once");
    initEnvironmentSensors();
    delay(300);
    SensorReading retryReading = readSensors();
    retryReading.recoveryUsed = true;
    r = retryReading;
  }
  Serial.printf("[HEALTH] AHT=%s BMP=%s BME=%s INA=%s recovery=%s\n",
                r.ahtReadOK ? "OK" : "FAIL", r.bmpReadOK ? "OK" : "FAIL",
                r.bmeReadOK ? "OK" : "FAIL", r.inaReadOK ? "OK" : "FAIL",
                r.recoveryUsed ? "YES" : "NO");
  ++sampleCount;
  bool wifiOK = (WiFi.status() == WL_CONNECTED);
  if (!wifiOK) wifiOK = ensureWiFi(WIFI_CONNECT_TIMEOUT_MS);
  if (wifiOK) r.rssiDBm = WiFi.RSSI();

  // Deep-sleep mode: allow a short retry before abandoning this cycle.
  bool mqttOK = false;
  if (wifiOK) {
    mqttOK = ensureMQTT();
    if (!mqttOK && USE_DEEP_SLEEP) {
      delay(750);
      mqttOK = ensureMQTT();
    }
  }

  String stateJson = makeStateJson(r);
  Serial.println("\n=== SENSOR READING ===");
  Serial.println(stateJson);
  const String alert = classifyAlert(r);
  Serial.println("[ALERT] " + alert);

  if (mqttOK) {
    // State is NOT retained: reboot/reconnect will NOT replay stale rows.
    const bool sent = mqtt.publish(topicState.c_str(), stateJson.c_str(), false);
    if (sent) {
      Serial.println("[MQTT] state queued into TCP (QoS 0; no broker PUBACK) -> " + topicState);
    } else {
      Serial.println("[MQTT] STATE publish FAILED (not saved offline)");
    }
    mqtt.loop();
    delay(200);
  } else {
    Serial.println("[MQTT] Offline: sample NOT delivered. No flash queue configured.");
  }
  if (inaOK && isfinite(r.currentMA) && r.currentMA < -5.0f) {
    Serial.println("[WARN] INA219 current is negative. Verify VIN+ -> VIN- power-flow direction.");
  }
}

void enterDeepSleep() {
  if (mqtt.connected()) {
    // PubSubClient publishes at QoS 0; a successful publish is not a broker PUBACK.
    // Give TCP time to transmit before closing the connection and sleeping.
    mqtt.loop();
    delay(350);
    mqtt.disconnect();
  }
  // Aim for the FULL cycle to take REPORT_INTERVAL_MINUTES including boot/WiFi.
  const uint32_t elapsedMs = millis();
  const uint32_t sleepMs = (elapsedMs < REPORT_INTERVAL_MS - 1000UL)
                               ? (REPORT_INTERVAL_MS - elapsedMs)
                               : 1000UL;
  Serial.printf("[SLEEP] %lu ms; wire D0(GPIO16) -> RST for auto wake\n",
                (unsigned long)sleepMs);
  Serial.flush();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  ESP.deepSleep((uint64_t)sleepMs * 1000ULL, WAKE_RF_DEFAULT);
  delay(1000); // fallback if deep sleep fails
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n====== WEATHER PRO ESP8266 ======");
  roomName = "room" + String(ROOM_NUMBER);
  baseTopic = String(TOPIC_ROOT) + "/" + roomName;
  topicState = baseTopic + "/state";
  mqttClientId = "wp-" + roomName + "-" + String(ESP.getChipId(), HEX);
  Serial.printf("Room %d [%s] | interval %d min | mode %s\n",
                ROOM_NUMBER, ROOM_LABEL, REPORT_INTERVAL_MINUTES,
                USE_DEEP_SLEEP ? "deep_sleep" : "normal");
  Serial.println("[MQTT] Target: " + String(MQTT_HOST) + ":" + String(MQTT_PORT));
  Serial.println("[MQTT] Topic: " + topicState);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(20);
  mqtt.setSocketTimeout(10);

  initSensors();
  delay(500);
  publishReading();
  lastReportAt = millis();
  if (USE_DEEP_SLEEP) enterDeepSleep();
}

void loop() {
#if USE_DEEP_SLEEP
  delay(500); // Normally never reached: setup entered deep sleep.
#else
  const uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED && now - lastWiFiTryAt >= WIFI_RETRY_INTERVAL_MS) {
    lastWiFiTryAt = now;
    ensureWiFi(6000);
  }
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected() &&
      now - lastMqttTryAt >= MQTT_RETRY_INTERVAL_MS) {
    lastMqttTryAt = now;
    ensureMQTT();
  }
  if (mqtt.connected()) mqtt.loop();
  if (millis() - lastReportAt >= REPORT_INTERVAL_MS) {
    publishReading();
    lastReportAt = millis();
  }
  delay(25);
#endif
}
