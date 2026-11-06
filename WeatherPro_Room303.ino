/*
 Weather Pro v6.4 | 301/302/303 | DUAL WIFI + PER-ROOM DEEP SLEEP
 ESP8266 NodeMCU / AHT20 / BMP280 or BME280 / INA219
 Use each room's own sketch. Edit settings in USER CONFIG.

 - USE_DEEP_SLEEP = 1: wake -> measure -> publish -> peripheral low-power -> ESP deep sleep.
 - USE_DEEP_SLEEP = 0: stays awake; peripherals are parked between measurements.
 - A second 2.4 GHz WiFi network is optional and can be edited below.
 - MQTT_HOST_WIFI1 and MQTT_HOST_WIFI2 may be different IPs if the LANs differ.
 - battery_voltage is valid ONLY if INA219 VIN+ measures the battery-positive source.
 - Sensor registers are parked, but sensor VCC is NOT physically switched off.
 - D0(GPIO16) -> RST is needed only for timed wake from deep sleep.
 - Avoid simultaneous USB and external 5V supply without correct power isolation.
 - Keep MQTT port 1883 on trusted/private LAN; do not expose it on the internet.
*/

/*
 Weather Pro v6.3 - SENSOR LOW-POWER + BATTERY VOLTAGE
 ESP8266 NodeMCU; keep MQTT payload and topic compatible with original flow.
 MQTT over private LAN port 1883 (not TLS); NEVER port-forward 1883.
 Battery positive -> INA219 VIN+; INA219 VIN- -> ESP baseboard supply.
 battery_voltage = bus voltage at VIN- + shunt voltage (VIN+ minus VIN-).
 This represents battery voltage only while VIN+ is connected directly to battery positive.
 Deep sleep D0 GPIO16 -> RST; unplug this jumper if it interferes with uploads.
 Do not power USB and external shield 5V simultaneously without isolation.
*/

// TEST FIRST WITH USB ONLY AND USE_DEEP_SLEEP=0. Re-enable later.
// DO NOT wire battery shield output and USB at the same time without safe power isolation.
// Credentials below are retained from your uploaded sketch; check them for the new broker.

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

// =================== USER CONFIG: EDIT HERE ========================
#define ROOM_NUMBER  303          // 301, 302 or 303
#define ROOM_LABEL   "EEHUB"

// WiFi 1: existing network. ESP8266 supports 2.4 GHz WiFi only.
const char* WIFI1_SSID = "Chamber of Secrets";
const char* WIFI1_PASS = "kmutt10140";

// WiFi 2: fill in BOTH values. Leave placeholder SSID to disable WiFi 2.
const char* WIFI2_SSID = "active";
const char* WIFI2_PASS = "learning";

// MQTT broker reachable from EACH WiFi. Change WiFi2 IP if it uses another LAN.
const char* MQTT_HOST_WIFI1 = "10.39.66.100";
const char* MQTT_HOST_WIFI2 = "10.39.66.100";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "staff";
const char* MQTT_PASS = "kmutt10140";

#define REPORT_INTERVAL_MINUTES 5  // 1 to 5 minutes
#define USE_DEEP_SLEEP 0         // 1 = deep sleep, 0 = continuously powered
#define BATTERY_AT_VIN_PLUS 0     // 1 ONLY when VIN+ is battery positive
// ====================================================================


const float TEMP_ALERT_C = 35.0f;
const float HUMIDITY_ALERT_PERCENT = 80.0f;
// Target full wake/sleep cycle: REPORT_INTERVAL_MINUTES (currently 5 minutes).
const uint8_t SENSOR_INIT_ATTEMPTS = 3;
const uint8_t SENSOR_READ_ATTEMPTS = 3;
const uint8_t I2C_SDA_GPIO = 4; // NodeMCU D2
const uint8_t I2C_SCL_GPIO = 5; // NodeMCU D1
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

uint8_t connectedWiFiIndex = 0;
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
uint8_t environmentAddress = 0;
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
  environmentAddress = 0;

  for (uint8_t attempt = 1; attempt <= SENSOR_INIT_ATTEMPTS && !ahtOK; ++attempt) {
    ahtOK = aht.begin(&Wire);
    Serial.printf("[INIT] AHT20 attempt %u/%u: %s\n", attempt,
                  SENSOR_INIT_ATTEMPTS, ahtOK ? "OK" : "FAIL");
    if (!ahtOK) delay(400);
  }

  for (uint8_t index = 0; index < 2 && !bmpOK && !bmeOK; ++index) {
    uint8_t address = (index == 0) ? 0x77 : 0x76;
    uint8_t chipId = readChipId(address);
    Serial.printf("[I2C] Chip at 0x%02X ID=0x%02X\n", address, chipId);
    if (chipId == 0x60) {
      for (uint8_t attempt = 1; attempt <= SENSOR_INIT_ATTEMPTS && !bmeOK; ++attempt) {
        bmeOK = bme.begin(address, &Wire);
        if (bmeOK) {
          environmentAddress = address;
          bme.setSampling(Adafruit_BME280::MODE_FORCED,
                          Adafruit_BME280::SAMPLING_X2,
                          Adafruit_BME280::SAMPLING_X16,
                          Adafruit_BME280::SAMPLING_X1,
                          Adafruit_BME280::FILTER_OFF);
        }
        Serial.printf("[INIT] BME280 0x%02X attempt %u: %s\n", address,
                      attempt, bmeOK ? "OK" : "FAIL");
        if (!bmeOK) delay(400);
      }
    } else if (chipId == 0x58 || chipId == 0x56 || chipId == 0x57) {
      for (uint8_t attempt = 1; attempt <= SENSOR_INIT_ATTEMPTS && !bmpOK; ++attempt) {
        bmpOK = bmp.begin(address, chipId); // verified chip ID
        if (bmpOK) {
          environmentAddress = address;
          bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                          Adafruit_BMP280::SAMPLING_X2,
                          Adafruit_BMP280::SAMPLING_X16,
                          Adafruit_BMP280::FILTER_OFF);
        }
        Serial.printf("[INIT] BMP280 0x%02X attempt %u: %s\n", address,
                      attempt, bmpOK ? "OK" : "FAIL");
        if (!bmpOK) delay(400);
      }
    }
  }
}

void initSensors() {
  Wire.begin(I2C_SDA_GPIO, I2C_SCL_GPIO); // GPIO numbers also compile as Generic ESP8266 Module
  Wire.setClock(100000);      // 100 kHz: conservative for shared wiring
  delay(1200);                // give sensor supplies extra time to settle after boot / wake

  for (uint8_t i = 0; i < 4; ++i) {
    uint8_t addr = (i == 0) ? 0x38 : (i == 1) ? 0x76 : (i == 2) ? 0x77 : 0x40;
    Wire.beginTransmission(addr);
    bool present = (Wire.endTransmission() == 0);
    Serial.printf("[I2C SCAN] 0x%02X %s\n", addr, present ? "ACK" : "NO-ACK");
  }

  Serial.printf("[CHIP] 0x77 ID=0x%02X (BMP=0x56/57/58, BME=0x60)\n", readChipId(0x77));
  initEnvironmentSensors();
  inaOK = false;
  for (uint8_t attempt = 1; attempt <= SENSOR_INIT_ATTEMPTS && !inaOK; ++attempt) {
    inaOK = ina.begin();
    Serial.printf("[INIT] INA219 attempt %u: %s\n", attempt,
                  inaOK ? "OK" : "FAIL");
    if (!inaOK) delay(400);
  }
  if (inaOK) {
    ina.setCalibration_32V_2A();
    ina.powerSave(true); // wake only when taking a reading
  }

  if (!ahtOK) Serial.println("[DIAG] AHT20 ACK alone does NOT guarantee begin()/calibration success");
  if (!bmpOK && !bmeOK) Serial.println("[DIAG] Check chip ID and begin() failure, even if 0x77 ACKed");
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
      delay(350);
    }
  }

  if (bmeOK) {
    for (uint8_t attempt = 1; attempt <= SENSOR_READ_ATTEMPTS; ++attempt) {
      if (!bme.takeForcedMeasurement()) {
        Serial.println("[READ] BME280 forced measurement failed");
        delay(100);
        continue;
      }
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
      delay(350);
    }
  } else if (bmpOK) {
    for (uint8_t attempt = 1; attempt <= SENSOR_READ_ATTEMPTS; ++attempt) {
      if (!bmp.takeForcedMeasurement()) {
        Serial.println("[READ] BMP280 forced measurement failed");
        delay(100);
        continue;
      }
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
      delay(350);
    }
  }

  if (inaOK) {
    ina.powerSave(false);
    // 32V_2A calibration uses 12-bit bus + shunt ADC conversions (~1.1ms total).
    // Allow settling after waking before reading fresh registers.
    delay(10);
    float busV = ina.getBusVoltage_V();
    float shuntMV = ina.getShuntVoltage_mV();
    float currentMA = ina.getCurrent_mA();
    float powerMW = ina.getPower_mW();
    ina.powerSave(true); // also saves power in USE_DEEP_SLEEP=0 between reports
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

bool isWiFi2Enabled() {
  // Default placeholder disables the second network until you edit it.
  return WIFI2_SSID[0] != '\0' && strncmp(WIFI2_SSID, "YOUR_", 5) != 0;
}

bool tryWiFi(const char* ssid, const char* password, uint32_t timeoutMs, uint8_t index) {
  Serial.printf("[WiFi] Trying WiFi%u: %s\n", index, ssid);
  WiFi.disconnect();
  delay(150);
  WiFi.begin(ssid, password);
  const uint32_t startAt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAt < timeoutMs) {
    delay(250);
    yield();
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("[WiFi] WiFi%u FAILED\n", index);
    return false;
  }
  connectedWiFiIndex = index;
  Serial.printf("[WiFi] Connected to WiFi%u: %s | IP: ", index, ssid);
  Serial.println(WiFi.localIP());
  return true;
}

bool ensureWiFi(uint32_t timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) return true;
  if (mqtt.connected()) mqtt.disconnect();
  connectedWiFiIndex = 0;
  WiFi.mode(WIFI_STA);
  const bool hasWiFi2 = isWiFi2Enabled();
  const uint32_t eachTimeout = hasWiFi2 ? timeoutMs / 2UL : timeoutMs;
  if (tryWiFi(WIFI1_SSID, WIFI1_PASS, eachTimeout, 1)) return true;
  if (hasWiFi2 && tryWiFi(WIFI2_SSID, WIFI2_PASS, eachTimeout, 2)) return true;
  Serial.println("[WiFi] All WiFi networks failed. Check SSIDs, passwords, 2.4 GHz and DHCP.");
  return false;
}

const char* selectedMQTTHost() {
  // We deliberately manage fallback ourselves (auto-reconnect is disabled).
  return connectedWiFiIndex == 2 ? MQTT_HOST_WIFI2 : MQTT_HOST_WIFI1;
}

bool ensureMQTT() {
  if (mqtt.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  // Each room has a unique client ID based on the ESP chip ID.
  if (strlen(MQTT_PASS) == 0 || String(MQTT_PASS).startsWith("YOUR_") || String(MQTT_USER).startsWith("YOUR_")) {
    Serial.println("[MQTT] Fill in MQTT password in USER CONFIG");
    return false;
  }
  mqtt.setServer(selectedMQTTHost(), MQTT_PORT);
  Serial.printf("[MQTT] Connecting via %s to %s:%u\n", WiFi.SSID().c_str(),
                selectedMQTTHost(), MQTT_PORT);
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
  // User-confirmed wiring: VIN+ is directly connected to battery positive.
  json += ",\"battery_voltage\":" + (BATTERY_AT_VIN_PLUS ? numOrNull(r.sourceVoltage) : String("null"));
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

// Read back mode registers: logs confirm register state, NOT physical rail current.
bool readRegisterBytes(uint8_t address, uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

void parkSensors() {
  if ((bmpOK || bmeOK) && environmentAddress) {
    uint8_t ctrl = 0;
    bool ok = readRegisterBytes(environmentAddress, 0xF4, &ctrl, 1);
    if (ok) {
      // Clear mode[1:0], preserve oversampling. Keep driver's cached FORCED mode
      // so takeForcedMeasurement() can wake it again during the next report.
      Wire.beginTransmission(environmentAddress);
      Wire.write(0xF4);
      Wire.write((uint8_t)(ctrl & 0xFC));
      ok = Wire.endTransmission() == 0;
      delay(50); // allow any conversion already in progress to finish
      uint8_t verify = 0xFF;
      ok = ok && readRegisterBytes(environmentAddress, 0xF4, &verify, 1)
              && ((verify & 0x03) == 0);
    }
    Serial.printf("[POWER] BMP/BME sleep register: %s\n", ok ? "OK" : "FAILED");
  } else {
    Serial.println("[POWER] BMP/BME not initialized; sleep state unverified");
  }
  if (inaOK) {
    ina.powerSave(true);
    uint8_t config[2] = {0xFF, 0xFF};
    bool ok = readRegisterBytes(0x40, 0x00, config, 2) && ((config[1] & 0x07) == 0);
    Serial.printf("[POWER] INA219 power-down register: %s\n", ok ? "OK" : "FAILED");
  } else {
    Serial.println("[POWER] INA219 not initialized; power-down state unverified");
  }
  // AHT20 is command-triggered. Do NOT invent a sleep command / send soft reset.
  // After getEvent() completes, no more measurements are requested until wake.
  Serial.println("[POWER] AHT20 idle between measurement commands; VCC remains on");
}

void publishReading() {
  SensorReading r = readSensors();
  // One bounded recovery attempt per wake. Never fabricate missing readings.
  if (!r.ahtReadOK || (!r.bmpReadOK && !r.bmeReadOK)) {
    Serial.println("[RECOVER] Environmental read failed; re-init + re-read once");
    initEnvironmentSensors();
    delay(300);
    SensorReading retryReading = readSensors();
    // Preserve all valid first-pass values. Retry should only improve missing data.
    if (retryReading.ahtReadOK) {
      r.temperature = retryReading.temperature;
      r.humidity = retryReading.humidity;
      r.ahtReadOK = true;
    }
    if (retryReading.bmpReadOK || retryReading.bmeReadOK) {
      r.pressure = retryReading.pressure;
      if (!r.ahtReadOK && isfinite(retryReading.temperature)) r.temperature = retryReading.temperature;
      if (retryReading.bmeReadOK && !r.ahtReadOK && isfinite(retryReading.humidity)) r.humidity = retryReading.humidity;
      r.bmpReadOK = retryReading.bmpReadOK;
      r.bmeReadOK = retryReading.bmeReadOK;
    }
    if (retryReading.inaReadOK && !r.inaReadOK) {
      r.voltage = retryReading.voltage;
      r.sourceVoltage = retryReading.sourceVoltage;
      r.shuntMV = retryReading.shuntMV;
      r.currentMA = retryReading.currentMA;
      r.powerMW = retryReading.powerMW;
      r.inaReadOK = true;
    }
    r.recoveryUsed = true;
  }
  Serial.printf("[HEALTH] AHT=%s BMP=%s BME=%s INA=%s recovery=%s\n",
                r.ahtReadOK ? "OK" : "FAIL", r.bmpReadOK ? "OK" : "FAIL",
                r.bmeReadOK ? "OK" : "FAIL", r.inaReadOK ? "OK" : "FAIL",
                r.recoveryUsed ? "YES" : "NO");
  parkSensors(); // stop conversions before spending time connecting to network
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
  if (BATTERY_AT_VIN_PLUS) {
    Serial.println("[BATTERY] VIN+ = " + numOrNull(r.sourceVoltage, 3) + " V (awake sample)");
  } else {
    Serial.println("[BATTERY] battery_voltage disabled: VIN+ wiring has not been verified");
  }
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
  parkSensors(); // also runs when WiFi/MQTT failed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  // Release I2C lines, no internal pullups. External bus pullups remain connected.
  // This is NOT power gating and does not remove VCC from any module.
  pinMode(I2C_SDA_GPIO, INPUT);
  pinMode(I2C_SCL_GPIO, INPUT);
  // Aim for the FULL cycle to take REPORT_INTERVAL_MINUTES including boot/WiFi.
  const uint32_t elapsedMs = millis();
  const uint32_t sleepMs = (elapsedMs < REPORT_INTERVAL_MS - 1000UL)
                               ? (REPORT_INTERVAL_MS - elapsedMs)
                               : 1000UL;
  Serial.printf("[SLEEP] %lu ms; wire D0(GPIO16) -> RST for auto wake\n",
                (unsigned long)sleepMs);
  Serial.flush();
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
  Serial.println("[MQTT] WiFi1 target: " + String(MQTT_HOST_WIFI1) + ":" + String(MQTT_PORT));
  Serial.println("[MQTT] WiFi2 target: " + String(MQTT_HOST_WIFI2) + ":" + String(MQTT_PORT));
  Serial.println("[MQTT] Topic: " + topicState);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false); // manual, deterministic fallback between WiFi1/WiFi2
  mqtt.setServer(MQTT_HOST_WIFI1, MQTT_PORT);
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
  enterDeepSleep(); // retry sleep if the initial call unexpectedly returned
#else
  const uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED && now - lastWiFiTryAt >= WIFI_RETRY_INTERVAL_MS) {
    lastWiFiTryAt = now;
    ensureWiFi(WIFI_CONNECT_TIMEOUT_MS);
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
