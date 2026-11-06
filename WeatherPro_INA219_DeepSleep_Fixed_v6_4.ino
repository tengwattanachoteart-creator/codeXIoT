// Patch notes for v6.4:
// Every deep-sleep wake reboots setup() and measures INA219 anew.
// INA219 now checks individual I2C read results and averages >=2 of 3 valid samples.
// It automatically re-initializes the INA219 on an INA-only failure.
// Missing INA measurements are sent as JSON null, NOT made-up 0.00 readings.
// New MQTT diagnostics: ina_i2c_ok, ina_recovery_used, ina_read_attempts, ina_valid_samples.
// Existing MQTT topics and measurement field names remain compatible.

/*
Weather Pro v6.4 — INA219 reliable wake / repeated reads
- เพิ่ม battery_voltage จาก INA219 ฝั่ง VIN+ ที่ต่อกับขั้วบวกแบตตามวงจรที่ผู้ใช้ยืนยัน
- ไม่ใช้ A0 ไม่เพิ่มสาย; อ่านแบตขณะตื่นก่อนสั่ง INA219 powerSave(true)
- voltage เดิมเป็นฝั่ง VIN-; source_voltage เดิมยังอยู่; เพิ่ม battery_voltage แยกต่างหาก
- ต้องเพิ่ม battery_voltage ใน Node-RED normalize และ Influx fields หากต้องการเก็บฟิลด์นี้
- ห้อง 301 / รอบ 5 นาที / Deep Sleep เปิดไว้ เปลี่ยน ROOM_NUMBER และ ROOM_LABEL ตามบอร์ด
- MQTT_HOST เปลี่ยนเป็น 10.39.66.100 ตามเซิร์ฟเวอร์ใหม่
- WiFi/MQTT user/password คงค่าจากไฟล์ที่ส่งมา ตรวจว่าตรงกับเครื่องใหม่ก่อนอัปโหลด
- ต้องต่อ D0(GPIO16) -> RST เพื่อให้ตื่นเอง
- BMP/BME ใช้ FORCED mode วัดครั้งเดียวแล้วพัก; INA219 powerSave(true) หลังอ่านค่า
- AHT20 วัดตามคำสั่งแล้ว idle ไม่ส่งคำสั่งวัดระหว่างหลับ
- parkSensors() ทำงานก่อนเชื่อมเครือข่าย และก่อนหลับแม้ส่งข้อมูลไม่สำเร็จ
- Serial [POWER] ... OK หมายถึงอ่าน mode register กลับได้ ไม่ใช่การวัดกระแสจริง
- VCC ยังจ่ายไฟอยู่: LED, regulator และ battery shield ยังอาจกินไฟ
- ต้องการตัดไฟจริงต้องใช้ load switch / MOSFET ที่เหมาะกับวงจร พร้อมจัดการไฟย้อน SDA/SCL
  อย่าตัดสายที่เลี้ยง ESP ผ่าน INA219 VIN+/VIN- และอย่าต่อไฟเลี้ยงเซ็นเซอร์จาก GPIO โดยตรง
- ค่ากระแส MQTT คือค่าขณะตื่น ไม่ใช่กระแสขณะหลับหรือค่าเฉลี่ยทั้งรอบ
- ทดสอบ 3 รอบ (ประมาณ 15 นาที) ดู MQTT และ log ก่อน/หลังตื่น
- วัดการกินไฟขณะหลับด้วยเครื่องวัดภายนอก; ไม่รับประกันระยะเวลาแบตโดยไม่วัดจริง
- ยังไม่ได้คอมไพล์ด้วย Arduino toolchain หรือทดสอบฮาร์ดแวร์ในสภาพแวดล้อมนี้
แหล่งอ้างอิง API:
https://github.com/adafruit/Adafruit_BMP280_Library/blob/master/examples/bmp280_forced/bmp280_forced.ino
https://github.com/adafruit/Adafruit_BME280_Library/blob/master/Adafruit_BME280.cpp
https://github.com/adafruit/Adafruit_INA219/blob/master/Adafruit_INA219.cpp
https://cdn-learn.adafruit.com/assets/assets/000/091/676/original/AHT20-datasheet-2020-4-16.pdf
*/

/*
 Weather Pro v6.4 - SENSOR LOW-POWER + ROBUST INA219 READING
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

// ================= USER CONFIG: EDIT THESE ONLY ===================
#define ROOM_NUMBER                  301
#define ROOM_LABEL                   "CB40508"
const char* WIFI_SSID = "Chamber of Secrets";
const char* WIFI_PASS = "kmutt10140";
const char* MQTT_HOST = "10.39.66.100";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "staff";
const char* MQTT_PASS = "kmutt10140";
// ================================================================

#define REPORT_INTERVAL_MINUTES 5
#define USE_DEEP_SLEEP 1

const float TEMP_ALERT_C = 35.0f;
const float HUMIDITY_ALERT_PERCENT = 80.0f;
// A zero bus voltage is not treated as a valid reading for a powered ESP8266.
// If your hardware genuinely measures below 1 V, adapt this threshold to the circuit.
const float INA_MIN_VALID_BUS_V = 1.0f;
const uint8_t INA_SAMPLES_PER_PASS = 3;
const uint8_t INA_MIN_VALID_SAMPLES = 2;
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
  bool inaI2cOK = false;
  bool inaRecoveryUsed = false;
  uint8_t inaAttempts = 0;
  uint8_t inaValidSamples = 0;
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

// Does a particular I2C address respond in this wake cycle?
bool i2cResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// Begin/re-begin INA219; checks that both calibration and power-down writes succeed.
// begin() on the Adafruit library reuses its I2C device on subsequent calls.
bool initializeInaSensor() {
  inaOK = false;
  for (uint8_t attempt = 1; attempt <= SENSOR_INIT_ATTEMPTS; ++attempt) {
    if (!i2cResponds(0x40)) {
      Serial.printf("[INIT] INA219 %u/%u: NO I2C ACK (0x40)\n", attempt, SENSOR_INIT_ATTEMPTS);
      delay(250);
      continue;
    }
    bool began = ina.begin(&Wire);
    bool calibrated = false;
    bool powerDown = false;
    if (began) {
      ina.setCalibration_32V_2A();
      calibrated = ina.success();
      ina.powerSave(true);
      powerDown = ina.success();
    }
    inaOK = began && calibrated && powerDown;
    Serial.printf("[INIT] INA219 %u/%u begin=%d calibration=%d park=%d => %s\n",
                  attempt, SENSOR_INIT_ATTEMPTS, began, calibrated, powerDown,
                  inaOK ? "OK" : "FAIL");
    if (inaOK) return true;
    delay(250);
  }
  return false;
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
  initializeInaSensor(); // sensor is parked after initialization

  if (!ahtOK) Serial.println("[DIAG] AHT20 ACK alone does NOT guarantee begin()/calibration success");
  if (!bmpOK && !bmeOK) Serial.println("[DIAG] Check chip ID and begin() failure, even if 0x77 ACKed");
  Serial.printf("[I2C INIT] AHT20=%s BMP280=%s BME280=%s INA219=%s\n",
                ahtOK ? "OK" : "MISSING", bmpOK ? "OK" : "MISSING",
                bmeOK ? "OK" : "MISSING", inaOK ? "OK" : "MISSING");
}

// Obtain 3 fresh independent readings. Two valid readings are required.
// On transport failure or zero/implausible BUS voltage, never invent a 0V result.
void sampleInaPass(SensorReading& r) {
  double busSum = 0.0, shuntSum = 0.0, currentSum = 0.0, powerSum = 0.0;
  uint8_t good = 0;
  for (uint8_t attempt = 1; attempt <= INA_SAMPLES_PER_PASS; ++attempt) {
    ++r.inaAttempts;
    const bool ack = i2cResponds(0x40);
    r.inaI2cOK = r.inaI2cOK || ack;
    if (!ack) {
      Serial.printf("[INA] sample %u/%u: NO-ACK at 0x40\n", attempt, INA_SAMPLES_PER_PASS);
      delay(120);
      continue;
    }
    ina.powerSave(false); // resume shunt and bus voltage continuous conversions
    const bool wakeOK = ina.success();
    delay(45); // deliberately longer than a 12-bit shunt + bus ADC cycle
    if (!wakeOK) {
      Serial.printf("[INA] sample %u/%u: powerSave(false) FAILED\n", attempt, INA_SAMPLES_PER_PASS);
      delay(120);
      continue;
    }

    const float busV = ina.getBusVoltage_V();
    const bool busReadOK = ina.success();
    const float shuntMV = ina.getShuntVoltage_mV();
    const bool shuntReadOK = ina.success();
    const float currentMA = ina.getCurrent_mA();
    const bool currentReadOK = ina.success();
    const float powerMW = ina.getPower_mW();
    const bool powerReadOK = ina.success();
    ina.powerSave(true); // save power between samples, and before WiFi/MQTT
    const bool parkOK = ina.success();

    const bool busOK = busReadOK && isfinite(busV) &&
                       busV >= INA_MIN_VALID_BUS_V && busV <= 26.0f;
    const bool shuntOK = shuntReadOK && isfinite(shuntMV) && fabsf(shuntMV) <= 320.0f;
    const bool currentOK = currentReadOK && isfinite(currentMA) && fabsf(currentMA) <= 3200.0f;
    const bool powerOK = powerReadOK && isfinite(powerMW) && fabsf(powerMW) <= 100000.0f;
    const bool valid = busOK && shuntOK && currentOK && powerOK && parkOK;
    Serial.printf("[INA] sample %u/%u: bus=%.3fV shunt=%.3fmV current=%.2fmA "
                  "power=%.2fmW i2c=%d%d%d%d park=%d valid=%d\n",
                  attempt, INA_SAMPLES_PER_PASS, busV, shuntMV, currentMA, powerMW,
                  busReadOK, shuntReadOK, currentReadOK, powerReadOK, parkOK, valid);
    if (valid) {
      ++good;
      busSum += busV;
      shuntSum += shuntMV;
      currentSum += currentMA;
      powerSum += powerMW;
    }
    delay(120);
  }
  r.inaValidSamples = good;
  if (good >= INA_MIN_VALID_SAMPLES) {
    r.voltage = (float)(busSum / good);
    r.shuntMV = (float)(shuntSum / good);
    r.currentMA = (float)(currentSum / good);
    r.powerMW = (float)(powerSum / good);
    r.sourceVoltage = r.voltage + r.shuntMV / 1000.0f;
    r.inaReadOK = true;
  }
  Serial.printf("[INA] pass result: %u/%u valid; %s\n", good,
                INA_SAMPLES_PER_PASS, r.inaReadOK ? "OK" : "RETRY/RECOVER");
}

void measureInaWithRecovery(SensorReading& r) {
  if (!inaOK) {
    r.inaRecoveryUsed = true;
    Serial.println("[INA] Not ready at boot: retrying initialization");
    initializeInaSensor();
  }
  if (inaOK) sampleInaPass(r);
  if (r.inaReadOK) return;

  // ONE reinitialization per report, not an infinite retry loop.
  r.inaRecoveryUsed = true;
  Serial.println("[INA] RECOVERY: reset calibration/config and retry 3 samples");
  if (initializeInaSensor()) sampleInaPass(r);
  if (!r.inaReadOK) {
    Serial.println("[INA] FAILED: sending null voltage/current/power; check INA219 wiring/power");
  }
}

SensorReading readSensors(bool includeINA = true) {
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

  if (includeINA) {
    measureInaWithRecovery(r);
    r.recoveryUsed = r.inaRecoveryUsed;
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
  if (strlen(MQTT_PASS) == 0 || String(MQTT_PASS).startsWith("YOUR_") || String(MQTT_USER).startsWith("YOUR_")) {
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
  // User-confirmed wiring: VIN+ is directly connected to battery positive.
  json += ",\"battery_voltage\":" + numOrNull(r.sourceVoltage);
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
  json += ",\"ina_i2c_ok\":" + String(r.inaI2cOK ? "true" : "false");
  json += ",\"ina_recovery_used\":" + String(r.inaRecoveryUsed ? "true" : "false");
  json += ",\"ina_read_attempts\":" + String(r.inaAttempts);
  json += ",\"ina_valid_samples\":" + String(r.inaValidSamples);
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
    SensorReading retryReading = readSensors(false); // environment only; do not re-read good INA data
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
  Serial.println("[BATTERY] VIN+ = " + numOrNull(r.sourceVoltage, 3) + " V (awake sample)");
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
  enterDeepSleep(); // retry sleep if the initial call unexpectedly returned
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
