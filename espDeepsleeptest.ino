#include <ESP8266WiFi.h>

void setup() {
  Serial.begin(74880);
  delay(1000);

  Serial.println("\nBOOT OK");
  Serial.println(ESP.getResetReason());

  WiFi.mode(WIFI_OFF);
  delay(2000);

  Serial.println("SLEEP 10 seconds");
  Serial.flush();

  ESP.deepSleep(10ULL * 1000000ULL, WAKE_RF_DEFAULT);
}

void loop() {}
