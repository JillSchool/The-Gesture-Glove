#include <Arduino.h>
#include "esp_sleep.h"
#include <WiFi.h>

#define LED 2 // Onboard LED (optioneel)

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  // 1. Active Mode
  Serial.println("Active Mode (geen slaapmodus)");
  // digitalWrite(LED, HIGH);
  delay(5000);

  // 2. Modem-sleep
  Serial.println("Modem-sleep Mode");
  WiFi.setSleep(true);
  WiFi.disconnect(true);
  setCpuFrequencyMhz(80); // Verminder CPU snelheid voor minder verbruik
  // digitalWrite(LED, LOW);
  delay(5000);

  // 3. Light-sleep
  Serial.println("Light-sleep Mode");
  esp_sleep_enable_timer_wakeup(5000000); // 5 seconden
  esp_light_sleep_start();

  // 4. Deep-sleep
  Serial.println("Deep-sleep Mode");
  esp_sleep_enable_timer_wakeup(5000000); // 5 seconden
  esp_deep_sleep_start();
}

void loop() {
  // Loopt nooit want deep-sleep herstart de ESP32
}
