#define BLYNK_TEMPLATE_ID "TMPL6pU_4ifNy"
#define BLYNK_TEMPLATE_NAME "Landar"
#define BLYNK_AUTH_TOKEN "3lM8hyfL_UbuU9GG520Sw-ORasbQIxBR"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "DHT.h"
#include <ArduinoJson.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "YourWiFiName";
char pass[] = "YourWiFiPass";


#define DHTPIN 15
#define DHTTYPE DHT11
#define AIRQ_PIN 34
#define WIND_PIN 4

DHT dht(DHTPIN, DHTTYPE);

volatile int Count = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 1000;

uint16_t pm2_5, pm10;
uint8_t buffer[32];
float windSpeed;
int airQ;

void IRAM_ATTR onChange() {
  if (digitalRead(WIND_PIN) == LOW) {
    Count++;
  }
}

bool readZH03B(uint16_t &pm2_5, uint16_t &pm10) {
  if (Serial2.available() >= 32) {
    // Read 32-byte frame
    uint8_t frame[32];
    Serial2.readBytes(frame, 32);

    // Check header
    if (frame[0] != 0x42 || frame[1] != 0x4D) {
      return false; // Invalid start
    }

    // Compute checksum
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) {
      sum += frame[i];
    }
    uint16_t checksum = (frame[30] << 8) | frame[31];
    if (sum != checksum) {
      return false; // Invalid
    }

    // Extract PM2.5 and PM10 (ug/m³)
    pm2_5 = (frame[12] << 8) | frame[13];
    pm10  = (frame[14] << 8) | frame[15];

    return true; // Success
  }
  return false; // Not enough
}

void setup() {
  Serial.begin(115200);      // Debug
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  pinMode(WIND_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), onChange, FALLING);

  dht.begin();

  Blynk.begin(auth, ssid, pass);
}


void loop() {
  Blynk.run();

  // Read air quality
  airQ = analogRead(AIRQ_PIN);

  // DHT
  float humid = dht.readHumidity();
  float temp  = dht.readTemperature();

  // Wind speed
  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    windSpeed = (Count * 8.75) / 100.0;
    Count = 0;
  }

  if (readZH03B(pm2_5, pm10)) {
    Serial.print("PM2.5: ");
    Serial.print(pm2_5);
    Serial.print(" µg/m³ | PM10: ");
    Serial.print(pm10);
    Serial.println(" µg/m³");

  // Send to Blynk
  Blynk.virtualWrite(V0, airQ);
  Blynk.virtualWrite(V1, humid);
  Blynk.virtualWrite(V2, temp);
  Blynk.virtualWrite(V3, windSpeed);
  Blynk.virtualWrite(V4, pm2_5);
  Blynk.virtualWrite(V5, pm10);

  delay(2000);
}




