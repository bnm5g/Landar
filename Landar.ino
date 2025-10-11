#define BLYNK_TEMPLATE_ID "ID here"
#define BLYNK_TEMPLATE_NAME "Landar"
#define BLYNK_AUTH_TOKEN "Token here"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "DHT.h"
#include <ArduinoJson.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Your Wifi ID here";
char pass[] = "Your Wifi password here";

#define DHTPIN 15
#define DHTTYPE DHT11
#define AIRQ_PIN 34
#define WIND_PIN 4

DHT dht(DHTPIN, DHTTYPE);

volatile int Count = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 1000;

uint16_t pm2_5, pm10;
float windSpeed;
int airQ;

BlynkTimer timer;

void IRAM_ATTR onChange() {
  if (digitalRead(WIND_PIN) == LOW) {
    Count++;
  }
}

bool readZH03B(uint16_t &pm2_5, uint16_t &pm10) {
  if (Serial2.available() >= 32) {
    uint8_t frame[32];
    Serial2.readBytes(frame, 32);
    if (frame[0] != 0x42 || frame[1] != 0x4D) return false;

    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) sum += frame[i];
    uint16_t checksum = (frame[30] << 8) | frame[31];
    if (sum != checksum) return false;

    pm2_5 = (frame[12] << 8) | frame[13];
    pm10  = (frame[14] << 8) | frame[15];
    return true;
  }
  return false;
}

void readSensors() {
  // Air quality
  airQ = analogRead(AIRQ_PIN);

  // DHT
  float humid = dht.readHumidity();
  float temp  = dht.readTemperature();

  // Wind speed
  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    float windSpeed = (Count * 8.75) / 100.0;
    Count = 0;
  }

  // ZH03B
  if (readZH03B(pm2_5, pm10)) {
    Serial.printf("PM2.5: %d µg/m³ | PM10: %d µg/m³\n", pm2_5, pm10);
  }

  // Print debug
  Serial.println("===== Sensor Readings =====");
  Serial.printf("AirQ: %d\n ppm", airQ);
  Serial.printf("Humidity: %.1f %% | Temp: %.1f °C\n", humid, temp);
  Serial.printf("Wind Speed: %.2f m/s\n", windSpeed);
  Serial.println("============================\n");

  // Send to Blynk
  Blynk.virtualWrite(V0, airQ);
  Blynk.virtualWrite(V3, humid);
  Blynk.virtualWrite(V4, temp);
  Blynk.virtualWrite(V5, windSpeed);
  Blynk.virtualWrite(V1, pm2_5);
  Blynk.virtualWrite(V2, pm10);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  pinMode(WIND_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), onChange, FALLING);
  dht.begin();

  Serial.println("Connecting WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Run every 2 seconds without blocking
  timer.setInterval(2000L, readSensors);
}

void loop() {
  Blynk.run();
  timer.run();
}
