#include "DHT.h"

#define DHTPIN A1
#define DHTTYPE DHT11 

uint8_t buffer[32];
int airQ;
DHT dht(DHTPIN, DHTTYPE);
unsigned long lastDebounceTime = 0;  
unsigned long debounceDelay = 1000;    

const int pinInterrupt = 2;  // chân ngắt
volatile int Count = 0;      // volatile để an toàn trong ISR

void onChange() {
  if (digitalRead(pinInterrupt) == LOW) {
    Count++;
  }
}

void setup() {  
  Serial.begin(9600);  
  Serial1.begin(9600);     // Connects to ZH03B
  dht.begin();      

  pinMode(pinInterrupt, INPUT_PULLUP);

  // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(pinInterrupt), onChange, FALLING);

}

void loop() {
  Serial.println();
  delay(2000);  // 2s delay (ZH03B outputs 1Hz anyway)

  // ---- Air Quality analog sensor ----
  airQ = analogRead(A0);       
  Serial.print("Air Quality = ");
  Serial.print(airQ, DEC);            
  Serial.println(" PPM");

  // ---- Humidity and temperature ----
  float humid = dht.readHumidity();
  float temp  = dht.readTemperature(); // Celsius
  Serial.print("Humidity: ");
  Serial.print(humid);
  Serial.print("%\tTemperature: ");
  Serial.print(temp);
  Serial.println(" °C");

  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();

    float windSpeed = (Count * 8.75) / 100.0;  // đổi sang float cho chính xác
    Serial.print("Windspeed: ");
    Serial.print(windSpeed);
    Serial.println(" m/s");


    Count = 0;
  }

  // ---- ZH03B ----
  if (Serial1.available() >= 32) {
    Serial.println("Passed");
    while (Serial1.available() > 0) {
      if (Serial1.peek() == 0x42) {   // Check header start
        Serial1.read();               // Consume 0x42
        if (Serial1.peek() == 0x4D) {
          Serial1.read();             // Consume 0x4D

          // Read rest of frame
          buffer[0] = 0x42;
          buffer[1] = 0x4D;
          Serial1.readBytes(buffer + 2, 30);

          // Verify checksum
          uint16_t sum = 0;
          for (int i = 0; i < 30; i++) {
            sum += buffer[i];
          }
          uint16_t checksum = (buffer[30] << 8) | buffer[31];

          if (sum == checksum) {
            // ✅ Valid frame
            uint16_t pm2_5 = (buffer[12] << 8) | buffer[13];
            uint16_t pm10  = (buffer[14] << 8) | buffer[15];

            Serial.print("PM2.5: ");
            Serial.print(pm2_5);
            Serial.print(" µg/m³ | PM10: ");
            Serial.print(pm10);
            Serial.println(" µg/m³");
          } else {
            Serial.println("⚠️ Bad checksum, discarding frame!");
          }
          break;  // Exit loop after one good (or bad) frame
        } else {
          Serial1.read(); // Not 0x4D, discard
        }
      } else {
        Serial1.read(); // Discard until we hit 0x42
      }
    }
  }
}



