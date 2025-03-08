// MQ-135 Example Code
const int mq135Pin = 1; // ADC pin, e.g., GPIO1

void setup() {
  Serial.begin(115200);
  pinMode(mq135Pin, INPUT);
}

void loop() {
  int sensorValue = analogRead(mq135Pin);
  float voltage = sensorValue * (3.3 / 4095.0);  // Assuming 12-bit ADC (ESP32 ADC range)
  Serial.print("MQ-135 Voltage: ");
  Serial.println(voltage);
  delay(1000);
}


// Sound Sensor Example Code
const int soundAnalogPin = 2;  // Analog output pin, e.g., GPIO2
const int soundDigitalPin = 25; // Digital output pin (assign as available)

void setup() {
  Serial.begin(115200);
  pinMode(soundAnalogPin, INPUT);
  pinMode(soundDigitalPin, INPUT);
}

void loop() {
  int analogVal = analogRead(soundAnalogPin);
  int digitalVal = digitalRead(soundDigitalPin);
  Serial.print("Sound Analog: ");
  Serial.print(analogVal);
  Serial.print(" | Digital: ");
  Serial.println(digitalVal);
  delay(500);
}


// Dust Sensor Example Code
const int dustSensorPin = 3; // ADC pin, e.g., GPIO3

void setup() {
  Serial.begin(115200);
  pinMode(dustSensorPin, INPUT);
}

void loop() {
  int sensorValue = analogRead(dustSensorPin);
  float voltage = sensorValue * (3.3 / 4095.0);
  Serial.print("Dust Sensor Voltage: ");
  Serial.println(voltage);
  delay(1000);
}


// Soil Moisture Example Code
const int soilMoisturePin = 4; // ADC pin, e.g., GPIO4

void setup() {
  Serial.begin(115200);
  pinMode(soilMoisturePin, INPUT);
}

void loop() {
  int moistureValue = analogRead(soilMoisturePin);
  Serial.print("Soil Moisture: ");
  Serial.println(moistureValue);
  delay(1000);
}


// Hall Effect Sensor Example Code
const int hallSensorPin = 7; // ADC pin, e.g., GPIO7

void setup() {
  Serial.begin(115200);
  pinMode(hallSensorPin, INPUT);
}

void loop() {
  int hallValue = analogRead(hallSensorPin);
  Serial.print("Hall Sensor Value: ");
  Serial.println(hallValue);
  delay(1000);
}


// MAX471 Example Code
const int max471Pin = 10; // ADC pin, e.g., GPIO10

void setup() {
  Serial.begin(115200);
  pinMode(max471Pin, INPUT);
}

void loop() {
  int sensorValue = analogRead(max471Pin);
  float voltage = sensorValue * (3.3 / 4095.0);
  Serial.print("MAX471 Voltage: ");
  Serial.println(voltage);
  delay(1000);
}


// Rain Sensor Example Code
const int rainAnalogPin = 46; // ADC pin, e.g., GPIO46 (ADC2)
const int rainDigitalPin = 27; // Digital pin (choose an available one)

void setup() {
  Serial.begin(115200);
  pinMode(rainAnalogPin, INPUT);
  pinMode(rainDigitalPin, INPUT);
}

void loop() {
  int analogVal = analogRead(rainAnalogPin);
  int digitalVal = digitalRead(rainDigitalPin);
  Serial.print("Rain Sensor Analog: ");
  Serial.print(analogVal);
  Serial.print(" | Digital: ");
  Serial.println(digitalVal);
  delay(1000);
}


#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9); // I2C pins: SDA = GPIO8, SCL = GPIO9
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED allocation failed");
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Smart City Sentinel");
  display.display();
}

void loop() {
  // Update display as needed
  delay(1000);
}


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme; // I2C

void setup() {
  Serial.begin(115200);
  if (!bme.begin(0x76)) {  // 0x76 or 0x77
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  Serial.print("Temp: ");
  Serial.print(bme.readTemperature());
  Serial.print(" °C | Pressure: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.print(" hPa | Humidity: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  delay(2000);
}

#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9); // I2C pins (same as OLED/BME280)
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  // Uncomment to set RTC time once
  // rtc.adjust(DateTime(F(_DATE), F(TIME_)));
}

void loop() {
  DateTime now = rtc.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);
  delay(1000);
}


const int flameSensorPin = 14; // Digital pin

void setup() {
  Serial.begin(115200);
  pinMode(flameSensorPin, INPUT);
}

void loop() {
  int flameVal = digitalRead(flameSensorPin);
  Serial.print("Flame Sensor: ");
  Serial.println(flameVal);
  delay(500);
}


volatile unsigned int pulseCount = 0;
const int flowSensorPin = 16; // Digital pin with interrupt

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  pinMode(flowSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);
}

void loop() {
  unsigned int count;
  noInterrupts();
  count = pulseCount;
  pulseCount = 0;
  interrupts();
  // Calculate flow rate based on sensor’s calibration (pulses per liter)
  float flowRate = count * 1.0; // Adjust conversion factor as per datasheet
  Serial.print("Flow Rate: ");
  Serial.print(flowRate);
  Serial.println(" pulses/sec");
  delay(1000);
}

#include "DHT.h"
#define DHTPIN 17    // Digital pin
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  dht.begin();
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\tTemperature: ");
  Serial.print(t);
  Serial.println(" °C");
  delay(2000);
}


#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN  18  // RC522 reset pin
#define SS_PIN   45  // SPI CS pin for RC522

MFRC522 mfrc522(SS_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  SPI.begin(19, 20, 21, SS_PIN); // SCLK=GPIO19, MISO=GPIO20, MOSI=GPIO21
  mfrc522.PCD_Init();
  Serial.println("RC522 RFID Reader Initialized");
}

void loop() {
  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent() || ! mfrc522.PICC_ReadCardSerial() ) {
    delay(50);
    return;
  }
  Serial.print("Card UID:");
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
  }
  Serial.println();
  mfrc522.PICC_HaltA();
}


#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial SerialGPS(2);  // Using Serial2

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 43, 44); // RX=GPIO43, TX=GPIO44
}

void loop() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  
  if (gps.location.isUpdated()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" | Lon: ");
    Serial.println(gps.location.lng(), 6);
  }
  delay(1000);
}


const int batteryVoltagePin = A0; // Example ADC pin for battery voltage divider
const int relayControlPin = 22;   // Digital output controlling the battery selector relay

void setup() {
  Serial.begin(115200);
  pinMode(batteryVoltagePin, INPUT);
  pinMode(relayControlPin, OUTPUT);
  // Assume relay LOW = disconnect battery (charging), HIGH = battery powering load
  digitalWrite(relayControlPin, HIGH); // Start with Battery A powering the load
}

void loop() {
  int rawValue = analogRead(batteryVoltagePin);
  float voltage = rawValue * (3.3 / 4095.0) * 2;  // Assuming voltage divider halves battery voltage
  Serial.print("Battery Voltage: ");
  Serial.println(voltage);
  
  // Simple logic: if battery voltage is low (<3.6V) switch battery
  if (voltage < 3.6) {
    digitalWrite(relayControlPin, LOW);  // Disconnect battery powering load; switch to charged battery
    Serial.println("Switching Battery...");
    delay(3000); // Delay to allow switchover (capacitor holds voltage)
    digitalWrite(relayControlPin, HIGH); // Reconnect load from charged battery
  }
  delay(2000);
}