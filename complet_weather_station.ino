#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "DFRobot_RainfallSensor.h"

// ----------------- SIM800L -----------------
SoftwareSerial gprsSerial(7, 8); // RX, TX for SIM800L
String apn = "airtelgprs.com";   // Replace with your SIM card APN
String apiKey = "V92RNSJQHGJFTPHC"; // Your ThingSpeak Write API Key

// ----------------- Anemometer -----------------
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 1000;
int pinInterrupt = 2;
volatile int Count = 0;

void onChange() {
  if (digitalRead(pinInterrupt) == LOW) Count++;
}

// ----------------- BME680 -----------------
Adafruit_BME680 bme;

// ----------------- Rainfall Sensor -----------------
#define MODE_UART
#ifdef MODE_UART
  SoftwareSerial mySerial(10, 11);   // RX=10, TX=11
  DFRobot_RainfallSensor_UART Sensor(&mySerial);
#else
  DFRobot_RainfallSensor_I2C Sensor(&Wire);
#endif

// ----------------- Wind Direction -----------------
int windDirPin = A0;

// ----------------- Helper -----------------
void ShowSerialData() {
  while (gprsSerial.available() != 0) {
    Serial.write(gprsSerial.read());
  }
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  gprsSerial.begin(9600);

  // Anemometer
  pinMode(pinInterrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinInterrupt), onChange, FALLING);

  Serial.println(F("🌦 Weather Station Started..."));

  // BME680
  if (!bme.begin()) {
    Serial.println("❌ BME680 not found!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);

  // Rainfall
#ifdef MODE_UART
  mySerial.begin(9600);
#endif
  delay(1000);
  while (!Sensor.begin()) {
    Serial.println("❌ Rainfall Sensor init error, retrying...");
    delay(1000);
  }
}

// ----------------- Send Data to ThingSpeak -----------------
void sendToThingSpeak(float windSpeed, int direction, float temp, float hum, float gas, float rainfall, float pressure, float altitude) {
  // Open GPRS
  gprsSerial.println("AT");
  delay(1000); ShowSerialData();

  gprsSerial.println("AT+CPIN?");
  delay(1000); ShowSerialData();

  gprsSerial.println("AT+CREG?");
  delay(1000); ShowSerialData();

  gprsSerial.println("AT+CGATT=1");
  delay(1000); ShowSerialData();

  gprsSerial.println("AT+CIPSHUT");
  delay(1000); ShowSerialData();

  gprsSerial.println("AT+CIPMUX=0");
  delay(2000); ShowSerialData();

  gprsSerial.println("AT+CSTT=\"" + apn + "\"");
  delay(2000); ShowSerialData();

  gprsSerial.println("AT+CIICR");
  delay(3000); ShowSerialData();

  gprsSerial.println("AT+CIFSR");
  delay(2000); ShowSerialData();

  gprsSerial.println("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"");
  delay(6000); ShowSerialData();

  gprsSerial.println("AT+CIPSEND");
  delay(2000); ShowSerialData();

  // Build URL
  String str = "GET /update?api_key=" + apiKey +
               "&field1=" + String(windSpeed) +
               "&field2=" + String(direction) +
               "&field3=" + String(temp) +
               "&field4=" + String(hum) +
               "&field5=" + String(gas) +
               "&field6=" + String(rainfall) +
               "&field7=" + String(pressure) +
               "&field8=" + String(altitude) +
               " HTTP/1.1\r\nHost: api.thingspeak.com\r\nConnection: close\r\n\r\n";

  Serial.println("🔗 Sending: " + str);
  gprsSerial.println(str);
  delay(2000);
  gprsSerial.write(26); // CTRL+Z to send
  delay(5000);

  gprsSerial.println("AT+CIPCLOSE");
  delay(1000); ShowSerialData();
}

// ----------------- Loop -----------------
void loop() {
  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();

    // Wind Speed
    float windSpeed = (Count * 8.75) / 100.0;
    Count = 0;

    // BME680
    if (!bme.performReading()) {
      Serial.println("❌ Failed to perform BME680 reading!");
      return;
    }

    float temperature = bme.temperature;
    float humidity = bme.humidity;
    float gas = bme.gas_resistance / 1000.0;
    float pressure = bme.pressure / 100.0;  // hPa
    float altitude = bme.readAltitude(1013.25); // Approx altitude

    // Rainfall
    float totalRainfall = Sensor.getRainfall();

    // Wind Direction
    int sensorValue = analogRead(windDirPin);
    int direction = map(sensorValue, 0, 1023, 0, 360);

    // Print
    Serial.println("========== Weather Station Data ==========");
    Serial.print("🌬️ Wind Speed: "); Serial.println(windSpeed);
    Serial.print("🧭 Wind Direction: "); Serial.println(direction);
    Serial.print("🌡️ Temperature: "); Serial.println(temperature);
    Serial.print("💧 Humidity: "); Serial.println(humidity);
    Serial.print("🛢️ Gas: "); Serial.println(gas);
    Serial.print("📈 Pressure: "); Serial.println(pressure);
    Serial.print("📊 Altitude: "); Serial.println(altitude);
    Serial.print("🌧️ Rainfall: "); Serial.println(totalRainfall);
    Serial.println("==========================================");

    // Send to ThingSpeak
    sendToThingSpeak(windSpeed, direction, temperature, humidity, gas, totalRainfall, pressure, altitude);
  }

  delay(20000); // ThingSpeak minimum update interval
}
