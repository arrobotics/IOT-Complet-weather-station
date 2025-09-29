#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "DFRobot_RainfallSensor.h"

// ================== Anemometer Variables ==================
unsigned long lastDebounceTime = 0;   // the last time we printed
unsigned long debounceDelay = 1000;   // measure every 1 second

int pinInterrupt = 2;   // Sensor output pin (connect through divider if >5V)
volatile int Count = 0; // Pulse counter

// ISR: runs when a pulse is detected
void onChange() {
  if (digitalRead(pinInterrupt) == LOW) {
    Count++;
  }
}

// ================== BME680 Object ==================
Adafruit_BME680 bme; // I2C

// ================== Rainfall Sensor ==================
#define MODE_UART
#ifdef MODE_UART // UART communication
  #include "SoftwareSerial.h"
  SoftwareSerial mySerial(/*rx =*/10, /*tx =*/11);
  DFRobot_RainfallSensor_UART Sensor(/*Stream *=*/&mySerial);
#else // I2C communication
  DFRobot_RainfallSensor_I2C Sensor(&Wire);
#endif

// ================== Setup ==================
void setup() {
  Serial.begin(115200);

  // --- Anemometer Setup ---
  pinMode(pinInterrupt, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(pinInterrupt), onChange, FALLING);

  Serial.println(F("🌬️ Anemometer + BME680 + Rainfall Sensor Started..."));

  // --- BME680 Setup ---
  if (!bme.begin()) {
    Serial.println("❌ Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // --- Rainfall Sensor Setup ---
  #ifdef MODE_UART
    mySerial.begin(9600);
  #endif
  delay(1000);
  while(!Sensor.begin()){
    Serial.println("❌ Rainfall Sensor init error, retrying...");
    delay(1000);
  }
  Serial.print("VID:\t"); Serial.println(Sensor.vid, HEX);
  Serial.print("PID:\t"); Serial.println(Sensor.pid, HEX);
  Serial.print("Version:\t"); Serial.println(Sensor.getFirmwareVersion());
  // Optional: Reset accumulated rainfall value (unit: mm)
  // Sensor.setRainAccumulatedValue(0.2794);
}

// ================== Loop ==================
void loop() {
  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();

    // ===== Wind Speed =====
    float windSpeed = (Count * 8.75) / 100.0; // m/s
    Count = 0;

    // ===== BME680 Reading =====
    if (!bme.performReading()) {
      Serial.println("❌ Failed to perform BME680 reading!");
      return;
    }

    // ===== Rainfall Sensor Reading =====
    unsigned long workingTime = Sensor.getSensorWorkingTime();
    float totalRainfall = Sensor.getRainfall();   // mm
    float oneHourRain = Sensor.getRainfall(1);    // last 1 hour rainfall (mm)
    unsigned long rawData = Sensor.getRawData();  // raw tipping bucket counts

    // ===== Print All Data =====
    Serial.println("========== Weather Station Data ==========");
    // Wind
    Serial.print("🌬️ Wind Speed: "); Serial.print(windSpeed); Serial.println(" m/s");

    // BME680
    Serial.print("🌡️ Temperature: "); Serial.print(bme.temperature); Serial.println(" °C");
    Serial.print("💧 Humidity: ");    Serial.print(bme.humidity);    Serial.println(" %");
    Serial.print("📈 Pressure: ");    Serial.print(bme.pressure / 100.0); Serial.println(" hPa");
    Serial.print("🛢️ Gas: ");         Serial.print(bme.gas_resistance / 1000.0); Serial.println(" KOhms");
    Serial.print("📊 Altitude: ");    Serial.print(bme.readAltitude(1013.25)); Serial.println(" m");

    // Rainfall
    Serial.print("⏱️ Sensor Working Time: "); Serial.print(workingTime); Serial.println(" h");
    Serial.print("🌧️ Total Rainfall: ");     Serial.print(totalRainfall); Serial.println(" mm");
    Serial.print("⏳ Last 1 Hour Rainfall: "); Serial.print(oneHourRain); Serial.println(" mm");
    Serial.print("📟 Raw Data (buckets): "); Serial.println(rawData);

    Serial.println("==========================================\n");
  }
  delay(2000);
}
