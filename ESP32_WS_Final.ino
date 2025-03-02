#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <ThingSpeak.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "RTClib.h"
#include <Adafruit_AHTX0.h>

#define blueLED 2
#define windSensorPin 13
#define SD_CS_PIN 5 //SD Clock Select

#define BUTTON_PIN_BITMASK 0x100 // 4^4 in hex or 456 
RTC_DATA_ATTR int bootCount = 0;

#define SEALEVELPRESSURE_HPA (1013.25) 
#define MicroS_to_S 1000000ULL //1,000,000 micro seconds is one secondplese
#define Sleep_Time 900 //TIme in seconds the ESP will sleep. 900 = 15 minutes

#define DEBOUNCE_DELAY 50  //milliseconds
#define MEASUREMENT_INTERVAL 1000  // 1 second

#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

Adafruit_AHTX0 aht; 
Adafruit_BME280 bme;
RTC_DS3231 rtc;

//-------------WiFi Connection-------------

WiFiClient client;

const char* ssid = "**************";    // Change this your SSID
const char* password = "**************";  // Change this your Password

unsigned long myChannelNumber = **************;         //Thingspeak channel number
const char* myWriteAPIKey = "**************";  //Thingspeak API write key

//-------------Variables to store readings-------------

int temperature = 0;
int humidity = 0;
int pressure = 0;
int altitude = 0;
float windSpeed = 0;
unsigned int temperatureFieldNumber = 1;

//-------------Timers-------------

const float anemometerRadius = 0.0175;  // 11cm circumference
const float circumference_meters = 2 * PI * anemometerRadius;
const float calibrationFactor = 0.06;  // Adjust this if reading are too high or low, check with hand held anemometer

volatile int rotationCount = 0;
volatile unsigned long lastInterruptTime = 0;
unsigned long lastMeasurementTime = 0;

// Interrupt with debounce
void IRAM_ATTR hallSensorInterrupt() {
unsigned long interruptTime = millis();
if (interruptTime - lastInterruptTime > DEBOUNCE_DELAY) {
rotationCount++;
lastInterruptTime = interruptTime;
  }
}

//----------SD card----------

SdFat sd;
File logFile;

// Timestamp of the last SD card health check
unsigned long lastSDCheckTime = 0;
const unsigned long SD_CHECK_INTERVAL = 86400000; // 24 hours in milliseconds

//----------Wifi----------

// Set the maximum number of connection attempts
const int maxConnectionAttempts = 4;

void connectToWiFi() {
  int attempt = 0;
  Serial.println("[WiFi] Connecting to WiFi...");
  while (attempt < maxConnectionAttempts) {
  Serial.print("[WiFi] Attempt ");
  Serial.println(attempt + 1);
      
  WiFi.begin(ssid, password);

  // Wait for connection or timeout
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 10) {
  digitalWrite(blueLED, HIGH);
  delay(75);
  digitalWrite(blueLED, LOW);
  delay(75);
  timeout++;
}

  if (WiFi.status() == WL_CONNECTED) {
  Serial.println("[WiFi] Connected!");
  Serial.print("[WiFi] IP address: ");
  Serial.println(WiFi.localIP());
  return;
  } else {
  Serial.println("[WiFi] Connection failed.");
  }
  attempt++;
  }
  Serial.println("[WiFi] Failed to connect to WiFi after maximum attempts.");
}

//----------NTP----------

const char* ntpServers[] = {"pool.ntp.org", "time.nist.gov", "ntp.ubuntu.com"};
const long gmtOffset_sec = -6 * 3600; // Adjust this according to your timezone
const int daylightOffset_sec = 0; // Adjust this if your timezone observes daylight saving time

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServers[2], gmtOffset_sec, daylightOffset_sec); // Selecting the first NTP server

//-------------Battery Voltage-------------

// Battery voltage pin (ADC)
int batteryPin = 32;  // Choose the appropriate analog pin
float R1 = 100000.0;   // Resistor 1 (100kΩ)
float R2 = 47000.0;    // Resistor 2 (47kΩ)
float maxADC = 4095.0; // Max ADC value for ESP32 (12-bit)

// Function to read battery voltage
float readBatteryVoltage() {
  int analogValue = analogRead(batteryPin);
  float voltage = (analogValue / maxADC) * 3.3; //Scale the reading to 0-3.3V
  // Calculate the actual battery voltage using the voltage divider formula
  float batteryVoltage = voltage * (R1 + R2) / R2;
  return batteryVoltage;  // Return the calculated battery voltage
}

void setup() {
// put your setup code here, to run once:
Serial.begin(115200);
delay(1000);

++bootCount;
Serial.println("Boot number: " + String(bootCount));

Wire.begin();
  
ThingSpeak.begin(client);

pinMode(blueLED, OUTPUT);
pinMode(windSensorPin, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(windSensorPin), hallSensorInterrupt, FALLING);

esp_sleep_enable_timer_wakeup(Sleep_Time * MicroS_to_S); //Initialize deep sleep mode with a timer.
esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 1); //pin 4 set to wake up from deep sleep

rtc.begin();
if (!rtc.begin()) {
Serial.println("Couldn't find RTC!");
}
    
aht.begin();
if (!aht.begin()) {
Serial.println("Couldn't find AHT10 sensor!");
}
  
bme.begin(0x76);
if (!bme.begin(0x76)) {
Serial.println("Couldn't find BME280 sensor!");
}

display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(10))) {
Serial.println("SD Card initialization failed!");
}
Serial.println("SD Card initialized.");

logFile = sd.open("sensorLog.txt", FILE_WRITE);
if (logFile) {
Serial.println("File written successfully.");
} else {
Serial.println("Error opening file!");
}

connectToWiFi();

// Synchronize time with NTP server
timeClient.begin();
timeClient.forceUpdate();
// Update RTC with NTP time
rtc.adjust(DateTime(timeClient.getEpochTime()));
Serial.println("RTC updated with NTP time");

DateTime now = rtc.now();

sensors_event_t temp_event, humidity_event;
aht.getEvent(&humidity_event, &temp_event);
int temp = temp_event.temperature;
int wet = humidity_event.relative_humidity;
int press = bme.readPressure() / 100;
int alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
recWindSpeed ();
float batteryVoltage = readBatteryVoltage();

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);

  display.setCursor(35, 0);
  display.print("ESP32");

  display.setCursor(25, 25);
  display.print("Weather");

  display.setCursor(25, 45);
  display.print("Station");
  display.display();

  display.setTextSize(1);
  delay(2500);
  display.clearDisplay();
  display.display();

  display.setCursor(20, 0);
  display.println("Connecting to:");

  display.setCursor(20, 35);
  display.println(ssid);
  display.display();
  delay(2000);


if (WiFi.status() == WL_CONNECTED){
  display.clearDisplay();
  display.display();
  digitalWrite(blueLED, HIGH);
  display.setCursor(20, 0);
  display.print("WiFi Connected!");

  display.setCursor(30, 20);
  display.print("IP Address:");

  display.setCursor(20, 40);
  display.print(WiFi.localIP());
  display.display();
  delay(2500);
  digitalWrite(blueLED, LOW);
} else {
  display.clearDisplay();
  display.display();
  digitalWrite(blueLED, HIGH);
  display.setCursor(15, 0);
  display.print("Wireless Offline!");

  display.setCursor(10, 20);
  display.print("Please Check Code!");

  display.setCursor(10, 40);
  display.print("Please Check WiFi!");
  display.display();
  delay(2500);
  digitalWrite(blueLED, LOW);
}

  display.clearDisplay();
  display.display();
  display.setCursor(0, 0);
  display.print("ESP32 Weather Station");

  display.setCursor(35, 18);
  display.print(batteryVoltage);
  display.print("V");

  display.setCursor(60, 8);
  display.print(now.day());
  display.print('/');
  display.print(now.month());
  display.print('/');
  display.print(now.year());

  display.setCursor(10, 8);
  display.print(now.hour());
  display.print(':'); 
  display.print(now.minute());

  display.setCursor(40, 32);
  display.print(windSpeed);
  display.print("Km/h");

  display.setCursor(10, 45);
  display.print(temp);
  display.write(167);
  display.print("C");

  display.setCursor(50, 45);
  display.print("Humidity:");
  display.print(wet);
  display.print("%");

  display.setCursor(10, 57);
  display.print(press);
  display.print("hPa");

  display.setCursor(70, 57);
  display.print(alt);
  display.print("m");
  display.display();
 
WiFi.mode(WIFI_OFF);

// Perform SD card health check every 24 hours
unsigned long currentMillis = millis();
if (currentMillis - lastSDCheckTime >= SD_CHECK_INTERVAL) {
 lastSDCheckTime = currentMillis;
 checkSDCardHealth();
}

recSensorData ();
recWindSpeed ();
    
logAllSensorDataToSD();

delay(250);

updateThingSpeak();

  display.clearDisplay();
  display.display();
  display.setTextSize(2);
  display.setCursor(20, 0);
  display.print("Warning!");

  display.setCursor(15, 20);
  display.print("Going to");

  display.setCursor(5, 40);
  display.print("Sleep Mode");
  display.display();
  display.setTextSize(1);
  
delay(2000);

  display.clearDisplay();
  display.display();
  esp_deep_sleep_start(); //starts the timer for deep sleep
  sendCommand(0xAE); //turn oled off
  
}

void loop() {
//this remains empty with deep sleep
}

// Function to send command to SSD1306
void sendCommand(uint8_t command) {
Wire.beginTransmission(0x3C);
Wire.write(command);
Wire.endTransmission();
}

void recSensorData() {
sensors_event_t temp_event, humidity_event;
int attempts = 6; // Number of retries for each sensor

  // Retry logic for AHT10 sensor
  for (int i = 0; i < attempts; i++) {
  aht.getEvent(&humidity_event, &temp_event);
  temperature = temp_event.temperature;
  humidity = humidity_event.relative_humidity;
  // Check if any reads failed
  if (!isnan(humidity) && !isnan(temperature)) {
  break; // Exit loop if valid readings are obtained
  }

  Serial.println("Failed to read from AHT sensor, retrying...");
  delay(500); // Wait for 500ms before retrying
  }

  // Check if readings are still invalid after retries
  if (isnan(humidity) || isnan(temperature)) {
  Serial.println("Failed to get valid AHT sensor readings after retries!");
  return;
  }

  // Retry logic for BME280 sensor
  for (int i = 0; i < attempts; i++) {
  delay(50); // Small delay before each BME reading
  float rawPressure = bme.readPressure() / 100; // in hPa
  float rawAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // altitude in meters

  // Apply sanity check for pressure readings
  if (rawPressure > 750 && rawPressure < 950) {
  pressure = rawPressure;
  } else {
  Serial.print("Discarded outlier pressure: ");
  Serial.println(rawPressure);
  continue; // Skip to the next attempt
}

  // Apply sanity check for altitude (optional)
  if (rawAltitude > -100 && rawAltitude < 3000) { // Assuming you're not above 3000m altitude
  altitude = rawAltitude;
  } else {
  Serial.print("Discarded outlier altitude: ");
  Serial.println(rawAltitude);
  continue; // Skip to the next attempt
  }
  // If valid readings are obtained, break out of the retry loop
  break;
  }
  // Check if readings are still invalid after retries
  if (isnan(pressure) || isnan(altitude)) {
  Serial.println("Failed to get valid BME sensor readings after retries!");
  return;
 }
}

void recWindSpeed() {
unsigned long currentTime = millis();
  
if (currentTime - lastMeasurementTime >= MEASUREMENT_INTERVAL) {
if (rotationCount > 0) { 
float rotationsPerSecond = rotationCount;
float rpm = rotationsPerSecond * 60.0;
float windSpeed_mps = (circumference_meters * rpm) * calibrationFactor;
int windSpeed_kmh = windSpeed_mps * 3.6;

Serial.print("Wind Speed: ");
Serial.print(windSpeed_mps);
Serial.println("m/s");

windSpeed = windSpeed_kmh;
} else {
windSpeed = 0.0;  // No rotations detected, set wind speed to zero
}

rotationCount = 0;  // Reset for next cycle
lastMeasurementTime = currentTime;
  }
}

void updateThingSpeak() {

connectToWiFi();
float batteryVoltage = readBatteryVoltage();

ThingSpeak.setField(1, temperature);
ThingSpeak.setField(2, humidity);
ThingSpeak.setField(3, pressure);
ThingSpeak.setField(4, altitude);
ThingSpeak.setField(5, windSpeed);
ThingSpeak.setField(6, batteryVoltage);

int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
if (x == 200) {
  
  digitalWrite(blueLED, HIGH);
  display.clearDisplay();
  display.display();
  display.setTextSize(2);
  display.setCursor(5, 0);
  display.println("ThingSpeak");

  display.setCursor(25, 20);
  display.println("Channel");

  display.setCursor(20, 40);
  display.println("Updated!");
  display.setTextSize(1);
  display.display();
  delay(2500);
  digitalWrite(blueLED,LOW);

} else {
  digitalWrite(blueLED, HIGH);
  delay(75);
  digitalWrite(blueLED,LOW);
  delay(75);
  display.clearDisplay();
  display.display();
  display.setCursor(0, 0);
  display.print("Cannot Update Channel");

  display.setCursor(25, 30);
  display.print("HTTP Code ");

  display.setCursor(35, 40);
  display.print(String(x));
  display.display();
  delay(2500);
  digitalWrite(blueLED,LOW);
  }
}

void checkSDCardHealth() {
Serial.println("Checking SD card health...");
if (sd.begin(SD_CS_PIN, SD_SCK_MHZ(10))) {
Serial.println("SD Card is healthy.");
} else {
Serial.println("SD Card error or not found!");
  }
}

void logAllSensorDataToSD() {

DateTime now = rtc.now();
  
// Format the date as yyyyMMdd (e.g., 20250228 for February 28, 2025)
String fileName = String(now.year()) + String(now.month(), DEC) + String(now.day(), DEC) + ".csv";

logFile = sd.open(fileName.c_str(), FILE_WRITE);
if (logFile) {

  logFile.print(now.day()); logFile.print("/");
  logFile.print(now.month()); logFile.print("/");
  logFile.print(now.year()); logFile.print(",");
  logFile.print(now.hour()); logFile.print(":");
  logFile.print(now.minute()); logFile.print(":");
  logFile.print(now.second()); logFile.print(",");

  logFile.print(temperature);
  logFile.print(",");
  logFile.print(humidity);
  logFile.print(",");
  logFile.print(pressure);
  logFile.print(",");
  logFile.print(altitude);
  logFile.print(",");
  logFile.print(windSpeed);
  logFile.print(",");
  logFile.println(readBatteryVoltage());

  logFile.close();
  Serial.println("Data logged to: " + fileName);
  } else {
  Serial.println("Error opening file!");
  }
}