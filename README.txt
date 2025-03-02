## ESP32 Weather Station

## Overview
This project is an ESP32-based weather station that collects environmental data using multiple sensors and logs the data to an SD card, displays it on an OLED screen, and optionally uploads it to ThingSpeak. The ESP32 enters deep sleep mode between readings to conserve power and can be manually awakened using a button connected to pin 4.

## Components Used
- **ESP32** (Main microcontroller)
- **AHT10** (Temperature and Humidity Sensor)
- **BME280** (Pressure Sensor)
- **3144 Hall Sensor** (Wind Speed Measurement)
- **DS3231 RTC** (Real-Time Clock for timestamping)
- **SSD1306** (0.96" OLED Display for real-time monitoring)
- **MicroSD Card Module** (For data logging)


## Features
- Logs temperature, humidity, pressure, altitude, wind speed, and battery voltage.
- Displays readings on an OLED screen.
- Stores data on an SD card with a new CSV file for each day.
- Sends data to **ThingSpeak** (Optional; requires API key and WiFi setup).
- Uses **Deep Sleep** to extend battery life, with configurable sleep duration.
- A wake-up button on **Pin 4** allows manual activation.



## Setup & Configuration
### 1. **WiFi & ThingSpeak Configuration**
Modify the following parameters in the sketch:
```cpp
const char* ssid = "Your_WiFi_SSID";
const char* password = "Your_WiFi_Password";
unsigned long myChannelNumber = Your_ThingSpeak_Channel;
const char* myWriteAPIKey = "Your_ThingSpeak_API_Key";
```
If you do not want to use ThingSpeak, you can disable WiFi functions in the code.



### 2. **Deep Sleep Settings**
The ESP32 is set to sleep for **900 seconds (15 minutes)** by default. You can adjust this in the code:
```cpp
#define Sleep_Time 900 // Time in seconds before ESP32 wakes up
```



### 3. **Wind Speed Sensor & Calculation**
The wind speed is measured using a **3144 Hall Effect sensor** with a magnet attached to the rotating anemometer. The formula used:
```cpp
float windSpeed_mps = (circumference_meters * rpm) * calibrationFactor;
int windSpeed_kmh = windSpeed_mps * 3.6;
```
- `circumference_meters` is based on the anemometer's diameter.
- `calibrationFactor` may need adjustments based on real-world testing.



### 4. **SD Card Logging**
Each day, a new file is created with the format `YYYYMMDD.csv`, containing:
```
Date,Time,Temperature (C),Humidity (%),Pressure (hPa),Altitude (m),Wind Speed (km/h),Battery Voltage (V)
```
Data is appended every 15 minutes when the ESP32 wakes up.



## Future Improvements
- **Wind Direction Sensor**: Possible implementation using a QMC5883L magnetometer or multiple Hall sensors.
- **Data Smoothing**: Improve wind speed accuracy by averaging multiple readings.
- **Alternative Data Uploads**: MQTT or other cloud services for more flexibility.

## License
This project is open-source and free to use. Feel free to modify and improve!

