# Alzheimer's Health and Location Tracking System

## Overview

This project aims to provide a comprehensive health and location tracking solution for individuals with Alzheimer's disease. Leveraging an ESP32 NodeMCU microcontroller and various sensors, the system monitors health parameters such as heart rate, SpO2, body temperature, and location using GPS. The collected data is then uploaded to ThingSpeak through a GSM module for remote monitoring.

## Features

- **Real-time GPS tracking**: Utilizes a GPS module to continuously monitor and track the user's location in real-time.
- **Health parameter monitoring**: Includes a MAX30102 Pulse Oximeter Sensor to measure heart rate and SpO2, and an LM35 Temperature Sensor for body temperature monitoring.
- **GSM connectivity**: Employs a SIM800L GSM Module for seamless data upload to ThingSpeak, enabling remote health monitoring.
- **Arduino-based implementation**: Developed using the VScode and libraries from platformio for easy replication and customization.

## Hardware Requirements

- **ESP32 NodeMCU**
- **MAX30102 Pulse Oximeter Sensor**
- **LM35 Temperature Sensor**
- **SIM800L GSM Module**
- **GPS Module**
- **LEDs and resistors (for indication)**

## Libraries Used

- **TinyGPS++**: Library for parsing NMEA data from GPS modules.
- **ThingSpeak**: Enables easy integration with the ThingSpeak platform for data logging.
- **MAX30105**: Library for interfacing with the MAX30105 Pulse Oximeter Sensor.
- **ESP_LM35**: Library for LM35 Temperature Sensor.
- **SoftwareSerial**: Allows serial communication on other digital pins.

## Setup Instructions

1. **Hardware Connection**: Connect sensors and modules to the ESP32 NodeMCU following the provided pin configurations in the code.
2. **Library Installation**: Install necessary libraries using the Arduino Library Manager.
3. **Code Configuration**: Open the main file in the VScode and configure parameters such as GSM APN details, ThingSpeak API key, and sensor pins.
4. **Upload Code**: Upload the code to the ESP32 NodeMCU.

## How It Works

1. **GPS Tracking**: The `loopGPS()` function reads GPS data using the TinyGPS++ library, updating latitude and longitude variables. Location data is printed to the serial monitor.

2. **Temperature Monitoring**: The `loopTempSensor()` function reads temperature from the LM35 sensor, calculates the average over time, and prints the results to the serial monitor.

3. **Pulse Oximetry**: The `loopPulse()` function continuously reads data from the MAX30105 Pulse Oximeter Sensor. Heart rate and SpO2 values are calculated and printed to the serial monitor.

4. **Data Upload to ThingSpeak**: The `ThingSpeakUpload()` function connects to the GSM network using the SIM800L module and uploads GPS coordinates, average temperature, heart rate, and SpO2 values to ThingSpeak.

## Usage

1. Power on the device.
2. The ESP32 NodeMCU will start monitoring health parameters and GPS location.
3. Data will be uploaded to ThingSpeak via GSM for remote monitoring.
4. Check the ThingSpeak channel for real-time data updates.

## Results and Discussion

### ThingSpeak Data Visualization

- The health and location data is successfully uploaded to ThingSpeak, providing a visual representation of the user's well-being and location.

#### ThingSpeak Channel Link:
[Link to ThingSpeak Channel](#)

### Telegram Notification System

- The system incorporates a Telegram-based alert system for immediate notifications based on abnormal health parameters or locations.

#### Telegram Alert Example:
![Telegram Notification Example](path/to/telegram_notification.png)


## Acknowledgements

## Author

Abang Amirulluqman Farhan bin Abang Kilat

Feel free to explore the code and contribute to the project!
