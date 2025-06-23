# 🌿 Offline Smart Plant Care System

An embedded system project that automates plant irrigation using real-time sensor data—no internet required.

## 🚀 Overview

This system monitors soil moisture, temperature, and humidity using sensors and irrigates automatically when moisture levels drop below a threshold. It operates completely offline using an ATmega328P microcontroller.

## 🌱 Features

- Real-time LCD display of temperature, humidity, and soil moisture
- Automatic irrigation control using a water pump
- Visual indicators (LED) for plant health and watering status
- Operates without internet or cloud support

## ⚙️ Hardware Components

- ATmega328P Microcontroller
- DHT11 Sensor
- Soil Moisture Sensor
- 16x2 LCD Display
- Relay Module (SRD-05VDC-SL-C)
- Water Pump + Tube
- LED
- 5V Regulated Battery Supply
- Misc: Resistors, wires, connectors

## 💻 Software Architecture

The code is modular and written in C using AVR libraries. Main modules include:

- `main.c`: Orchestrates sensor reads, watering logic, and includes the LCD.
- `dht11.h`: Interfaces with the DHT11 sensor to read temperature and humidity (see References below)


## 📊 Schematic & Flow Diagram

- Check the `schematic/` directory for circuit visuals
- Flowchart illustrating data flow from sensor read to irrigation

![flowchart](https://github.com/user-attachments/assets/897b4dd5-ed84-466d-8965-9790ccb5632f)


## 🧪 Testing Summary

| Action                     | Expected Outcome                   | Actual Outcome | Result |
|---------------------------|------------------------------------|----------------|--------|
| Power on                  | Temp: 24°C, Hum: 60%, Moist: OK     | As expected    | ✅ Pass |
| Dry Soil Detected         | Watering triggered, LED blinks     | As expected    | ✅ Pass |

## 📌 Usage

1. Power the system with 5V
2. Watch LCD for data updates every 2 seconds
3. System will water if moisture falls below threshold

## 📚 References

- [DHT11 Datasheet](link)
- [AVR-DHT11 Library](https://github.com/ryanj1234/AVR-DHT11)

