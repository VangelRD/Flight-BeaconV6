# V-Flight Computer

Robust flight data acquisition, logging, and telemetry for model rockets. The system uses a dual‑ESP32 architecture: a sensor-rich transmitter (TX) and a ground/bench receiver (RX) that provides local logging.

## Current Implementation Status

**THIS IS A STRICTLY BAREBONES IMPLEMENTATION** - Keep it minimal and simple at all costs:
- NO unnecessary features or abstractions
- NO complex frameworks or libraries beyond essential sensor/LoRa drivers
- NO fancy GUIs, web interfaces, or over-engineered solutions
- Focus on core functionality only:
  - Reading sensors at maximum reliable rates
  - Local logging at ~100Hz
  - LoRa transmission at ~5Hz
  - Terminal display and logging on RX

**Keep the code simple, direct, and maintainable. Avoid feature creep.**

See [QUICKSTART.md](QUICKSTART.md) for immediate usage instructions.

## Table of Contents
- [Overview](#overview)
- [Hardware Specifications](#hardware-specifications)
- [Hardware Pin Configuration](#hardware-pin-configuration)
- [System Architecture](#system-architecture)
- [Software Features](#software-features)
- [Data Logging](#data-logging)
- [Project Structure](#project-structure)
- [Installation and Setup](#installation-and-setup)
- [Usage](#usage)
- [LoRa Settings](#lora-settings)
- [Sensor Sampling Rates](#sensor-sampling-rates)
- [Performance Characteristics](#performance-characteristics)
- [Troubleshooting](#troubleshooting)
- [Project Status](#project-status)
- [Contributing](#contributing)
- [License](#license)
- [Documentation](#documentation)
- [Support](#support)
- [Acknowledgments](#acknowledgments)
- [Safety and Legal](#safety-and-legal)

## Overview

The V‑Flight Computer consists of two main components:
- **Transmitter (TX)**: Collects sensor data from onboard sensors, logs it locally, and transmits telemetry via LoRa.
- **Receiver (RX)**: Receives telemetry and logs it locally.

### Design Priorities
The primary design goals of this project are:
- **Barebones Simplicity**: Keep implementation minimal, direct, and maintainable - NO unnecessary complexity
- **High-Speed Local Logging**: Maximize the TX logging rate to capture high-frequency sensor data without data loss
- **Fast Telemetry Transmission**: Optimize TX transmission speed to deliver real-time data over LoRa with minimal latency

These priorities ensure comprehensive flight data capture and responsive telemetry for critical mission phases, while maintaining code simplicity.

## Hardware Specifications

### Transmitter (TX) Hardware
- **Microcontroller**: ESP32 (240 MHz dual‑core; typical boards have ~520 KB SRAM; flash size depends on module, often up to 16 MB)
- **Pressure/Temperature Sensor**: BMP280 (barometric pressure and temperature)
- **IMU (Inertial Measurement Unit)**: LSM9DS1 (9‑axis: accelerometer, gyroscope, magnetometer)
- **GNSS Module**: LC76G Multi‑GNSS
  - Supports GPS, BDS, GLONASS, Galileo, QZSS
  - High‑precision positioning for location and velocity data
  - [Product Link](https://grobotronics.com/multi-gnss-module-lc76g-gps-bds-glonass-galileo-qzss.html)
- **Wireless**: LoRa Module 433 MHz – SX1278 (Ra‑02)
  - Long‑range LoRa communication based on Semtech SX1278
  - [Product Link](https://grobotronics.com/lora-module-433mhz-sx1278.html)

### Receiver (RX) Hardware
- **Microcontroller**: ESP32 (generic development board)
- **Wireless**: LoRa Module 433 MHz – SX1278 (Ra‑02)
  - Matches the TX module for reliable link
  - [Product Link](https://grobotronics.com/lora-module-433mhz-sx1278.html)

## Hardware Pin Configuration

### Transmitter (TX) Pin Mapping

#### I2C Bus (SDA=21, SCL=22)
- **LSM9DS1 Accelerometer/Gyroscope**: I2C address (configurable)
- **LSM9DS1 Magnetometer**: I2C address (configurable, same bus)
- **BMP280 Barometric Sensor**: I2C address 0x76 or 0x77 (same bus)

#### SPI Bus (LoRa Module)
- **MOSI**: GPIO 23
- **MISO**: GPIO 19
- **SCK**: GPIO 18
- **NSS/CS**: GPIO 25
- **RST**: GPIO 14
- **DIO0**: GPIO 4

#### UART1 (GNSS Module)
- **RX**: GPIO 16
- **TX**: GPIO 17


### Receiver (RX) Pin Mapping

#### SPI Bus (LoRa Module)
- **MOSI**: GPIO 23
- **MISO**: GPIO 19
- **SCK**: GPIO 18
- **NSS/CS**: GPIO 25
- **RST**: GPIO 14
- **DIO0**: GPIO 4


## System Architecture

### Data Flow
```
Sensors → ESP32 TX → LoRa TX → LoRa RX → ESP32 RX → Local Storage
```

### Transmitter (TX) Responsibilities
1. **Sensor Data Acquisition**: Read data from connected sensors at appropriate rates.
2. **Local Data Logging**: Store time‑stamped sensor data on onboard flash.
3. **Wireless Transmission**: Send telemetry via LoRa with tunable trade‑offs among reliability, speed, and range.

### Receiver (RX) Responsibilities
1. **Data Reception**: Receive telemetry via LoRa.
2. **Local Data Storage**: Log received data for backup and analysis.

## Software Features

### Sensor Integration
- **BMP280**: Barometric altitude, temperature, pressure
- **LSM9DS1**: 3‑axis acceleration, angular velocity, magnetic field
- **LC76G**: GPS coordinates, altitude, speed, heading, satellite info

## Data Logging
- **Filesystem**: SPIFFS or LittleFS on ESP32 flash (LittleFS generally recommended on newer cores)
- **Data Format**: CSV or newline‑delimited JSON with timestamps
- **Capacity**: Depends on board flash size and partitioning (commonly up to 16 MB total flash)
- **Retention**: Rolling logs with size‑based rotation

## Project Structure

```
V_Flight_Computer/
├── tx/                          # Transmitter firmware
├── rx/                          # Receiver firmware
└── docs/                        # Documentation and schematics
```

## Installation and Setup

### Prerequisites
- PlatformIO (VS Code extension or CLI) , or ESP‑IDF (advanced)
- ESP32 development boards (TX and RX)
- Required sensors and LoRa modules
- USB cables for programming

### Hardware Setup

#### Transmitter Setup
1. **I2C Sensor Connections** (SDA=GPIO21, SCL=GPIO22):
   - BMP280 at 0x76 or 0x77
   - LSM9DS1 (accel/gyro and magnetometer on same bus)
2. **LoRa Module (SPI):**
   - MOSI → GPIO 23
   - MISO → GPIO 19
   - SCK → GPIO 18
   - NSS/CS → GPIO 25
   - RST → GPIO 14
   - DIO0 → GPIO 4
3. **GNSS Module (UART1):**
   - GNSS TX → ESP32 RX (GPIO 16)
   - GNSS RX → ESP32 TX (GPIO 17)
4. **Power and Ground:**
   - Stable 3.3 V supply and common ground

#### Receiver Setup
1. **LoRa Module (SPI – identical to TX)**
   - MOSI → GPIO 23, MISO → GPIO 19, SCK → GPIO 18, NSS/CS → GPIO 25, RST → GPIO 14, DIO0 → GPIO 4
2. **Power Supply**
   - Stable power for continuous operation

### Software Installation

#### Option A: PlatformIO 
1. Install PlatformIO:
   - VS Code extension: PlatformIO IDE
   - Or CLI: `pipx install platformio` (or `pip install platformio`)
2. Open this project folder in VS Code (PlatformIO will auto‑detect).
3. Select the appropriate environments for TX and RX and build:
   - `pio run -e tx`
   - `pio run -e rx`
4. Flash the boards:
   - `pio run -e tx -t upload`
   - `pio run -e rx -t upload`
5. (Optional) Monitor serial output at 115200 baud:
   - `pio device monitor -b 115200`

#### Option B: ESP‑IDF (advanced)
1. Install ESP‑IDF (v5.x recommended) and set up the toolchain.
2. Set target: `idf.py set-target esp32`.
3. Configure, build, and flash with `idf.py menuconfig`, `idf.py build`, `idf.py flash`, and `idf.py monitor`.

## Usage
1. Power up TX and RX; confirm LoRa link (LEDs/serial logs).
2. Adjust LoRa parameters and sensor rates as needed for your mission profile.
3. Download logs from RX or TX for post‑flight analysis.

## LoRa Settings
- **Region/Frequency**: 433 MHz band (ensure compliance with your local regulations; e.g., 433.05–434.79 MHz ISM sub‑band with duty‑cycle limits in many regions)
- **Spreading Factor (SF)**: 7–12 (higher SF = longer range, lower rate)
- **Bandwidth (BW)**: 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500 kHz
- **Coding Rate (CR)**: 4/5, 4/6, 4/7, 4/8 (higher CR = more robust, slower)
- **TX Power**: Configurable (observe legal EIRP limits and use appropriate antennas)

### Approximate LoRa PHY Bitrates (BW=125 kHz, CR=4/5)
- **SF7** ≈ 5.5 kbps
- **SF8** ≈ 3.1 kbps
- **SF9** ≈ 1.8 kbps
- **SF10** ≈ 1.0 kbps
- **SF11** ≈ 0.54 kbps
- **SF12** ≈ 0.29 kbps

Payload‑level throughput will be lower due to LoRaWAN/packet overhead and inter‑packet timing. Wider bandwidth (e.g., 250/500 kHz) increases bitrate but reduces sensitivity and range.

### Recommended Configuration for 2 km+ Line‑of‑Sight
- **SF9–SF10**, **BW=125 kHz**, **CR=4/6–4/7**
- Expected payload throughput: ~0.5–1.5 kbps
- Packet size: 100–200 bytes
- Update frequency: 1–5 Hz (aggregate sensor set)

## Sensor Sampling Rates

### BMP280 (Barometric Pressure/Temperature)
- **Typical**: ~10 Hz
- **Maximum**: ~157 Hz (with oversampling)

### LSM9DS1 (9DOF IMU)
- **Accelerometer**
  - Typical: 100–200 Hz (flight dynamics)
  - Max: up to 952 Hz (normal mode options include ~238/476/952 Hz)
- **Gyroscope**
  - Typical: 100–200 Hz (attitude estimation)
  - Max: up to 952 Hz
- **Magnetometer**
  - Typical: 20–50 Hz (heading)
  - Max: ~80 Hz; low‑power options include ~0.6/1.25/2.5/5/10 Hz

### LC76G (Multi‑GNSS)
- **Typical**: 1 Hz
- **Maximum**: up to 10 Hz (module‑dependent configuration)

## Performance Characteristics

### Transmission Range
- **Target**: 2 km+ line‑of‑sight with suitable antennas and tuning
- Actual range depends on antennas, interference, altitude, and environment

### Data Throughput
- **Packet size**: 50–200 bytes recommended
- **Update rate**: 1–5 Hz typical for complete sensor datasets
- **Latency**: ~100–500 ms depending on packetization and link settings
- **Effective throughput**: ~0.2–2.0 kB/s depending on SF/BW/CR and overhead

### Power Consumption
- **TX unit**: Depends on sensors and duty cycle; expect ~80–200 mA baseline with LoRa TX bursts adding up to ~120 mA (module‑dependent)
- **RX unit**: Continuous operation recommended on stable power

## Troubleshooting

### Common Issues
- **LoRa link fails**: Verify antenna, frequency plan, SF/BW/CR match, and duty‑cycle limits
- **Sensor not detected**: Check I2C/SPI wiring, pull‑ups, and addresses

## Project Status
- Active development; hardware validated; firmware under iterative improvement

## Contributing

Contributions are welcome! Please open issues for bugs/ideas and submit pull requests for improvements.

## License

No license file is currently present in this repository. Until a license is added, all rights are reserved. If you intend to use or contribute, please propose a license (e.g., MIT, Apache‑2.0) via a pull request.

## Documentation

Project documentation and schematics live in the `docs/` folder (create if not present). Include hardware pinouts, configuration guides, and troubleshooting notes.

## Support

For technical support or questions:
- Open an issue in this repository
- Review documentation in [`docs/`](docs/)
- Check troubleshooting guides above

## Acknowledgments
- Arduino‑ESP32 core and PlatformIO ecosystem
- Semtech SX127x reference designs
- Bosch Sensortec BMP280 documentation
- STMicroelectronics LSM9DS1 documentation
- Quectel LC76G documentation

## Safety and Legal
This project is for experimental and educational use only. Comply with your local radio regulations (frequency, power, and duty cycle limits) and aviation safety rules at all times.

Built with the Arduino framework for ESP32 for ease of development (ESP‑IDF also supported with additional integration work).

## Common issues
- Lora packets too big
- Lora reciver isnt in continious mode or is unset from continiouse mode after a while 
- Lora keyword / settings mismatch 