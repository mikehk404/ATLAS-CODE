# 📡 ATLAS_ASCEND 1.2 - Integrated Disaster Warning Ecosystem

![Status](https://img.shields.io/badge/Status-Release_Candidate-success?style=flat-square)
![Hardware](https://img.shields.io/badge/Hardware-ESP32_S3_%7C_ESP32_CAM_%7C_ESP32_WROOM-blue?style=flat-square)
![Telemetry](https://img.shields.io/badge/Telemetry-LoRa_433MHz-orange?style=flat-square)
![AI-Engine](https://img.shields.io/badge/AI_Engine-Scikit_Learn_%7C_RandomForest-red?style=flat-square)

> **Team:** VN7-ATLAS  
> **Event:** ASCEND 2026  
> **Version:** 5.2 (Hybrid Ground-Satellite Architecture)

---

## 📖 Project Overview

**ATLAS_ASCEND 1.2** is a comprehensive disaster monitoring ecosystem designed for infrastructure-denied regions. It integrates a distributed **Ground Sensor Network (GSN)** with a **CubeSat Relay Node** and an **AI-driven Ground Station** to provide resilient early warnings for landslides, floods, and forest fires.

The system moves beyond traditional "Threshold-based" monitoring by implementing **"Proactive Detection"** philosophy, utilizing **Rate-of-Change (RoC)** algorithms at the edge and **Machine Learning** at the core.

---

## 🏗️ System Architecture

The ecosystem is defined by three synchronized operational layers:

### 1. 🌍 GROUND SEGMENT (The "Fab Four" Sensor Nodes)
Distributed autonomous nodes deployed in high-risk zones (mountains, riverbanks).
* **MCU:** ESP32-WROOM-32 (Optimized for Deep Sleep).
* **Sensor Stack ("The Fab Four"):**
    1.  **Soil:** Capacitive Moisture Sensor (Landslide/Flood Saturation).
    2.  **Kinematics:** MPU-6050 6-DOF IMU (Seismic Activity/Slope Stability).
    3.  **Climate:** BME280 (Temp/Hum/Pressure for Fire & Storm Prediction).
    4.  **Geolocation:** NEO-M8N GNSS (Precise Hazard Mapping).
* **Edge Logic:** Performs **Rate-of-Change (RoC)** analysis to detect rapid environmental shifts (e.g., Flash Floods) before critical thresholds are reached.

### 2. 🛰️ SPACE SEGMENT (1U CubeSat Relay)
A 1U Satellite acting as a Data Relay and Optical Verification node.
* **Master Node (ESP32-S3):** Handles Navigation (GPS), Telemetry (LoRa), and Mission Control via **FreeRTOS**.
* **Slave Node (ESP32-CAM):** Dedicated to High-Resolution Imaging and buffering via a custom **Binary Chunking Protocol**.

### 3. 🖥️ PROCESSING SEGMENT (AI Ground Station)
A centralized server running Python & Scikit-learn.
* **Algorithm:** **Random Forest Regression**.
* **Function:** Analyzes multi-variate correlations (Pressure Drop + Soil Saturation + Vibration) to predict disaster probability scores (0-100%).

---

## 🚀 Key Engineering Innovations

### A. "Rate-of-Change" (RoC) Trigger
Unlike passive loggers that only alarm at static thresholds (e.g., >90% moisture), ATLAS nodes calculate the **first derivative (velocity)** of sensor data.
* *Scenario:* If soil moisture spikes >10% in 5 minutes, the system triggers a **"Flash Flood Alert"** immediately, even if the absolute value is only 50%.

### B. Master-Slave Satellite Core
To prevent the satellite from "freezing" during image processing, we utilize a dual-core distributed architecture:
* **Core 0 (Master):** Handles Telemetry & Navigation (High Priority).
* **Core 1 (Slave):** Handles Image Compression & Fragmentation (Low Priority).

### C. Adaptive Power Management
* **Normal Mode:** Deep Sleep for 5 minutes (<20µA).
* **Urgent Mode:** Deep Sleep for 1 minute (Active Monitoring) upon detecting RoC anomalies.

---

## 🔌 Pin Mapping (Hardware Wiring)

### 1. Ground Sensor Node (ESP32-WROOM)
| Component | Pin Name | GPIO (ESP32) | Note |
| :--- | :--- | :--- | :--- |
| **LoRa (SX1278)** | NSS (CS) | 5 | SPI Bus |
| | RST | 14 | |
| | DIO0 | 2 | Interrupt |
| **I2C Bus** | SDA | 42 | BME280 + MPU6050 |
| | SCL | 41 | BME280 + MPU6050 |
| **Soil Sensor** | Signal | 34 | **Analog Input Only** |
| **GPS (NEO-M8N)** | TX | 16 | Connect to GPS TX |
| | RX | 17 | Connect to GPS RX |

### 2. Satellite
### Master Node (ESP32-S3)
| Component | Pin Name | GPIO (S3) | Note |
| :--- | :--- | :--- | :--- |
| **LoRa (Ra-02)** | MISO | 13 | SPI Bus |
| | MOSI | 11 | SPI Bus |
| | SCK | 12 | SPI Bus |
| | NSS (CS) | 10 | |
| | RST | 5 | |
| | DIO0 | 6 | |
| **Sensors** | SDA | 42 | I2C Bus |
| | SCL | 41 | I2C Bus |
| **GPS (NEO-M8N)** | TX | 18 | Connect to GPS TX |
| | RX | 17 | Connect to GPS RX |
| **Slave Comm** | RX | 16 | **Connect to CAM U0T** |
| | TX | 15 | **Connect to CAM U0R** |

### Slave Node (ESP32-CAM)
*Note: The ESP32-CAM uses internal wiring for the Camera and SD Card. Only UART and Power need to be connected.*

| Pin Name | Connection | Description |
| :--- | :--- | :--- |
| **5V** | 5V Source | Requires stable >500mA source |
| **GND** | Common GND | **Must connect to Master GND** |
| **U0R** | Master TX (15) | Receive Commands |
| **U0T** | Master RX (16) | Transmit Image Data |

---

## 📊 Telemetry Data Format

### 1. Ground Node Packet (Uplink)
Format: `GN,[ID],[TYPE],[TEMP],[HUM],[PRES],[SOIL],[VIB],[LAT],[LON]`

| Field | Description | Example |
| :--- | :--- | :--- |
| **TYPE** | Hazard Class | `NORMAL`, `FIRE_RISK`, `STORM_ALERT`, `LANDSLIDE` |
| **PRES** | Pressure (hPa) | `990.2` (Low pressure indicates storm) |
| **VIB** | Vibration (G) | `1.5` (Seismic activity) |

### 2. Satellite Heartbeat (Downlink)
Format: `TM:[VOLT],[LAT],[LON],[ALT],[STATUS]`

---

## 🛠️ Installation & Setup

### Directory Structure
```text
VN7-ATLAS_ASCEND/
├── README.md               # This file
├── Ground_Node/            # Firmware for Fab Four Sensors
│   └── Ground_Node_Firmware.ino
├── Sat_Master/             # Firmware for Satellite OBC
│   └── Master_Node.ino
├── Sat_Slave/              # Firmware for Satellite Camera
│   └── Slave_Node.ino
└── AI_Ground_Station/      # Python Machine Learning Core
    ├── disaster_model.py
    └── nodes_data.txt
