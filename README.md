# ATLAS-CODE
# 🛰️ ATLAS_ASCEND 1.2 - Disaster Warning CubeSat

![Status](https://img.shields.io/badge/Status-Release_Candidate-success)
![Hardware](https://img.shields.io/badge/Hardware-ESP32--S3_%7C_ESP32--CAM-blue)
![Telemetry](https://img.shields.io/badge/Telemetry-LoRa_433MHz-orange)
![Platform](https://img.shields.io/badge/Platform-Arduino_IDE_%7C_FreeRTOS-green)

> **Team:** VN7-ATLAS  
> **Event:** ASCEND 2026  
> **Version:** 5.0 (Master-Slave Distributed Architecture)

## 📖 Project Overview

**ATLAS_ASCEND 1.2** is a CubeSat flight software designed for disaster monitoring (landslide detection & environmental sensing). 

Unlike traditional single-core systems, this project utilizes a **Distributed Master-Slave Architecture** to handle high-resolution image processing without compromising real-time telemetry stability.

### 🏗️ System Architecture

The system consists of two independent processing units communicating via High-Speed UART (460800 baud):

1.  **MASTER NODE (OBC - On Board Computer):**
    * **Hardware:** ESP32-S3 DevKit V1.
    * **Role:** Mission Control. It handles GPS navigation, environmental sensing (BME280/MPU6050), and Long-Range communication (LoRa). It acts as the "Brain".
2.  **SLAVE NODE (Payload):**
    * **Hardware:** AI Thinker ESP32-CAM.
    * **Role:** Image Acquisition. It handles the camera (OV2640), SD Card buffering, and data fragmentation. It acts as the "Eye".

---

## 🚀 Key Engineering Features

### 1. Sequential Image Transmission (Chunking Protocol)
Sending a 50KB image directly via UART or LoRa would cause a **Stack Overflow** or block the main thread for seconds. We solved this by implementing a **"Chunking & Flow Control"** mechanism:
* **Step 1:** Slave saves the full image to the SD Card (Buffer).
* **Step 2:** Master requests image size (Handshake).
* **Step 3:** Master requests data in small **200-byte chunks**.
* **Step 4:** Slave seeks to the specific file offset and sends raw binary data.
* **Result:** Zero RAM overflow, stable transmission even with large files.

### 2. Multi-threading with FreeRTOS
The Master Node utilizes the dual-core architecture of the ESP32-S3:
* **Core 0 (TaskSystem):** Handles critical sensors, GPS, and LoRa telemetry (High Priority).
* **Core 1 (TaskVision):** Handles the time-consuming image transfer process (Lower Priority).
* **Benefit:** The satellite never "freezes" while processing images. Telemetry is sent every 5 seconds regardless of camera status.

### 3. Hardware Optimization
* **High-Speed UART:** Overclocked internal UART to **460800 baud** (4x standard speed) to minimize latency between Master and Slave.
* **Brownout Protection:** Disabled Brownout Detector on ESP32-CAM to prevent resets during high-current spikes (Flash/SD write).
* **1-Bit SD Mode:** Configured SD Card in 1-bit mode to free up pins and avoid conflict with the on-board Flash LED.

---

## 🔌 Pin Mapping (Wiring)

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

## 🛠️ Installation & Setup

### Prerequisites
* **IDE:** Arduino IDE 2.x
* **Board Manager:** `esp32` by Espressif Systems (v2.0.11 or newer).

### Required Libraries
Install these via Arduino Library Manager:
1.  `LoRa` by Sandeep Mistry
2.  `TinyGPSPlus` by Mikal Hart
3.  `Adafruit Unified Sensor`
4.  `Adafruit BME280 Library`
5.  `Adafruit MPU6050`
6.  `Adafruit INA219`

### Flashing Instructions
1.  **Master Node:**
    * Select Board: `ESP32S3 Dev Module`.
    * **USB CDC On Boot:** "Enable" (Important for Serial Monitor).
    * Upload `Master_Node/Master_Node.ino`.
2.  **Slave Node:**
    * Select Board: `AI Thinker ESP32-CAM`.
    * **PSRAM:** "Enabled" (Critical for High-Res images).
    * **Partition Scheme:** "Huge APP".
    * Upload `Slave_Node/Slave_Node.ino`.

---

## 📊 Telemetry Format

**Downlink Packet (CSV Format):**
`TM:[VOLT],[LAT],[LON],[ALT],[STATUS]`

* `VOLT`: Battery Voltage (V)
* `LAT/LON`: GPS Coordinates (Decimal Degrees)
* `ALT`: Altitude (Meters)
* `STATUS`: System Health Flag (e.g., "OK", "ERR_CAM")

**Example:**
`TM:4.15,21.028511,105.854200,150.5,OK`

---

## 👨‍💻 Authors & Acknowledgments

**Team VN7-ATLAS**
* Lead Developer: [Le Duc Minh]

*Special thanks to the open-source community for the libraries used in this project.*
