# ATLAS_ASCEND 1.2 - Integrated Disaster Warning Ecosystem

> **Team:** VN7-ATLAS  
> **Event:** ASCEND 2026  
> **Version:** 5 (Hybrid Ground-Satellite Architecture)

---

## Project Overview

**ATLAS_ASCEND 1.2** is a comprehensive disaster warning ecosystem designed for infrastructure-denied regions. It integrates a distributed **Ground Sensor Network (GSN)** with a **CubeSat Relay Node** and an **AI-driven Ground Station** to provide reliable early warnings for landslides, floods, and forest fires.

The system moves beyond traditional "Threshold-based" monitoring by implementing **"Proactive Detection"** philosophy, utilizing **Rate-of-Change (RoC)** algorithms at the edge and **Machine Learning** at the core.

---

## System Architecture

The ecosystem is defined by three synchronized operational layers:

### 1. GROUND SEGMENT (Ground Sensors Node - GNS)
Distributed autonomous nodes deployed in high-risk zones (mountains, riverbanks).
* **MCU:** ESP32-WROOM-32 (Optimized for Deep Sleep).
* **Sensor Stack ("The Fab Four"):**
    1.  **Soil:** Capacitive Moisture Sensor (Landslide/Flood Saturation).
    2.  **Kinematics:** MPU-6050 6-DOF IMU (Seismic Activity/Slope Stability).
    3.  **Climate:** BME280 (Temp/Hum/Pressure for Fire & Storm Prediction).
    4.  **Geolocation:** NEO-M8N GNSS (Precise Hazard Mapping).
* **Edge Logic:** Performs **Rate-of-Change (RoC)** analysis to detect rapid environmental shifts (e.g., Flash Floods) before critical thresholds are reached.

### 2. SPACE SEGMENT (1U CubeSat Relay)
A 1U Satellite acting as a Data Relay and Optical Verification node.
* **Master Node (ESP32-S3):** Handles Navigation (GPS), Telemetry (LoRa), and Mission Control via **FreeRTOS**.
* **Slave Node (ESP32-CAM):** Dedicated to High-Resolution Imaging and buffering via a custom **Binary Chunking Protocol**.

### 3. PROCESSING SEGMENT (AI Ground System)

The Ground Station functions as the ecosystem's **Central Intelligence Core**, designed to transition disaster management from *reactive monitoring* to *proactive forecasting*. It performs **Multi-Modal Data Fusion**, aggregating real-time telemetry from the Ground Mesh Network with satellite imagery baselines.

#### A. Core Architecture: Sensor Fusion & Machine Learning 
The system utilizes a **Random Forest Regressor** (via `scikit-learn`) to model non-linear environmental correlations, generating a precise **Disaster Probability Score ($0-100\%$)**.

| Input Source | Data Type | Function in Model |
| :--- | :--- | :--- |
| **Ground Nodes** | *Dynamic* | Real-time monitoring of **Soil Moisture**, **Seismic Vibration ($G$)**, and **Pressure Trends ($\Delta P$)**. |
| **Satellite** | *Static* | **NDVI (Vegetation Index)** establishes a baseline risk. *Logic: Low NDVI (Deforestation) = High Soil Instability Factor.* |

#### B. Temporal Analytics: "Rate-of-Change" (RoC) Logic 
Instead of relying solely on static thresholds, the engine analyzes **Time-Series Data** to detect rapid anomalies:
* **Derivative Analysis ($d/dt$):** Calculates the velocity of sensor changes. A moisture spike of **$>10\%$ in 5 mins** triggers a "Flash Flood" alert even if absolute saturation is below critical limits.
* **Noise Filtration:** Implements statistical averaging algorithms to reject sensor jitter and false positives.

#### C. Geospatial Intelligence (GIS Dashboard) 
The Python engine (utilizing `Matplotlib` & `Cartopy`) transforms raw telemetry into actionable decision support layers:
* **Dynamic Risk Polygons:** Applies **Convex Hull** algorithms to delineate high-risk clusters dynamically.
* **Wildfire Heatmaps:** Generates interpolated risk layers based on Temperature/Humidity anomalies and Fire Weather Index (FWI).
* **Predictive Forecasting:** Plots 7-day trend projections by comparing Historical Actuals vs. AI-Predicted models.

---
**Tech Stack:** `Python 3.9+` • `Scikit-learn` • `NumPy` • `Cartopy` • `SciPy`

---

## Key Engineering Innovations

### A. "Rate-of-Change" (RoC) Trigger
Unlike passive loggers that only alarm at static thresholds (e.g., >90% moisture), ATLAS nodes calculate the **first derivative (velocity)** of sensor data.
* *Scenario:* If soil moisture spikes **>10% in 5 minutes**, the system triggers a **"Flash Flood Alert"** immediately, even if the absolute value is only 50%.

### B. Master-Slave Satellite Core
To prevent the satellite from "freezing" during image processing, we utilize a dual-core distributed architecture:
* **Core 0 (Master):** Handles Telemetry & Navigation (High Priority).
* **Core 1 (Slave):** Handles Image Compression & Fragmentation (Low Priority).

### C. Adaptive Power Management
* **Normal Mode:** Deep Sleep for 5 minutes (<20µA).
* **Urgent Mode:** Deep Sleep for 1 minute (Active Monitoring) upon detecting RoC anomalies.

### D. Multi-Modal Sensor Fusion (Ground + Space)
The system bridges the gap between IoT and Remote Sensing by fusing **Dynamic Telemetry** (Ground Nodes) with **Static Satellite Baselines** (NDVI).
* *Innovation:* The AI assigns a "Vulnerability Weight" based on vegetation health. An area with **Low NDVI (Deforestation)** will trigger a Landslide Alert at a lower soil moisture threshold compared to a dense forest, significantly reducing false negatives.

### E. Predictive GIS & Temporal Forecasting (AI-Driven)
Moving beyond reactive alerts, the Ground Station functions as a **"Time Machine"** for disaster management:
* **Short-Term (Tactical):** Extrapolates Barometric Pressure trends ($\Delta P$) to predict storm surges **12 hours in advance**.
* **Long-Term (Strategic):** Utilizes **Linear Regression** on 30-day historical data to project disaster risk trends for the **next 7 days**, enabling authorities to allocate resources (food, medicine) *before* the crisis peaks.
* **Visualization:** Uses **Convex Hull algorithms** to dynamically delineate high-risk zones on the map.

---

## Pin Mapping (Hardware Wiring)

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

## Telemetry Data Format

### 1. Ground Node Packet (Uplink - LoRa)
Raw sensor data transmitted from Edge Nodes to the Satellite/Gateway.
* **Format:** `GN,[ID],[TYPE],[TEMP],[HUM],[PRES],[SOIL],[VIB],[LAT],[LON]`

| Field | Description | Example |
| :--- | :--- | :--- |
| **TYPE** | Detected Hazard Class | `NORMAL`, `FIRE_RISK`, `STORM_ALERT`, `LANDSLIDE` |
| **PRES** | Atmospheric Pressure (hPa) | `990.2` (Low pressure indicates storm approach) |
| **VIB** | Seismic Vibration ($G$) | `1.5` (Indicates ground instability) |
| **SOIL** | Soil Moisture (%) | `85.4` (Near saturation point) |

### 2. Satellite Heartbeat (Downlink)
System health status and piggybacked ground alerts broadcasted to the Ground Station.
* **Format:** `TM:[VOLT],[LAT],[LON],[ALT],[STATUS],[RELAY_COUNT]`

| Field | Description | Example |
| :--- | :--- | :--- |
| **VOLT** | Battery Voltage (V) | `4.1` (LiPo level) |
| **STATUS** | FSM State | `IDLE`, `LISTENING`, `IMG_TX` (Image Transmitting) |
| **RELAY** | Buffered Packets Count | `5` (Number of ground alerts in queue) |

### 3. AI Predictive Output (Processed API JSON)
Final intelligence generated by the **Random Forest Engine**, sent to the GIS Dashboard.

```json
{
  "node_id": 101,
  "timestamp": "2026-02-14T12:00:00Z",
  "risk_analysis": {
    "probability_score": 88.5,       // 0-100% Disaster Probability
    "hazard_type": "LANDSLIDE",      // Predicted Disaster Mode
    "severity_level": "CRITICAL"     // LOW, MODERATE, HIGH, CRITICAL
  },
  "sensor_fusion": {
    "soil_saturation": 85.0,         // Ground Truth
    "ndvi_baseline": 0.21,           // Satellite Data (Low = Deforestation)
    "pressure_trend": -3.5           // RoC: Dropping fast (-3.5hPa/hr)
  },
  "forecast": {
    "12h_prediction": "STORM_SURGE", // Short-term tactical forecast
    "7d_trend": "INCREASING"         // Long-term strategic trend
  }
}
```
---

## Dependencies & Libraries

To compile and run the project, ensure the following libraries and environments are installed.

### 1. Hardware (C++ / Arduino IDE)
Ensure you have the ESP32 Board Manager installed in your IDE.
* **LoRa:** `LoRa by Sandeep Mistry` (or `RadioLib`)
* **Sensors:** * `Adafruit BME280 Library`
  * `Adafruit MPU6050`
  * `TinyGPSPlus` (for NEO-M8N)
* **Core:** `FreeRTOS` (Built-in with ESP32 core)

### 2. Software (Python AI & Ground Station)
It is recommended to use a virtual environment.
* `python >= 3.9`
* `scikit-learn`
* `numpy`
* `pandas`
* `folium` / `cartopy` (for GIS Mapping)
* `pyserial` (for reading serial data from the LoRa Gateway)

---

## ⚙️ How to Run & Simulate (Instructions)

Follow these steps to reproduce the system simulations and run the AI model.

### Step 1: Hardware Firmware (ESP32)
1. Open the source code files for the microcontrollers using your preferred IDE (e.g., Arduino IDE).
2. For the Ground Node: Flash the file `gns_code.cpp` to the ESP32 board.
3. For the CubeSat 1U: Flash `satellite_codev5_master.cpp` to the ESP32-S3 and `satellite_codev5_slave.cpp` to the ESP32-CAM.

*(Note: If testing via Wokwi Simulator, please refer to the simulation links provided in the main report).*

### Step 2: AI Simulation & Dashboard (Software Demo)
To evaluate the software and Machine Learning logic without physical hardware, run the provided synthetic simulation script.

1. Ensure the dataset file `nodes_data.txt` is in the same directory as the Python script.
2. Install the required Python dependencies:
   ```bash
   download python (3.x -> x>8)
    cd path\to\your\folder
    python -m venv env
    env\Scripts\activate
    pip install folium numpy matplotlib scipy cartopy
