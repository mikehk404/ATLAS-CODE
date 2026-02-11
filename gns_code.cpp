/**
 * ===================================================================================
 * PROJECT ATLAS_ASCEND 1.2 - MULTI-HAZARD GROUND SENSOR NODE
 * ===================================================================================
 * author      [Team VN7-ATLAS]
 * brief       Autonomous IoT Firmware for Environmental Monitoring & Early Warning.
 * Implements "Deep Sleep" architecture and "Rate-of-Change" algorithms.
 * * HARDWARE CONFIGURATION ("The Fab Four" Sensor Stack):
 * 1. MCU      ESP32-WROOM-32 (Dual Core, 240MHz)
 * 2. SOIL     Capacitive Soil Moisture Sensor v1.2 (Analog)
 * 3. IMU      MPU-6050 (6-DOF Accelerometer & Gyroscope)
 * 4. CLIMATE  BME280 (Temperature, Humidity, Barometric Pressure)
 * 5. GNSS     NEO-M8N (GPS/GLONASS)
 * 6. COMMS    LoRa SX1278 (433MHz)
 * * SCIENTIFIC LOGIC & ALGORITHMS
 * - Power Mgmt:  Deep Sleep cycle (Wake -> Measure -> Analyze -> Tx -> Sleep).
 * - Anti-Noise:  Hardware Low-Pass Filters (MPU6050) + Software EMA Averaging.
 * - Detection:   
 * a) FIRE    Based on Nesterov Index proxy (High Temp + Low Humidity).
 * b) STORM   Barometric Tendency (Pressure drop rate > Threshold).
 * c) SLIDE   Coulomb's Law (Soil Saturation reduces shear strength) + Seismic Vib.
 * d) FLOOD   Infiltration Rate Analysis (Moisture spike > Infiltration capacity).
 * * ===================================================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>

// --- HARDWARE PIN MAPPING ---
#define NODE_ID       1         
#define PIN_SOIL      34        // ADC1_CH6 (Input Only)
// LoRa SX1278 (SPI)
#define LORA_SS       5
#define LORA_RST      14
#define LORA_DIO0     2
// GPS NEO-M8N (UART2)
#define GPS_RX_PIN    16 
#define GPS_TX_PIN    17 
// I2C Bus (Sensors)
#define I2C_SDA         42
#define I2C_SCL         41

// --- SCIENTIFIC THRESHOLDS (CALIBRATED) ---

// 1. FIRE RISK (Thermo-Hygrometric Index)
// Ignition probability increases significantly when Temp > 38°C and RH < 35%.
const float TH_FIRE_TEMP = 38.0; 
const float TH_FIRE_HUM  = 35.0; 

// 2. LANDSLIDE RISK (Geotechnical Stability)
// Soil Saturation > 85% significantly reduces the friction angle (phi) of soil.
const float TH_SLIDE_SOIL = 85.0; 
// Seismic Vibration > 1.2G indicates slope failure initiation or earthquake.
const float TH_SLIDE_VIB  = 1.2;  

// 3. STORM / CYCLONE (Barometric Tendency)
// A pressure drop > 3hPa within a short window indicates a rapid low-pressure system approach.
const float TH_STORM_DROP = 3.0; 

// 4. FLASH FLOOD (Infiltration Rate)
// Soil moisture increase > 10% per cycle implies rainfall intensity > soil infiltration capacity.
const float TH_FLOOD_ROC  = 10.0;

// --- POWER MANAGEMENT ---
// Deep Sleep allows operation for months on Li-ion batteries.
#define SLEEP_NORMAL   300 * 1000000ULL // 5 Minutes (Standard Monitoring)
#define SLEEP_URGENT   60 * 1000000ULL  // 1 Minute (High Alert Mode)

// --- RTC MEMORY (PERSISTENT DATA STORAGE) ---
// Variables stored here retain values during Deep Sleep to calculate "Rate-of-Change".
RTC_DATA_ATTR float prevPres = 0; // Pressure history (hPa)
RTC_DATA_ATTR float prevSoil = 0; // Soil Moisture history (%)
RTC_DATA_ATTR int   bootCount = 0;

// --- OBJECT INSTANTIATION ---
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2); 

void setup() {
    // 1. SYSTEM INITIALIZATION
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL); 
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    bootCount++;
    // Stabilization delay for sensors after power-up
    delay(100); 

    // 2. SENSOR INITIALIZATION & HEALTH CHECK
    bool sysHealth = true;

    // Init BME280 (I2C 0x76 or 0x77)
    if (!bme.begin(0x76)) sysHealth = false; 

    // Init MPU6050 with Hardware Filter
    if (!mpu.begin()) sysHealth = false;
    else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        // Low-Pass Filter (21Hz) removes high-frequency noise from wind/machinery
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    // Init LoRa Transceiver
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(433E6)) {
        Serial.println("FATAL: LoRa Module Failed!");
        esp_deep_sleep_start(); // Abort to save power
    }
    LoRa.setSyncWord(0xF3); // Private Network Key

    // =========================================================================
    // STEP 3: DATA ACQUISITION & SIGNAL PROCESSING
    // =========================================================================

    // 3.1 CLIMATE DATA (BME280)
    float temp = bme.readTemperature();
    float hum  = bme.readHumidity();
    float pres = bme.readPressure() / 100.0F; // Convert Pa to hPa

    // 3.2 KINEMATIC DATA (MPU6050)
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    // Calculate Magnitude of Acceleration Vector: |A| = sqrt(x^2 + y^2 + z^2)
    float totalAcc = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
    // Subtract Gravity (9.81 m/s^2) to isolate dynamic vibration
    float vib = abs(totalAcc - 9.81); 

    // 3.3 SOIL DATA (Capacitive Sensor) with Software Averaging
    long soilRaw = 0;
    for(int i=0; i<10; i++) { soilRaw += analogRead(PIN_SOIL); delay(5); }
    // Mapping ADC (0-4095) to Percentage (0-100%). Needs field calibration.
    // 4095 = Dry Air, 1500 = Water Submerged
    float soil = map(soilRaw/10, 4095, 1500, 0, 100); 
    soil = constrain(soil, 0, 100);

    // 3.4 GEOLOCATION (NEO-M8N) - "Best Effort" acquisition (1 sec timeout)
    String lat = "0", lon = "0";
    unsigned long startGPS = millis();
    while (millis() - startGPS < 1000) {
        while (GPS_Serial.available()) gps.encode(GPS_Serial.read());
    }
    if (gps.location.isValid()) {
        lat = String(gps.location.lat(), 6);
        lon = String(gps.location.lng(), 6);
    }

    // =========================================================================
    // STEP 4: MULTI-HAZARD CLASSIFICATION ALGORITHM
    // =========================================================================

    String alertType = "NORMAL";
    bool isUrgent = false;

    // --- LOGIC A: FIRE RISK ANALYSIS ---
    // High Temperature + Low Humidity = Fuel Drying -> Ignition Risk
    if (temp > TH_FIRE_TEMP && hum < TH_FIRE_HUM) {
        alertType = "FIRE_RISK";
        isUrgent = true;
    }

    // --- LOGIC B: STORM / CYCLONE PREDICTION ---
    // Barometric Tendency: Rapid pressure drop indicates approaching low-pressure system.
    // Check performed only after the first boot cycle.
    if (bootCount > 1 && (prevPres - pres) > TH_STORM_DROP) {
        alertType = "STORM_ALERT";
        isUrgent = true;
    }

    // --- LOGIC C: LANDSLIDE DETECTION ---
    // Soil Saturation (pore water pressure) OR Seismic Activity
    if (soil > TH_SLIDE_SOIL || vib > TH_SLIDE_VIB) {
        alertType = "LANDSLIDE";
        isUrgent = true;
    }

    // --- LOGIC D: FLASH FLOOD (Rate-of-Change) ---
    // Detecting rapid infiltration rate (Soil Moisture Spike)
    // Example: 10% increase in 5 mins is physically impossible by normal seepage.
    if (bootCount > 1 && (soil - prevSoil) > TH_FLOOD_ROC) {
        alertType = "FLASH_FLOOD";
        isUrgent = true;
    }

    // =========================================================================
    // STEP 5: PACKETIZATION & TRANSMISSION
    // =========================================================================

    // CSV Format: GN,ID,TYPE,TEMP,HUM,PRES,SOIL,VIB,LAT,LON
    // Optimized for parsing by Python AI Server
    String packet = "GN," + String(NODE_ID) + "," + alertType + "," +
                    String(temp, 1) + "," + String(hum, 1) + "," + String(pres, 1) + "," +
                    String(soil, 1) + "," + String(vib, 2) + "," +
                    lat + "," + lon;

    Serial.println("Transmitting: " + packet);

    LoRa.beginPacket();
    LoRa.print(packet);
    LoRa.endPacket();
    delay(100); // Allow Tx buffer to clear

    // =========================================================================
    // STEP 6: DEEP SLEEP & STATE RETENTION
    // =========================================================================

    // Save current states to RTC Memory for next cycle's comparison
    prevPres = pres;
    prevSoil = soil;

    // Adaptive Sleep Interval: 
    // - High Risk -> 1 Minute (Continuous Monitoring)
    // - Normal    -> 5 Minutes (Power Saving)
    uint64_t sleepTime = isUrgent ? SLEEP_URGENT : SLEEP_NORMAL;

    Serial.printf("Entering Deep Sleep for %d seconds...\n", (int)(sleepTime/1000000));

    LoRa.end(); // Power down LoRa
    esp_sleep_enable_timer_wakeup(sleepTime);
    esp_deep_sleep_start();
}

void loop() {
    // Unreachable code due to Deep Sleep architecture.
}