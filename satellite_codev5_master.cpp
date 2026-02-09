/*
* ======================================================================================
* PROJECT:     ATLAS_ASCEND 1.2 - DISASTER WARNING CUBESAT
* MODULE:      MASTER NODE (OBC - On Board Computer)
* HARDWARE:    ESP32-S3 DevKit V1
* AUTHOR:      [VN7-ATLAS Team]
* DATE:        10 Feb 2026
* VERSION:     5
* DESCRIPTION: Master-Slave Architecture using UART Flow Control & LoRa Telemetry
* ======================================================================================
*/

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_INA219.h>

// ======================================================================================
// 1. SYSTEM CONFIGURATION
// ======================================================================================

// --- TIMING SETTINGS ---
#define TELEMETRY_INTERVAL_MS   5000    // Heartbeat telemetry interval (5s)
#define CAMERA_INTERVAL_MS      60000   // Image capture cycle (60s)
#define CAMERA_TIMEOUT_MS       10000   // Watchdog timeout for camera response
#define CHUNK_SIZE              200     // Payload fragmentation size (Bytes)

// --- PIN DEFINITIONS ---
// I2C Bus (Sensors)
#define I2C_SDA         42
#define I2C_SCL         41

// LoRa SX1278 (SPI Bus)
#define LORA_MISO       13
#define LORA_MOSI       11
#define LORA_SCK        12
#define LORA_CS         10
#define LORA_RST        5
#define LORA_IRQ        6

// GPS NEO-M8N (UART1)
#define GPS_RX          18 
#define GPS_TX          17 

// Slave Camera Comm (UART2)
// High-speed UART is used for asynchronous image streaming
#define CAM_RX          16 
#define CAM_TX          15 

// ======================================================================================
// 2. OBJECTS & GLOBAL VARIABLES
// ======================================================================================

// Sensor Objects
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_INA219 ina219;

// Communication Objects
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
HardwareSerial CamSerial(2);

// Data Structure for Telemetry
struct SatelliteTelemetry {
    float bat_voltage;
    double lat, lon, alt;
    char system_status[20]; // Status flags: "OK", "ERR_CAM", "ERR_LORA"
} satData;

// Thread-safe Shared Variables
String relayQueue = "";
bool hasRelayData = false;

// ======================================================================================
// 3. SETUP ROUTINE
// ======================================================================================
void setup() {
    // 1. Initialize Debug Serial
    Serial.begin(115200);
    Serial.println("\n>>> MASTER NODE BOOTING (ATLAS_ASCEND v5.0)...");

    // 2. Initialize Peripherals Serial
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    
    // Configure High-Speed UART for Camera (460800 baud)
    // This reduces transfer latency significantly compared to standard 115200
    CamSerial.begin(460800, SERIAL_8N1, CAM_RX, CAM_TX); 

    // 3. Initialize I2C & Sensors
    Wire.begin(I2C_SDA, I2C_SCL);
    bool sens_ok = true;
    
    // Sensor Health Check
    if (!bme.begin(0x76)) { Serial.println("[ERR] BME280 Not Found"); sens_ok = false; }
    if (!mpu.begin())     { Serial.println("[ERR] MPU6050 Not Found"); sens_ok = false; }
    if (!ina219.begin())  { Serial.println("[ERR] INA219 Not Found"); sens_ok = false; }
    
    // Set initial system status
    strcpy(satData.system_status, sens_ok ? "OK" : "ERR_SENS");

    // 4. Initialize LoRa Radio
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
    if (!LoRa.begin(433E6)) {
        Serial.println("[FATAL] LoRa Init Failed!");
        strcpy(satData.system_status, "ERR_LORA");
        // System continues to run to log errors, but telemetry won't work
    } else {
        Serial.println("[INFO] LoRa Initialized at 433MHz");
    }

    // 5. Initialize FreeRTOS Tasks
    // Core 0: Critical System Tasks (GPS, Sensors, LoRa) - High Priority
    xTaskCreatePinnedToCore(TaskSystem, "System", 10000, NULL, 1, NULL, 0);
    
    // Core 1: Heavy Computation Tasks (Image Processing) - Lower Priority
    xTaskCreatePinnedToCore(TaskVisionMaster, "VisionMaster", 10000, NULL, 1, NULL, 1); 
}

void loop() { 
    vTaskDelete(NULL); // Standard Arduino loop is disabled in RTOS
}

// ======================================================================================
// 4. TASK: SYSTEM MANAGEMENT (Core 0)
// ======================================================================================
/* * Maintains satellite health, reads continuous data streams (GPS), 
* and handles ground communication.
*/
void TaskSystem(void * parameter) {
    unsigned long lastTelemetryTime = 0;
    
    for(;;) {
        // A. Process GPS Stream & Sensors
        while (GPSSerial.available()) gps.encode(GPSSerial.read());
        
        if (gps.location.isValid()) {
            satData.lat = gps.location.lat();
            satData.lon = gps.location.lng();
            satData.alt = gps.altitude.meters();
        }
        satData.bat_voltage = ina219.getBusVoltage_V();

        // B. Handle LoRa Relay (Uplink from Ground Nodes)
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            String msg = "";
            while (LoRa.available()) msg += (char)LoRa.read();
            relayQueue = msg;
            hasRelayData = true;
            Serial.println("[RELAY] Received Uplink: " + msg);
        }

        // C. Transmit Telemetry (Downlink)
        if (millis() - lastTelemetryTime > TELEMETRY_INTERVAL_MS) {
            // Format: "TM:VOLT,LAT,LON,ALT,STATUS"
            String packet = "";
            packet.reserve(128); // Optimize memory allocation
            
            packet = "TM:" + String(satData.bat_voltage, 2) + "," + 
                    String(satData.lat, 6) + "," + 
                    String(satData.lon, 6) + "," + 
                    String(satData.alt, 2) + "," + 
                    String(satData.system_status);
            
            // Piggyback Relay Data if queue is not empty
            if(hasRelayData) { 
                packet += "; REL_GNS:" + relayQueue; 
                hasRelayData = false; 
                relayQueue = ""; 
            }
            
            LoRa.beginPacket(); 
            LoRa.print(packet); 
            LoRa.endPacket();
            
            Serial.println("[TX] Telemetry: " + packet);
            lastTelemetryTime = millis();
        }
        
        // Anti-starvation delay
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ======================================================================================
// 5. TASK: VISION MASTER CONTROL (Core 1)
// ======================================================================================
/* * Controls the Master-Slave protocol for image acquisition.
* Uses 'Request-Response' handshake and 'Chunking' flow control.
*/
void TaskVisionMaster(void * parameter) {
    uint8_t buffer[250]; // Buffer slightly larger than chunk size for safety
    
    for(;;) {
        // Wait for next cycle
        vTaskDelay(CAMERA_INTERVAL_MS / portTICK_PERIOD_MS);
        Serial.println("\n[VISION] Initiating Capture Sequence...");

        // --- PHASE 1: HANDSHAKE ---
        
        // Clear RX Buffer to prevent reading stale bytes
        while(CamSerial.available() > 0) { 
            CamSerial.read();
        }
        
        CamSerial.println("CMD:CAPTURE");

        // Wait for Slave Response with Timeout Protection
        unsigned long startWait = millis();
        bool isCaptured = false;
        long imageSize = 0;
        int totalChunks = 0;
        
        while(millis() - startWait < CAMERA_TIMEOUT_MS) {
            if(CamSerial.available()) {
                String resp = CamSerial.readStringUntil('\n');
                resp.trim(); 
                
                // Protocol Check: "OK:SIZE:XXXX"
                if(resp.startsWith("OK:SIZE:")) {
                    imageSize = resp.substring(8).toInt();
                    // Ceiling Division to calculate total chunks needed
                    totalChunks = (imageSize + CHUNK_SIZE - 1) / CHUNK_SIZE; 
                    isCaptured = true;
                    break;
                }
            }
            vTaskDelay(10 / portTICK_PERIOD_MS); 
        }

        // Error Handling: Timeout
        if(!isCaptured) {
            Serial.println("[VISION] Error: Camera Response Timeout!");
            strcpy(satData.system_status, "ERR_CAM_TO");
            continue; // Skip this cycle
        }

        Serial.printf("[VISION] Target Acquired. Size: %ld bytes. Chunks: %d\n", imageSize, totalChunks);
        
        // Notify Ground Station: Start of Image Transmission
        LoRa.beginPacket(); 
        LoRa.print("IMG_START,SIZE:" + String(imageSize)); 
        LoRa.endPacket();
        
        vTaskDelay(500 / portTICK_PERIOD_MS); 

        // --- PHASE 2: DATA STREAMING (Flow Control) ---
        for(int i=0; i < totalChunks; i++) {
            
            // a. Request specific chunk ID
            CamSerial.printf("CMD:CHUNK:%d\n", i);
            
            // b. Calculate expected bytes for this chunk
            // Explicit logic to handle the last chunk correctly
            int expectedBytes = CHUNK_SIZE;
            if (i == totalChunks - 1) {
                int remainder = imageSize % CHUNK_SIZE;
                if (remainder != 0) expectedBytes = remainder;
            }
            
            // c. Wait for Raw Data from Slave
            unsigned long chunkWait = millis();
            int bytesRead = 0;
            
            while(millis() - chunkWait < 2000) { // 2s Timeout per chunk
                if(CamSerial.available()) {
                    if (bytesRead < sizeof(buffer)) {
                        buffer[bytesRead] = CamSerial.read();
                        bytesRead++;
                    } else {
                        CamSerial.read(); // Discard overflow bytes
                    }

                    if (bytesRead == expectedBytes) {
                        break; // Data complete
                    }
                }
            }

            // d. Forward via LoRa (Packet Forwarding)
            if (bytesRead > 0) {
                LoRa.beginPacket();
                LoRa.write((uint8_t)(i >> 8));   // Header: ID High
                LoRa.write((uint8_t)(i & 0xFF)); // Header: ID Low
                LoRa.write(buffer, bytesRead);   // Payload
                LoRa.endPacket(); // Blocking call until TX done
                
                // Log progress every 10 chunks
                if(i % 10 == 0) Serial.printf(">> Forwarded Chunk %d/%d\n", i, totalChunks);
            } else {
                Serial.printf("[VISION] Error: Chunk %d missing or empty\n", i);
            }
            
            // Duty Cycle Delay (Critical for LoRa module thermal management)
            vTaskDelay(100 / portTICK_PERIOD_MS); 
        }
        
        // Notify Ground Station: End of Transmission
        LoRa.beginPacket(); 
        LoRa.print("IMG_END"); 
        LoRa.endPacket();
        
        Serial.println("[VISION] Transmission Complete!");
        strcpy(satData.system_status, "OK");
    }
}