/*
* PROJECT: ATLAS_ASCEND 1.2 - DISASTER WARNING SATELLITE
* VERSION: 4.0 (Final Release - SD Buffer & Image Chunking)
* HARDWARE: ESP32-S3 + LoRa + GPS + Sensors + OV2640 + MicroSD
* AUTHOR: [VN7-ATLAS]
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
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"

// ==========================================
// 1. CẤU HÌNH CHÂN (PIN MAPPING)
// ==========================================

// I2C Bus (Sensors)
#define I2C_SDA 42
#define I2C_SCL 41

// SPI Bus (LoRa + SD Card dùng chung)
#define SPI_MISO 13
#define SPI_MOSI 11
#define SPI_SCK  12

// LoRa SX1278 (Ra-02)
#define LORA_CS   10
#define LORA_RST  5
#define LORA_IRQ  6

// MicroSD Card Module
#define SD_CS     21 // Chân chọn chip cho thẻ nhớ

// GPS NEO-M8N (UART1)
#define GPS_RX_PIN 18 // Nối TX GPS
#define GPS_TX_PIN 17 // Nối RX GPS

// Camera OV2640 (ESP32-S3 DevKit Pinout)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// ==========================================
// 2. KHAI BÁO ĐỐI TƯỢNG & BIẾN TOÀN CỤC
// ==========================================
Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_INA219 ina219;
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

// Cấu trúc gói tin Telemetry (Sức khỏe vệ tinh)
struct SatelliteTelemetry {
float bat_voltage;
float temp_internal;
double lat;
double lon;
double alt;
char system_status[20]; // Trạng thái hệ thống (VD: "OK", "ERR_CAM")
} satData;

// Biến chia sẻ giữa các luồng (Thread-safe logic đơn giản)
String relayQueue = "";
bool hasRelayData = false;

// Task Handles
TaskHandle_t TaskSystemHandle;
TaskHandle_t TaskVisionHandle;

// ==========================================
// 3. HÀM KHỞI TẠO (SETUP)
// ==========================================
void setup() {
Serial.begin(115200);
GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
Wire.begin(I2C_SDA, I2C_SCL);

Serial.println("\n>>> ATLAS_ASCEND SATELLITE SYSTEM V4.0 BOOTING...");

// A. Khởi tạo Cảm biến (Health Check)
bool sens_ok = true;
if (!bme.begin(0x76)) sens_ok = false;
if (!mpu.begin()) sens_ok = false;
if (!mag.begin()) sens_ok = false;
if (!ina219.begin()) sens_ok = false;

// Trạng thái ban đầu
if(sens_ok) strcpy(satData.system_status, "OK");
else strcpy(satData.system_status, "ERR_SENS");

// B. Khởi tạo SPI & LoRa
SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
if (!LoRa.begin(433E6)) {
    Serial.println("FAIL: LoRa Radio");
    strcpy(satData.system_status, "ERR_LORA");
    while(1); // Chết lâm sàng nếu không có LoRa
}

// C. Khởi tạo Thẻ nhớ SD (Bộ đệm ảnh)
if(!SD.begin(SD_CS)){
    Serial.println("FAIL: SD Card Mount");
    strcpy(satData.system_status, "ERR_SD");
} else {
    Serial.println("SUCCESS: SD Card Mounted");
}

// D. Khởi tạo Camera OV2640
camera_config_t config;
config.ledc_channel = LEDC_CHANNEL_0;
config.ledc_timer = LEDC_TIMER_0;
config.pin_d0 = Y2_GPIO_NUM;
config.pin_d1 = Y3_GPIO_NUM;
config.pin_d2 = Y4_GPIO_NUM;
config.pin_d3 = Y5_GPIO_NUM;
config.pin_d4 = Y6_GPIO_NUM;
config.pin_d5 = Y7_GPIO_NUM;
config.pin_d6 = Y8_GPIO_NUM;
config.pin_d7 = Y9_GPIO_NUM;
config.pin_xclk = XCLK_GPIO_NUM;
config.pin_pclk = PCLK_GPIO_NUM;
config.pin_vsync = VSYNC_GPIO_NUM;
config.pin_href = HREF_GPIO_NUM;
config.pin_sscb_sda = SIOD_GPIO_NUM;
config.pin_sscb_scl = SIOC_GPIO_NUM;
config.pin_pwdn = PWDN_GPIO_NUM;
config.pin_reset = RESET_GPIO_NUM;
config.xclk_freq_hz = 20000000;
config.pixel_format = PIXFORMAT_JPEG;

if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // 1600x1200
    config.jpeg_quality = 10;
    config.fb_count = 2;
} else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
}

if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("FAIL: Camera Init");
    strcpy(satData.system_status, "ERR_CAM");
}

// E. Đa nhiệm (Multitasking)
// Core 0: Hệ thống, Cảm biến, LoRa (Nhẹ, Ổn định)
xTaskCreatePinnedToCore(TaskSystem, "System", 10000, NULL, 1, &TaskSystemHandle, 0);
// Core 1: Xử lý ảnh, Thẻ nhớ (Nặng)
xTaskCreatePinnedToCore(TaskVision, "Vision", 20000, NULL, 1, &TaskVisionHandle, 1);
}

void loop() { vTaskDelete(NULL); }

// ==========================================
// 4. TASK SYSTEM: QUẢN LÝ SỰ SỐNG & TRUYỀN TIN
// ==========================================
void TaskSystem(void * parameter) {
unsigned long lastTelemetry = 0;
for(;;) {
    // 1. ĐỌC DỮ LIỆU CẢM BIẾN
    while (GPSSerial.available() > 0) gps.encode(GPSSerial.read());
    if (gps.location.isValid()) {
    satData.lat = gps.location.lat();
    satData.lon = gps.location.lng();
    satData.alt = gps.altitude.meters();
    }
    
    satData.bat_voltage = ina219.getBusVoltage_V();
    satData.temp_internal = bme.readTemperature();

    // 2. NHẬN TIN NHẮN TỪ MẶT ĐẤT (RELAY)
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
    String msg = "";
    while (LoRa.available()) msg += (char)LoRa.read();
    Serial.println("RELAY RX: " + msg);
    relayQueue = msg;
    hasRelayData = true;
    }

    // 3. GỬI TELEMETRY (Mỗi 5 giây)
    if (millis() - lastTelemetry > 5000) {
    // Định dạng gói tin: "ATLAS_TM,VOLT,LAT,LON,STATUS,RELAY_DATA"
    String packet = "ATLAS_TM,";
    packet += String(satData.bat_voltage) + ",";
    packet += String(satData.lat, 6) + ",";
    packet += String(satData.lon, 6) + ",";
    packet += String(satData.system_status); // Quan trọng: Gửi trạng thái lỗi về

    if (hasRelayData) {
        packet += ",RELAY:" + relayQueue;
        hasRelayData = false;
        relayQueue = "";
    }

    LoRa.beginPacket();
    LoRa.print(packet);
    LoRa.endPacket();
    Serial.println("TELEMETRY TX: " + packet);
    
    lastTelemetry = millis();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
}

// ==========================================
// 5. TASK VISION: CHỤP ẢNH & CẮT GÓI TIN (CHUNKING)
// ==========================================
void TaskVision(void * parameter) {
for(;;) {
    // Chu kỳ chụp ảnh: 60 giây
    vTaskDelay(60000 / portTICK_PERIOD_MS);
    Serial.println("[VISION] Starting Capture Sequence...");

    // BƯỚC 1: CHỤP ẢNH
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
    Serial.println("Error: Camera Capture Failed");
    strcpy(satData.system_status, "ERR_CAM"); // BÁO LỖI NGAY LẬP TỨC
    continue;
    } else {
    // Nếu trước đó bị lỗi, giờ chụp được thì xóa lỗi
    if (strcmp(satData.system_status, "ERR_CAM") == 0) strcpy(satData.system_status, "OK");
    }

    // BƯỚC 2: LƯU VÀO THẺ NHỚ (BUFFER)
    // Xóa file cũ để tiết kiệm chỗ
    if(SD.exists("/temp.jpg")) SD.remove("/temp.jpg");

    File file = SD.open("/temp.jpg", FILE_WRITE);
    if (!file) {
    Serial.println("Error: Failed to open SD card");
    strcpy(satData.system_status, "ERR_SD"); // BÁO LỖI SD
    esp_camera_fb_return(fb);
    continue;
    }
    
    file.write(fb->buf, fb->len);
    file.close();
    size_t fileSize = fb->len;
    esp_camera_fb_return(fb); // Giải phóng RAM Camera

    Serial.printf("[VISION] Saved to SD: %d bytes. Starting Transmission...\n", fileSize);

    // BƯỚC 3: CẮT NHỎ VÀ GỬI (FRAGMENTATION)
    // Gửi thông báo bắt đầu gửi ảnh
    LoRa.beginPacket();
    LoRa.print("IMG_START,SIZE:" + String(fileSize));
    LoRa.endPacket();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    File readFile = SD.open("/temp.jpg", FILE_READ);
    uint8_t buffer[200]; // Kích thước mỗi gói con (Chunk)
    uint16_t packetID = 0;
    
    while (readFile.available()) {
    int bytesRead = readFile.read(buffer, 200);
    
    // Cấu trúc gói: [ID High][ID Low][Data...]
    LoRa.beginPacket();
    LoRa.write((uint8_t)(packetID >> 8));
    LoRa.write((uint8_t)(packetID & 0xFF));
    LoRa.write(buffer, bytesRead);
    LoRa.endPacket();
    
    packetID++;
    // Delay nhỏ để tránh nghẽn mạng LoRa
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // In ra debug (chỉ để kiểm tra)
    if (packetID % 10 == 0) Serial.printf("Sent Chunk %d\n", packetID);
    }
    readFile.close();

    // Gửi thông báo kết thúc
    LoRa.beginPacket();
    LoRa.print("IMG_END,PACKETS:" + String(packetID));
    LoRa.endPacket();
    
    Serial.println("[VISION] Transmission Complete!");
}
}
