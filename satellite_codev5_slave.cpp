/*
* ======================================================================================
* PROJECT:     ATLAS_ASCEND 1.2 - DISASTER WARNING CUBESAT
* MODULE:      SLAVE NODE (Payload Unit)
* HARDWARE:    AI Thinker ESP32-CAM
* AUTHOR:      [VN7-ATLAS Team]
* DATE:        10 Feb 2026
* VERSION:     5 (Matched with Master v5)
* DESCRIPTION: Handles Image Capture, SD Buffering, and UART Streaming
* ======================================================================================
*/

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "soc/soc.h"           // Brownout control library
#include "soc/rtc_cntl_reg.h"  // Brownout control register

// ======================================================================================
// 1. SYSTEM CONFIGURATION
// ======================================================================================

// COMMUNICATION
// Match Master Node's baud rate. 460800 is used for high-speed image transfer.
#define SERIAL_BAUD_RATE    460800  

// STORAGE & BUFFERING
#define CHUNK_SIZE          200     // Fragmentation size (Bytes)
#define TEMP_FILE_PATH      "/temp.jpg"
#define FLASH_GPIO          4       // On-board Flash LED pin

// --- CAMERA PIN MAPPING (AI Thinker Standard) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ======================================================================================
// 2. SETUP ROUTINE
// ======================================================================================
void setup() {
    // A. Disable Brownout Detector 
    // CRITICAL: Prevents rebooting during high current spikes (Camera/SD usage)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 

    Serial.begin(SERIAL_BAUD_RATE);

    // B. Disable Flash LED
    // GPIO 4 is shared with SD Card. Must be pulled LOW to prevent blinding flash.
    pinMode(FLASH_GPIO, OUTPUT);
    digitalWrite(FLASH_GPIO, LOW);

    // C. Initialize Camera
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

    // RAM Configuration: Use PSRAM for High-Res, otherwise fallback to Low-Res
    if(psramFound()){
        config.frame_size = FRAMESIZE_UXGA; // 1600x1200
        config.jpeg_quality = 10;           // High Quality (Lower number = Better)
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("ERR:CAM_INIT");
        while(1); // Halt system on error
    }

    // D. Initialize SD Card
    // Use 1-bit Mode (true) to save pins and avoid Flash conflict
    if(!SD_MMC.begin("/sdcard", true)){
        Serial.println("ERR:SD_INIT");
        while(1);
    }

    // Ensure Flash is off again (Library might toggle it)
    pinMode(FLASH_GPIO, OUTPUT);
    digitalWrite(FLASH_GPIO, LOW);
}

// ======================================================================================
// 3. MAIN LOOP
// ======================================================================================
void loop() {
    // Listen for commands from Master (ESP32-S3)
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim(); // Clean trailing characters (\r\n)

        if (cmd == "CMD:CAPTURE") {
            handleCapture();
        }
        else if (cmd.startsWith("CMD:CHUNK:")) {
            // Extract Chunk ID from command string
            int chunkID = cmd.substring(10).toInt();
            handleSendChunk(chunkID);
        }
    }
}

// ======================================================================================
// 4. HELPER FUNCTIONS
// ======================================================================================

/*
* Capture photo and save to SD Card buffer
*/
void handleCapture() {
    // 1. Acquire Frame
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("ERR:CAPTURE_FAIL");
        return;
    }

    // 2. Open File for Writing
    // Object Reference: Abstracting the hardware layer for maintainability
    fs::FS &fs = SD_MMC; 
    File file = fs.open(TEMP_FILE_PATH, FILE_WRITE);
    
    if(!file){
        Serial.println("ERR:FILE_WRITE");
        esp_camera_fb_return(fb); // Release RAM before return
        return;
    }

    // 3. Write Buffer to Storage
    file.write(fb->buf, fb->len);
    file.close();

    // 4. Handshake: Report Image Size to Master
    long imageSize = fb->len;
    Serial.printf("OK:SIZE:%ld\n", imageSize);

    // 5. Release Camera RAM (Critical step to prevent memory leak)
    esp_camera_fb_return(fb); 
}

/*
* Read a specific file segment (Chunk) and stream via UART
* @param chunkID: Sequence number requested by Master
*/
void handleSendChunk(int chunkID) {
    fs::FS &fs = SD_MMC;
    File file = fs.open(TEMP_FILE_PATH, FILE_READ);

    if(!file) {
        // If file open fails, do nothing. 
        // Master has a 10s timeout mechanism to handle this failure.
        return; 
    }

    // Random Access: Calculate offset mapping
    size_t seekPos = chunkID * CHUNK_SIZE; 

    // Boundary Safety Check
    if (seekPos < file.size()) { 
        // Move file pointer to the specific chunk location
        if (file.seek(seekPos)) {
            uint8_t buffer[CHUNK_SIZE]; 
            
            // Read up to CHUNK_SIZE bytes.
            // If it's the last chunk, 'bytesRead' will automatically be smaller.
            int bytesRead = file.read(buffer, CHUNK_SIZE);
            
            // Send Raw Binary Data
            if (bytesRead > 0) {
                Serial.write(buffer, bytesRead);
            }
        }
    }

    // Resource Management: 
    // Close file immediately to prevent 'Too Many Open Files' error
    file.close(); 
}