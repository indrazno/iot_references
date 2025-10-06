// =============================================================================
// MILES DEVICE - LoRaWAN + GPS + HTTP Tracker for LILYGO T3-S3
// =============================================================================
// Author: Indrazno Siradjuddin
// Platform: ESP32-S3 (LILYGO T3-S3)
// Version: 3.1 (Production Stable)
// License: MIT
//
// FEATURES:
//   ✅ LoRaWAN (US915, LMIC 1.0.3) to The Things Network (TTN)
//   ✅ SIM808 GPS acquisition with sflt16 compact encoding
//   ✅ Periodic HTTP POST to Simpera server via GPRS (Indosat APN)
//   ✅ OLED display (SSD1306) with real-time status cycling
//   ✅ SD card logging with auto-incremented CSV files
//   ✅ FreeRTOS multi-tasking (8KB stack, core-pinned for performance)
//   ✅ Thread-safe shared data using mutexes (100ms timeout)
//   ✅ Built-in LED for visual status feedback
//   ✅ Serial monitor (compile-time disable for field safety)
//
// CRITICAL DESIGN PRINCIPLES:
//   🔒 SIM808 is NOT thread-safe. All modem operations (GPS + HTTP)
//      are serialized in a SINGLE task: updateGsmTask().
//      This prevents AT command corruption and ensures stability.
//   🛡️ All shared data (g_dataPayload, g_logEvents, g_highPrecisionGps) 
//      protected by mutexes with timeout to avoid deadlocks.
//   ⚡ Baud rate fixed to 115200 to match SIM808 configuration.
//   📉 Graceful degradation: skip operations if resources are busy.
//   🕒 Watchdog-safe: all tasks yield regularly; no blocking calls.
//
// DEPLOYMENT NOTES:
//   - Set ENABLE_SERIAL_MON = 0 before field deployment
//   - Ensure SIM808 antenna is attached
//   - Verify SIM has active Indosat data package
//   - SD logs stored in /logs/t3s3-log-*.csv
// =============================================================================

#define ENABLE_SERIAL_MON 0  // Set to 1 ONLY during development/debugging

// =============================================================================
// INCLUDES
// =============================================================================
#include <Arduino.h>
#include <cstring>         // For strncpy, memcpy
#include <SPI.h>           // LoRa and SD card
#include <FS.h>            // File system abstraction
#include <SD.h>            // ESP32-optimized SD library
#include <Wire.h>          // I2C for OLED
#include "esp_task_wdt.h"  // Task watchdog (monitoring only)
#include <lmic.h>          // MCCI LoRaWAN stack
#include <hal/hal.h>       // LMIC hardware abstraction
#include <SSD1306Wire.h>   // OLED driver

// =============================================================================
// HTTP / GPRS CONFIGURATION
// MUST be defined BEFORE TinyGsmClient.h to select modem type
// =============================================================================
#define TINY_GSM_MODEM_SIM808    // Enable SIM808-specific features
#define TINY_GSM_RX_BUFFER 1024  // Increase RX buffer for GPRS responses

#define MODEM_TX 16       // ESP32 GPIO → SIM808 RX
#define MODEM_RX 15       // SIM808 TX → ESP32 GPIO
#define SerialAT Serial1  // Use built-in UART1

// Network credentials for Indosat
#define APN "indosatgprs"
#define GPRS_USER "indosat"
#define GPRS_PASS "indosat"
#define SIM_PIN ""  // Leave empty if no PIN

// Simpera server configuration
#define SERVER "new-simpera.isar.web.id"
#define RESOURCE "/end_point_new_simpera_table.php"
#define SERVER_PORT 80
#define API_KEY "tPmAT5Ab3j7F9"

// =============================================================================
// INCLUDE GSM LIBRARY AFTER CONFIGURATION
// =============================================================================
#include <TinyGsmClient.h>

// FreeRTOS headers
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// Mutex timeout: 100ms to prevent deadlocks and watchdog resets
#define MUTEX_TIMEOUT pdMS_TO_TICKS(100)

// =============================================================================
// LORAWAN CONFIGURATION (US915, Class A, 1.0.3)
// =============================================================================
#define CFG_us915 1
#define CFG_sx1276_radio 1
#define LMIC_LORAWAN_SPEC_VERSION LMIC_LORAWAN_SPEC_VERSION_1_0_3

// OTAA credentials (replace with your TTN device credentials)
static const u1_t PROGMEM APPEUI[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF };
static const u1_t PROGMEM DEVEUI[8] = { 0xF9, 0x4B, 0x00, 0xD8, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM APPKEY[16] = { 0x2B, 0xD9, 0x08, 0x24, 0x47, 0x8D, 0x48, 0xE7,
                                         0x46, 0x8E, 0x84, 0x3E, 0x2F, 0x5D, 0x01, 0xFA };

// LMIC credential injection callbacks
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

// =============================================================================
// HARDWARE PIN DEFINITIONS (LILYGO T3-S3)
// =============================================================================
// LoRa (SX1276)
#define LORA_SCK 5
#define LORA_MISO 3
#define LORA_MOSI 6
#define LORA_CS 7
#define LORA_RST 8
#define LORA_DIO0 9
#define LORA_DIO1 33
#define LORA_DIO2 34

// SD Card (SPI)
#define SDCARD_MISO 2
#define SDCARD_MOSI 11
#define SDCARD_SCLK 14
#define SDCARD_CS 13

// OLED (I2C)
#define OLED_SDA 18
#define OLED_SCL 17
#define OLED_RST 0  // Not used (hardware reset)

// Built-in LED
#define LED_BUILTIN 37

// =============================================================================
// GLOBAL CONSTANTS
// =============================================================================
const char* DEVICE_ID = "t3s3";              // Prefix for log files
const unsigned TX_INTERVAL = 10;             // LoRa TX interval (seconds)
const long HTTP_REPORT_INTERVAL_MS = 10000;  // HTTP POST interval (10s)

// =============================================================================
// DATA STRUCTURES (PACKED FOR LORAWAN EFFICIENCY)
// #pragma pack(1) eliminates padding for compact payload
// =============================================================================
#pragma pack(1)

/// @brief GPS coordinates encoded as sflt16 (semi-float 16-bit)
/// Range: [-1.0, 1.0) → maps to [-90°,90°] lat, [-180°,180°] lon
struct GpsPayload {
  uint16_t latitude;
  uint16_t longitude;
};

/// @brief Main device payload (15 bytes total)
/// Well within US915 DR3 limit of 51 bytes
struct DataPayload {
  uint32_t address_id;             // Device identifier
  uint8_t sub_address_id;          // Sub-device ID
  uint32_t shooter_address_id;     // Associated shooter ID
  uint8_t shooter_sub_address_id;  // Shooter sub-ID
  uint8_t status;                  // 0 = inactive, 1 = active
  GpsPayload gps_data;             // 4-byte GPS
};

/// @brief Log events for system status tracking
/// Used by OLED, SD logger, and serial monitor
struct LogEvents {
  uint32_t timestamp;         // Uptime-based timestamp (seconds)
  char generalEventType[16];  // e.g., "BOOT", "NORMAL"
  char lorawanEventType[16];  // e.g., "JOINED", "TX_OK"
  char gpsEventType[16];      // e.g., "GPS_OK", "NO_FIX"
  char httpEventType[16];     // e.g., "HTTP_OK", "CONN_FAIL"
};

/// @brief High-precision GPS storage for local use (OLED/SD)
/// Updated exclusively by updateGsmTask, protected by xGpsMutex
struct HighPrecisionGps {
  float latitude;
  float longitude;
};

#pragma pack()

// =============================================================================
// GLOBAL VARIABLES (SHARED BETWEEN TASKS)
// Protected by mutexes to ensure thread safety
// =============================================================================

DataPayload g_dataPayload = {
  .address_id = 1001,
  .sub_address_id = 1,
  .shooter_address_id = 2001,
  .shooter_sub_address_id = 1,
  .status = 1,
  .gps_data = { 0, 0 }
};

LogEvents g_logEvents = {
  .timestamp = 0,
  .generalEventType = "BOOT",
  .lorawanEventType = "INIT",
  .gpsEventType = "INIT",
  .httpEventType = "INIT"
};

HighPrecisionGps g_highPrecisionGps = { 0.0f, 0.0f };
SemaphoreHandle_t xGpsMutex = NULL;  // Protects g_highPrecisionGps

// Mutexes for thread-safe access
SemaphoreHandle_t xDataMutex = NULL;  // Protects g_dataPayload
SemaphoreHandle_t xLogMutex = NULL;   // Protects g_logEvents

// SD Card state
SPIClass sdspi = SPIClass(HSPI);  // Use HSPI to avoid LoRa conflict
bool sdInitialized = false;
char currentLogFileName[64] = "";

// OLED display
SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL);

// GSM modem and client (CRITICAL: accessed by ONE task only)
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

// LoRaWAN
osjob_t sendjob;
const lmic_pinmap lmic_pins = {
  .nss = LORA_CS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LORA_RST,
  .dio = { LORA_DIO0, LORA_DIO1, LORA_DIO2 },
};

// Task handles
TaskHandle_t xDisplayTaskHandle = NULL;
TaskHandle_t xSerialMonitorTaskHandle = NULL;
TaskHandle_t xTFCardTaskHandle = NULL;
TaskHandle_t xGsmTaskHandle = NULL;  // Single task for all modem operations

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/// @brief Blink built-in LED N times for error indication
/// @param count Number of blinks
/// @param delayMs Delay between on/off states (ms)
void blinkLED(int count, int delayMs) {
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delayMs);
  }
}

/// @brief Short LED pulse (50ms) for normal activity confirmation
void pulseLED() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
}

/// @brief Decode sflt16 (16-bit semi-float) to float in range [-1.0, 1.0)
/// @param val sflt16-encoded value
/// @return Decoded float
float GpsUint16toFloat(uint16_t val) {
  if (val == 0x8000) return -0.0f;  // Special case: -0.0
  bool isNegative = (val & 0x8000) != 0;
  int exponent = (val >> 11) & 0x0F;
  uint16_t mantissa = val & 0x07FF;
  float fraction = mantissa / 2048.0f;
  float result = fraction * powf(2.0f, exponent - 15);
  return isNegative ? -result : result;
}

/// @brief Ensure /logs directory exists on SD card
/// @return true on success
bool ensureLogDirectory() {
  if (!SD.exists("/logs")) {
    return SD.mkdir("/logs");
  }
  return true;
}

/// @brief Read log counter from /counter.txt (starts at 1)
/// @return Current counter value
uint32_t readCounterFromFile() {
  if (!SD.exists("/counter.txt")) return 1;
  File file = SD.open("/counter.txt", FILE_READ);
  if (!file) return 1;
  String content = file.readString();
  file.close();
  content.trim();
  return (content.length() > 0) ? content.toInt() : 1;
}

/// @brief Write new counter value to /counter.txt
/// @param counter Value to persist
void writeCounterToFile(uint32_t counter) {
  if (SD.exists("/counter.txt")) {
    SD.remove("/counter.txt");
  }
  File file = SD.open("/counter.txt", FILE_WRITE);
  if (file) {
    file.print(counter);
    file.close();
  }
}

/// @brief Initialize SD card and create new log file with CSV header
/// @return true on success
bool initSDLogging() {
  pinMode(SDCARD_CS, OUTPUT);
  digitalWrite(SDCARD_CS, HIGH);
  delay(10);
  sdspi.begin(SDCARD_SCLK, SDCARD_MISO, SDCARD_MOSI, SDCARD_CS);

  if (!SD.begin(SDCARD_CS, sdspi)) return false;
  if (SD.cardType() == CARD_NONE) return false;

  uint32_t counter = readCounterFromFile();
  snprintf(currentLogFileName, sizeof(currentLogFileName),
           "/logs/%s-log-%lu.csv", DEVICE_ID, counter);

  if (!ensureLogDirectory()) return false;
  if (SD.exists(currentLogFileName)) {
    SD.remove(currentLogFileName);
  }

  File logFile = SD.open(currentLogFileName, FILE_WRITE);
  if (!logFile) return false;
  logFile.println("timestamp,general,lorawan,gps,http,lat,lon,address,sub,shooter,shooter_sub,status");
  logFile.close();

  writeCounterToFile(counter + 1);
  return true;
}

// =============================================================================
// HTTP UTILITY FUNCTION (Called ONLY from updateGsmTask)
// Thread-safe by design (single task access to modem/client)
// =============================================================================
void sendHttpPayload() {
  // Step 1: Safely copy current device payload
  DataPayload localPayload;
  if (xDataMutex != NULL && xSemaphoreTake(xDataMutex, MUTEX_TIMEOUT) == pdTRUE) {
    memcpy(&localPayload, &g_dataPayload, sizeof(DataPayload));
    xSemaphoreGive(xDataMutex);
  } else {
    // Mutex timeout: log error and abort
    if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE) {
      strncpy(g_logEvents.httpEventType, "MUTEX_ERR", sizeof(g_logEvents.httpEventType) - 1);
      g_logEvents.httpEventType[sizeof(g_logEvents.httpEventType) - 1] = '\0';
      xSemaphoreGive(xLogMutex);
    }
    return;
  }

  // Step 2: Ensure GPRS is active
  if (!modem.isGprsConnected()) {
    modem.gprsDisconnect();
    delay(1000);
    if (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
      if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE) {
        strncpy(g_logEvents.httpEventType, "GPRS_FAIL", sizeof(g_logEvents.httpEventType) - 1);
        g_logEvents.httpEventType[sizeof(g_logEvents.httpEventType) - 1] = '\0';
        xSemaphoreGive(xLogMutex);
      }
      return;
    }
    delay(2000);  // Allow IP assignment
  }

  // Step 3: Connect to HTTP server
  if (!client.connect(SERVER, SERVER_PORT)) {
    if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE) {
      strncpy(g_logEvents.httpEventType, "CONN_FAIL", sizeof(g_logEvents.httpEventType) - 1);
      g_logEvents.httpEventType[sizeof(g_logEvents.httpEventType) - 1] = '\0';
      xSemaphoreGive(xLogMutex);
    }
    return;
  }

  // Step 4: Build and send POST request
  String httpRequestData = "api_key=" + String(API_KEY)
                           + "&address_id=" + String(localPayload.address_id)
                           + "&sub_address_id=" + String(localPayload.sub_address_id)
                           + "&shooter_address_id=" + String(localPayload.shooter_address_id)
                           + "&shooter_sub_address_id=" + String(localPayload.shooter_sub_address_id)
                           + "&status=" + String(localPayload.status)
                           + "&latitude_sflt16=" + String(localPayload.gps_data.latitude)
                           + "&longitude_sflt16=" + String(localPayload.gps_data.longitude);

  client.print(String("POST ") + RESOURCE + " HTTP/1.1\r\n");
  client.print(String("Host: ") + SERVER + "\r\n");
  client.println("Connection: close");
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.println(httpRequestData.length());
  client.println();
  client.print(httpRequestData);

  // Step 5: Wait for response (max 10s)
  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 10000) {
    while (client.available()) {
      client.read();  // Discard response body
    }
  }
  client.stop();

  // Step 6: Report success
  if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE) {
    strncpy(g_logEvents.httpEventType, "HTTP_OK", sizeof(g_logEvents.httpEventType) - 1);
    g_logEvents.httpEventType[sizeof(g_logEvents.httpEventType) - 1] = '\0';
    xSemaphoreGive(xLogMutex);
  }
  pulseLED();  // Visual confirmation
}

// =============================================================================
// FREE RTOS TASKS
// Each task runs independently with 8KB stack and pinned to specific core
// =============================================================================

/// @brief OLED Display Task: Cycles between system status and device info
/// Updates every second with mutex-safe data access
/// Page 1: System events and status
/// Page 2: Device identifiers and FULL-PRECISION GPS coordinates
void updateDisplayTask(void* pvParameters) {
  uint8_t page = 0;
  for (;;) {
    display.clear();

    // Attempt to acquire mutexes with timeout
    bool gotLog = (xLogMutex != NULL) && (xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE);
    bool gotData = (xDataMutex != NULL) && (xSemaphoreTake(xDataMutex, MUTEX_TIMEOUT) == pdTRUE);

    if (page == 0) {
      // Page 1: System events
      display.drawString(0, 0, String("GPS: ") + (gotLog ? g_logEvents.gpsEventType : "MUTEX_ERR"));
      display.drawString(0, 12, String("LoRa: ") + (gotLog ? g_logEvents.lorawanEventType : "MUTEX_ERR"));
      display.drawString(0, 24, String("HTTP: ") + (gotLog ? g_logEvents.httpEventType : "MUTEX_ERR"));
      display.drawString(0, 36, String("Gen: ") + (gotLog ? g_logEvents.generalEventType : "MUTEX_ERR"));
      display.drawString(0, 48, "Status: " + String(gotData ? g_dataPayload.status : 0));
    } else {
      // Page 2: Device info and FULL-PRECISION GPS (mutex-protected)
      display.drawString(0, 0, "Device Info");
      display.drawString(0, 12, "Addr: " + String(gotData ? g_dataPayload.address_id : 0));
      display.drawString(0, 24, "Shooter: " + String(gotData ? g_dataPayload.shooter_address_id : 0));
      
      // Read high-precision GPS with mutex protection
      float lat = 0.0f, lon = 0.0f;
      if (xGpsMutex != NULL && xSemaphoreTake(xGpsMutex, MUTEX_TIMEOUT) == pdTRUE) {
        lat = g_highPrecisionGps.latitude;
        lon = g_highPrecisionGps.longitude;
        xSemaphoreGive(xGpsMutex);
      }
      display.drawString(0, 36, String("Lat: ") + String(lat, 6));
      display.drawString(0, 48, String("Lon: ") + String(lon, 6));
      // NOTE: Removed duplicate raw access lines to ensure mutex safety
    }

    // Always release mutexes if acquired
    if (gotData) xSemaphoreGive(xDataMutex);
    if (gotLog) xSemaphoreGive(xLogMutex);

    display.display();
    page = 1 - page;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/// @brief Serial Monitor Task: Prints system status to USB serial
/// Disabled by default for field deployment (ENABLE_SERIAL_MON=0)
void updateSerialMonitorTask(void* pvParameters) {
#if ENABLE_SERIAL_MON
  Serial.begin(115200);
  vTaskDelay(1000);
  Serial.println("MILES Device - Serial Monitor");
#else
  vTaskDelete(NULL);  // Exit immediately if disabled
#endif

  for (;;) {
    // Initialize with error defaults
    char genEvt[16] = "ERR", loraEvt[16] = "ERR", gpsEvt[16] = "ERR", httpEvt[16] = "ERR";
    float lat = 0, lon = 0;
    uint32_t addr = 0, shooter = 0;
    uint8_t sub = 0, shooter_sub = 0, status = 0;

    // Safely copy shared data
    bool gotLog = (xLogMutex != NULL) && (xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE);
    bool gotData = (xDataMutex != NULL) && (xSemaphoreTake(xDataMutex, MUTEX_TIMEOUT) == pdTRUE);

    if (gotLog) {
      strncpy(genEvt, g_logEvents.generalEventType, sizeof(genEvt) - 1);
      strncpy(loraEvt, g_logEvents.lorawanEventType, sizeof(loraEvt) - 1);
      strncpy(gpsEvt, g_logEvents.gpsEventType, sizeof(gpsEvt) - 1);
      strncpy(httpEvt, g_logEvents.httpEventType, sizeof(httpEvt) - 1);
    }
    if (gotData) {
      lat = GpsUint16toFloat(g_dataPayload.gps_data.latitude);
      lon = GpsUint16toFloat(g_dataPayload.gps_data.longitude);
      addr = g_dataPayload.address_id;
      sub = g_dataPayload.sub_address_id;
      shooter = g_dataPayload.shooter_address_id;
      shooter_sub = g_dataPayload.shooter_sub_address_id;
      status = g_dataPayload.status;
    }

    // Release mutexes immediately after copy
    if (gotData) xSemaphoreGive(xDataMutex);
    if (gotLog) xSemaphoreGive(xLogMutex);

    // Print to USB serial
    Serial.printf("[LOG] Gen:%s | LoRa:%s | GPS:%s | HTTP:%s\n", genEvt, loraEvt, gpsEvt, httpEvt);
    Serial.printf("[GPS] Lat:%.6f Lon:%.6f\n", lat, lon);
    Serial.printf("[DEV] Addr:%u Sub:%u Shooter:%u Status:%u\n", addr, sub, shooter, status);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/// @brief SD Card Logging Task: Appends CSV log entry every second
/// Logs full-precision GPS coordinates (from g_highPrecisionGps) for local accuracy,
/// while LoRaWAN/HTTP use compact sflt16 encoding.
/// Skips logging if SD card is not initialized or mutex acquisition fails.
void updateTFCardTask(void* pvParameters) {
  for (;;) {
    // Skip if SD card not initialized
    if (!sdInitialized) {
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    // Attempt to acquire mutexes with timeout
    bool gotLog = (xLogMutex != NULL) && (xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE);
    bool gotData = (xDataMutex != NULL) && (xSemaphoreTake(xDataMutex, MUTEX_TIMEOUT) == pdTRUE);
    
    // Read high-precision GPS with mutex protection
    float lat = 0.0f, lon = 0.0f;
    if (xGpsMutex != NULL && xSemaphoreTake(xGpsMutex, MUTEX_TIMEOUT) == pdTRUE) {
      lat = g_highPrecisionGps.latitude;
      lon = g_highPrecisionGps.longitude;
      xSemaphoreGive(xGpsMutex);
    }

    // Log only if both main mutexes acquired successfully
    if (gotLog && gotData) {
      File file = SD.open(currentLogFileName, FILE_APPEND);
      if (file) {
        // Use local lat/lon variables (mutex-protected) for logging
        file.printf("%lu,%s,%s,%s,%s,%.6f,%.6f,%u,%u,%u,%u,%u\n",
                    millis() / 1000,                       // timestamp
                    g_logEvents.generalEventType,          // general
                    g_logEvents.lorawanEventType,          // lorawan
                    g_logEvents.gpsEventType,              // gps
                    g_logEvents.httpEventType,             // http
                    lat,                                   // FULL PRECISION LAT (mutex-protected)
                    lon,                                   // FULL PRECISION LON (mutex-protected)
                    g_dataPayload.address_id,              // address
                    g_dataPayload.sub_address_id,          // sub_address
                    g_dataPayload.shooter_address_id,      // shooter
                    g_dataPayload.shooter_sub_address_id,  // shooter_sub
                    g_dataPayload.status);                 // status
        file.close();
      }
    }

    // Always release main mutexes if acquired
    if (gotData) xSemaphoreGive(xDataMutex);
    if (gotLog) xSemaphoreGive(xLogMutex);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/// @brief GSM TASK: SINGLE TASK FOR ALL MODEM OPERATIONS (GPS + HTTP)
/// Critical for SIM808 stability — prevents AT command corruption.
/// - Acquires full-precision GPS fix every second
/// - Saves raw float coordinates for OLED/SD logging
/// - Encodes normalized coordinates as sflt16 for LoRaWAN/HTTP
/// - Sends HTTP payload every 10 seconds
void updateGsmTask(void* pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(5000));  // Allow system to stabilize after boot

  for (;;) {
    // Step 1: Attempt to get GPS fix
    float lat = 0.0f, lon = 0.0f;
    bool hasFix = modem.getGPS(&lat, &lon);

    if (hasFix && lat != 0.0f && lon != 0.0f) {
      // >>> SAVE FULL-PRECISION GPS FOR LOCAL USE (OLED + SD) <<<
      if (xGpsMutex != NULL && xSemaphoreTake(xGpsMutex, MUTEX_TIMEOUT) == pdTRUE) {
        g_highPrecisionGps.latitude = lat;
        g_highPrecisionGps.longitude = lon;
        xSemaphoreGive(xGpsMutex);
      }

      // Step 2: Normalize to [-1, 1) range for sflt16 encoding
      // Latitude: [-90, 90] → [-1, 1]
      // Longitude: [-180, 180] → [-1, 1]
      float normLat = lat / 90.0f;
      float normLon = lon / 180.0f;

      // Clamp to valid range to prevent sflt16 overflow
      // Use 0.9999 to stay within [-1, 1) open interval
      normLat = constrain(normLat, -0.9999f, 0.9999f);
      normLon = constrain(normLon, -0.9999f, 0.9999f);

      // Step 3: Encode as sflt16 for compact LoRaWAN/HTTP payload
      uint16_t lat_sflt = LMIC_f2sflt16(normLat);
      uint16_t lon_sflt = LMIC_f2sflt16(normLon);

      // Step 4: Update shared LoRa/HTTP payload (mutex-protected)
      if (xDataMutex != NULL && xSemaphoreTake(xDataMutex, MUTEX_TIMEOUT) == pdTRUE) {
        g_dataPayload.gps_data.latitude = lat_sflt;
        g_dataPayload.gps_data.longitude = lon_sflt;
        xSemaphoreGive(xDataMutex);
      }

      // Step 5: Update GPS event status
      if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE) {
        strncpy(g_logEvents.gpsEventType, "GPS_OK", sizeof(g_logEvents.gpsEventType) - 1);
        g_logEvents.gpsEventType[sizeof(g_logEvents.gpsEventType) - 1] = '\0';
        xSemaphoreGive(xLogMutex);
      }
    } else {
      // No valid GPS fix: clear high-precision values
      if (xGpsMutex != NULL && xSemaphoreTake(xGpsMutex, MUTEX_TIMEOUT) == pdTRUE) {
        g_highPrecisionGps.latitude = 0.0f;
        g_highPrecisionGps.longitude = 0.0f;
        xSemaphoreGive(xGpsMutex);
      }

      // Update GPS event status
      if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE) {
        strncpy(g_logEvents.gpsEventType, "NO_FIX", sizeof(g_logEvents.gpsEventType) - 1);
        g_logEvents.gpsEventType[sizeof(g_logEvents.gpsEventType) - 1] = '\0';
        xSemaphoreGive(xLogMutex);
      }
    }

    // Visual heartbeat (pulses every second)
    pulseLED();

    // Step 6: Send HTTP payload every 10 seconds
    static uint32_t lastHttp = 0;
    if (millis() - lastHttp >= HTTP_REPORT_INTERVAL_MS) {
      sendHttpPayload();
      lastHttp = millis();
    }

    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =============================================================================
// LORAWAN CALLBACKS (MUTEX-SAFE)
// =============================================================================

/// @brief LMIC event handler: Updates LoRaWAN status in shared log events
/// Called by LMIC stack on network events (join, TX, etc.)
void onEvent(ev_t ev) {
  const char* evStr = "?";

  // Map LMIC event codes to human-readable strings
  switch (ev) {
    case EV_JOINING: evStr = "JOINING"; break;
    case EV_JOINED: evStr = "JOINED"; break;
    case EV_JOIN_FAILED: evStr = "JOIN_FAIL"; break;
    case EV_REJOIN_FAILED: evStr = "REJOIN_FAIL"; break;
    case EV_TXCOMPLETE:
      evStr = "TX_OK";
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC: evStr = "LOST_TSYNC"; break;
    case EV_RESET: evStr = "RESET"; break;
    case EV_RXCOMPLETE: evStr = "RX_OK"; break;
    case EV_LINK_DEAD: evStr = "LINK_DEAD"; break;
    case EV_LINK_ALIVE: evStr = "LINK_ALIVE"; break;
    default: evStr = "UNKNOWN";
  }

  // Update shared log (mutex-safe with timeout)
  if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE) {
    strncpy(g_logEvents.lorawanEventType, evStr, sizeof(g_logEvents.lorawanEventType) - 1);
    g_logEvents.lorawanEventType[sizeof(g_logEvents.lorawanEventType) - 1] = '\0';
    xSemaphoreGive(xLogMutex);
  }

  // Visual feedback on successful transmission
  if (ev == EV_TXCOMPLETE) {
    pulseLED();
  }
}

/// @brief Prepare and send LoRaWAN payload
/// Copies current data (mutex-safe) and schedules TX
void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) return;  // Skip if TX already pending

  DataPayload localPayload;
  if (xDataMutex != NULL && xSemaphoreTake(xDataMutex, MUTEX_TIMEOUT) == pdTRUE) {
    memcpy(&localPayload, &g_dataPayload, sizeof(DataPayload));
    xSemaphoreGive(xDataMutex);
  } else {
    return;  // Skip if mutex not available
  }

  // Safety check: ensure payload fits in allowed size (51 bytes for US915 DR3)
  if (sizeof(DataPayload) > 51) return;

  // Send unconfirmed message on port 1
  LMIC_setTxData2(1, (uint8_t*)&localPayload, sizeof(DataPayload), 0);
}

// =============================================================================
// SYSTEM INITIALIZATION
// =============================================================================
void setup() {
  // Initialize built-in LED (active HIGH)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize OLED display
  display.init();
  display.flipScreenVertically();  // T3-S3 has inverted display
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawString(0, 0, "MILES DEVICE");
  display.drawString(0, 12, "Initializing...");
  display.display();
  delay(500);

  // Initialize LoRa SPI bus
  display.clear();
  display.drawString(0, 0, "Init LoRa SPI...");
  display.display();
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  delay(200);

  // Initialize SD card logging
  display.clear();
  display.drawString(0, 0, "Init SD Card...");
  display.display();
  sdInitialized = initSDLogging();
  if (!sdInitialized) {
    display.drawString(0, 12, "SD INIT FAILED!");
    display.display();
    blinkLED(5, 300);  // 5 slow blinks = SD error
    delay(2000);
  } else {
    display.drawString(0, 12, "SD OK");
    display.display();
    delay(500);
  }

  // Initialize SIM808 modem (CRITICAL: 115200 baud)
  display.clear();
  display.drawString(0, 0, "Init Modem...");
  display.display();
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1000);

  if (!modem.restart()) {
    display.clear();
    display.drawString(0, 0, "MODEM FAILED!");
    display.drawString(0, 12, "Check wiring!");
    display.display();
    blinkLED(20, 200);        // Rapid blink = modem failure
    while (1) vTaskDelay(1);  // Halt
  }
  display.drawString(0, 12, "Modem OK");
  display.display();
  delay(500);

  // Enable GPS on SIM808
  display.clear();
  display.drawString(0, 0, "Enable GPS...");
  display.display();
  if (!modem.enableGPS()) {
    display.drawString(0, 12, "GPS ENABLE FAIL");
    display.display();
    blinkLED(10, 300);  // 10 blinks = GPS error
    delay(2000);
  } else {
    display.drawString(0, 12, "GPS Enabled");
    display.display();
    delay(500);
  }

  // Connect to GPRS network and update HTTP status
  display.clear();
  display.drawString(0, 0, "Init GPRS...");
  display.display();
  modem.gprsDisconnect();
  delay(1000);
  bool gprsOk = modem.gprsConnect(APN, GPRS_USER, GPRS_PASS);
  if (gprsOk) {
    display.drawString(0, 12, "GPRS OK");
    // Update HTTP event to reflect GPRS success
    if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE) {
      strncpy(g_logEvents.httpEventType, "GPRS_OK", sizeof(g_logEvents.httpEventType) - 1);
      g_logEvents.httpEventType[sizeof(g_logEvents.httpEventType) - 1] = '\0';
      xSemaphoreGive(xLogMutex);
    }
  } else {
    display.drawString(0, 12, "GPRS FAIL");
    blinkLED(3, 500);  // 3 slow blinks = GPRS error
    // Update HTTP event to reflect GPRS failure
    if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE) {
      strncpy(g_logEvents.httpEventType, "GPRS_FAIL", sizeof(g_logEvents.httpEventType) - 1);
      g_logEvents.httpEventType[sizeof(g_logEvents.httpEventType) - 1] = '\0';
      xSemaphoreGive(xLogMutex);
    }
  }
  display.display();
  delay(1000);

  // Create mutexes for thread-safe data sharing
  xDataMutex = xSemaphoreCreateMutex();
  xLogMutex = xSemaphoreCreateMutex();
  xGpsMutex = xSemaphoreCreateMutex();

  // Start FreeRTOS tasks (8KB stack, pinned to cores)
  display.clear();
  display.drawString(0, 0, "Starting Tasks...");
  display.display();
  delay(300);

  xTaskCreatePinnedToCore(updateDisplayTask, "DisplayTask", 8192, NULL, 2, &xDisplayTaskHandle, 1);

#if ENABLE_SERIAL_MON
  xTaskCreatePinnedToCore(updateSerialMonitorTask, "SerialMon", 8192, NULL, 1, &xSerialMonitorTaskHandle, 0);
#endif

  xTaskCreatePinnedToCore(updateTFCardTask, "SDLogger", 8192, NULL, 1, &xTFCardTaskHandle, 1);
  // CRITICAL: Only ONE task handles modem (GPS + HTTP)
  xTaskCreatePinnedToCore(updateGsmTask, "GsmTask", 8192, NULL, 1, &xGsmTaskHandle, 0);

  // Initialize LoRaWAN stack
  display.clear();
  display.drawString(0, 0, "Init LoRaWAN...");
  display.display();
  os_init();
  LMIC_reset();
  LMIC_setAdrMode(1);                 // Enable Adaptive Data Rate
  LMIC_setDrTxpow(DR_SF9, 14);        // Data Rate: SF9, TX Power: 14 dBm
  LMIC_setLinkCheckMode(0);           // Disable link check (not needed for Class A)
  os_setCallback(&sendjob, do_send);  // Schedule first transmission

  // Final system ready message
  display.clear();
  display.drawString(0, 0, "SYSTEM READY");
  display.drawString(0, 12, "Joining network...");
  display.display();
  delay(1000);

  // Update general event to "NORMAL" after successful init
  if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, MUTEX_TIMEOUT) == pdTRUE) {
    strncpy(g_logEvents.generalEventType, "NORMAL", sizeof(g_logEvents.generalEventType) - 1);
    g_logEvents.generalEventType[sizeof(g_logEvents.generalEventType) - 1] = '\0';
    xSemaphoreGive(xLogMutex);
  }
}

// =============================================================================
// MAIN EVENT LOOP
// =============================================================================
void loop() {
  os_runloop_once();  // Process LMIC events (non-blocking)
  vTaskDelay(1);      // Yield to other FreeRTOS tasks
}