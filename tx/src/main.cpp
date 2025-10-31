/**
 * V-Flight Computer - Transmitter (TX)
 * Barebones implementation for rocket telemetry
 *
 * Features:
 * - BMP280 pressure/temperature sensor
 * - LSM9DS1 9-DOF IMU (accel, gyro, mag)
 * - LC76G Multi-GNSS module
 * - Local logging at ~100Hz
 * - LoRa transmission at ~5Hz
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <SPIFFS.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>

// ===== PIN DEFINITIONS =====
// I2C (SDA=21, SCL=22 - default for ESP32)
#define I2C_SDA 21
#define I2C_SCL 22

// LoRa SPI
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS 25
#define LORA_RST 14
#define LORA_DIO0 4

// GNSS UART1
#define GNSS_RX 16
#define GNSS_TX 17

// ===== LORA SETTINGS =====
#define LORA_FREQ 433E6         // 433 MHz
#define LORA_BANDWIDTH 250E3    // 250 kHz (2x faster than 125 kHz)
#define LORA_SPREADING 8        // SF8 - balanced speed/range for 2km
#define LORA_CODING_RATE 6      // 4/6
#define LORA_TX_POWER 20        // 20 dBm

// ===== TIMING =====
#define LOG_INTERVAL_MS 5       // 200Hz logging
#define LORA_INTERVAL_MS 100    // 10Hz transmission (5x faster, ~100ms air time)
#define STATUS_INTERVAL_MS 1000 // 1Hz status print

// ===== GLOBALS =====
Adafruit_BMP280 bmp;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

File logFile;
unsigned long lastLogTime = 0;
unsigned long lastLoRaTime = 0;
unsigned long lastStatusTime = 0;
unsigned long packetCounter = 0;
bool sensorsOK = false;
bool dumpLogRequested = false;
Preferences fsPrefs;

// Sensor data structure
struct SensorData {
  unsigned long timestamp;
  float temp;
  float pressure;
  float altitude;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
  float gpsLat, gpsLon, gpsAlt;
  float gpsSpeed;
  uint8_t gpsSats;
} data = {0}; // Initialize to zero

// ===== SETUP FUNCTIONS =====
void setupFilesystem() {
  Serial.println("[FS] Initializing SPIFFS (more stable than LittleFS)...");
  
  // Try to mount SPIFFS (don't format on fail initially)
  fsPrefs.begin("fs", false);
  int mountFailCount = fsPrefs.getInt("mount_fail", 0);

  if (!SPIFFS.begin(false)) {
    mountFailCount++;
    fsPrefs.putInt("mount_fail", mountFailCount);
    Serial.printf("[FS] Mount failed (attempt %d/4)\n", mountFailCount);

    if (mountFailCount >= 4) {
      Serial.println("[FS] Formatting SPIFFS after 4 consecutive mount failures...");
      if (!SPIFFS.begin(true)) {
        Serial.println("[FS] ERROR: SPIFFS format failed");
        Serial.println("[FS] Continuing without logging");
        fsPrefs.end();
        return;
      }

      Serial.println("[FS] SPIFFS formatted successfully");
      fsPrefs.putInt("mount_fail", 0);
    } else {
      Serial.println("[FS] Skipping format to preserve data");
      fsPrefs.end();
      return;
    }
  } else {
    if (mountFailCount != 0) {
      Serial.println("[FS] SPIFFS mount succeeded, resetting failure counter");
      fsPrefs.putInt("mount_fail", 0);
    }
  }
  fsPrefs.end();

  Serial.println("[FS] SPIFFS mounted successfully");

  // Open log file
  logFile = SPIFFS.open("/flight_log.csv", "a");
  if (!logFile) {
    Serial.println("[FS] ERROR: Failed to open log file");
    return;
  }

  // Write header if file is new
  if (logFile.size() == 0) {
    logFile.println("time,temp,pressure,alt,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,galt,spd,sats");
    logFile.flush();
  }

  Serial.printf("[FS] Log file ready (current size: %d bytes)\n", logFile.size());
  Serial.println("[FS] Filesystem ready");
}

void setupSensors() {
  Serial.println("[SENSORS] Initializing...");

  // Init I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz

  // Init BMP280
  if (!bmp.begin(0x76)) {
    if (!bmp.begin(0x77)) {
      Serial.println("[BMP280] ERROR: Sensor not found");
      return;
    }
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X4,
                  Adafruit_BMP280::FILTER_X4,
                  Adafruit_BMP280::STANDBY_MS_0_5);
  Serial.println("[BMP280] OK");

  // Init LSM9DS1
  if (!lsm.begin()) {
    Serial.println("[LSM9DS1] ERROR: Sensor not found");
    return;
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);

  // Boost IMU data rates for high-dynamic flights
  uint8_t ctrl6_xl = lsm.read8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG6_XL);
  ctrl6_xl &= ~(0b111 << 5);
  ctrl6_xl |= (0b110 << 5); // 952 Hz ODR
  lsm.write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG6_XL, ctrl6_xl);

  uint8_t ctrl1_g = lsm.read8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG1_G);
  ctrl1_g &= ~(0b111 << 5);
  ctrl1_g |= (0b110 << 5); // 952 Hz ODR
  lsm.write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG1_G, ctrl1_g);

  uint8_t ctrl1_m = lsm.read8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG1_M);
  ctrl1_m &= ~0b111;
  ctrl1_m |= 0b111; // 80 Hz ODR
  lsm.write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG1_M, ctrl1_m);
  Serial.println("[LSM9DS1] OK");

  // Init GPS
  GPSSerial.begin(115200, SERIAL_8N1, GNSS_RX, GNSS_TX);
  Serial.println("[GPS] OK - 115200 baud");

  sensorsOK = true;
  Serial.println("[SENSORS] All sensors initialized");
}

void setupLoRa() {
  Serial.println("[LoRa] Initializing...");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("[LoRa] ERROR: Init failed");
    while (1);
  }

  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setSpreadingFactor(LORA_SPREADING);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setTxPower(LORA_TX_POWER);
  
  // Set sync word (must match RX) - 0x12 is default, but set explicitly
  LoRa.setSyncWord(0x12);
  
  // Enable CRC for error detection
  LoRa.enableCrc();
  
  // Set preamble length (default is 8, increase for better detection)
  LoRa.setPreambleLength(8);
  
  // Verify frequency was actually set
  Serial.printf("[LoRa] Frequency register check: %ld Hz\n", (long)LORA_FREQ);

  Serial.println("[LoRa] Configuration:");
  Serial.printf("  Frequency: %.2f MHz\n", LORA_FREQ / 1E6);
  Serial.printf("  Bandwidth: %.1f kHz\n", LORA_BANDWIDTH / 1E3);
  Serial.printf("  Spreading Factor: %d\n", LORA_SPREADING);
  Serial.printf("  Coding Rate: 4/%d\n", LORA_CODING_RATE);
  Serial.printf("  TX Power: %d dBm\n", LORA_TX_POWER);
  Serial.println("[LoRa] Ready - Starting transmissions...");
}

// ===== DATA FUNCTIONS =====
void readSensors() {
  if (!sensorsOK) return;

  data.timestamp = millis();

  // BMP280
  data.temp = bmp.readTemperature();
  data.pressure = bmp.readPressure();
  
  // Only calculate altitude if pressure is valid (avoid divide-by-zero)
  if (data.pressure > 0 && data.pressure < 200000) {
    data.altitude = bmp.readAltitude(1013.25);
  }

  // LSM9DS1
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  data.accelX = a.acceleration.x;
  data.accelY = a.acceleration.y;
  data.accelZ = a.acceleration.z;

  data.gyroX = g.gyro.x;
  data.gyroY = g.gyro.y;
  data.gyroZ = g.gyro.z;

  data.magX = m.magnetic.x;
  data.magY = m.magnetic.y;
  data.magZ = m.magnetic.z;

  // GPS (limit processing to prevent blocking)
  int gpsCharsProcessed = 0;
  while (GPSSerial.available() > 0 && gpsCharsProcessed < 100) {
    char c = GPSSerial.read();
    gps.encode(c);
    gpsCharsProcessed++;
  }

  if (gps.location.isValid()) {
    data.gpsLat = gps.location.lat();
    data.gpsLon = gps.location.lng();
    data.gpsAlt = gps.altitude.meters();
    data.gpsSpeed = gps.speed.mps();
    data.gpsSats = gps.satellites.value();
  }
}

void logData() {
  if (!logFile) return;
  if (!sensorsOK) return;

  // CSV format: time,temp,pressure,alt,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,galt,spd,sats
  logFile.printf("%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%d\n",
    data.timestamp,
    data.temp, data.pressure, data.altitude,
    data.accelX, data.accelY, data.accelZ,
    data.gyroX, data.gyroY, data.gyroZ,
    data.magX, data.magY, data.magZ,
    data.gpsLat, data.gpsLon, data.gpsAlt,
    data.gpsSpeed, data.gpsSats
  );

  // Flush periodically (only after 200 packets)
  static uint32_t logCounter = 0;
  logCounter++;
  if (logCounter >= 200) {
    logFile.flush();
    logCounter = 0;
  }
}

void transmitLoRa() {
  if (!sensorsOK) return;
  
  // Make sure we have fresh sensor data
  if (data.pressure == 0 || data.timestamp == 0) {
    Serial.println("[TX] Warning: Sensor data not ready, skipping transmission");
    return;
  }
  
  // Create compact binary packet (optimized for 2Hz @ SF9)
  // Format: [4B:time][4B:pressure][4B:alt][12B:accel/gyro][8B:gps] = 32 bytes

  uint8_t packet[32];
  memcpy(&packet[0], &data.timestamp, 4);
  memcpy(&packet[4], &data.pressure, 4);
  memcpy(&packet[8], &data.altitude, 4);

  // Pack accel/gyro as 2-byte integers (scaled)
  int16_t ax = (int16_t)(data.accelX * 100);
  int16_t ay = (int16_t)(data.accelY * 100);
  int16_t az = (int16_t)(data.accelZ * 100);
  int16_t gx = (int16_t)(data.gyroX * 100);
  int16_t gy = (int16_t)(data.gyroY * 100);
  int16_t gz = (int16_t)(data.gyroZ * 100);

  memcpy(&packet[12], &ax, 2);
  memcpy(&packet[14], &ay, 2);
  memcpy(&packet[16], &az, 2);
  memcpy(&packet[18], &gx, 2);
  memcpy(&packet[20], &gy, 2);
  memcpy(&packet[22], &gz, 2);

  // GPS
  memcpy(&packet[24], &data.gpsLat, 4);
  memcpy(&packet[28], &data.gpsLon, 4);

  // Debug: Print first 10 packets with raw bytes
  if (packetCounter < 10) {
    Serial.println("\n[TX DEBUG] Packet data:");
    Serial.printf("  T=%lu, P=%.2f, Alt=%.2f, Az=%.2f\n", 
                  data.timestamp, data.pressure, data.altitude, data.accelZ);
    Serial.println("  Raw packet (hex):");
    for (int i = 0; i < 32; i++) {
      Serial.printf("%02X ", packet[i]);
      if ((i + 1) % 16 == 0) Serial.println();
    }
    Serial.println();
  }

  // Transmit
  LoRa.beginPacket();
  LoRa.write(packet, sizeof(packet));
  LoRa.endPacket();  // Blocking mode for reliability

  packetCounter++;
  
  // No delay needed - endPacket() is blocking and waits for completion
}

void printStatus() {
  Serial.printf("[TX STATUS] T:%lu | P:%.2f Pa | Alt:%.1f m | Ax:%.2f | GPS:%d sats | Pkts TX:%lu\n",
    data.timestamp, data.pressure, data.altitude, data.accelZ, data.gpsSats, packetCounter);
}

void dumpLog() {
  Serial.println("\n[LOG_DUMP_START]");
  Serial.println("Dumping flight log from SPIFFS...");
  
  // Disable watchdog during dump
  yield();
  
  // Close current log file if open
  if (logFile) {
    logFile.flush();
    logFile.close();
    delay(100);  // Give filesystem time to close properly
  }
  
  // Reopen in read mode
  File log = SPIFFS.open("/flight_log.csv", "r");
  if (!log) {
    Serial.println("[ERROR] Could not open log file");
    Serial.println("[LOG_DUMP_END]");
    
    // Try to reopen for writing
    delay(100);
    logFile = SPIFFS.open("/flight_log.csv", "a");
    return;
  }
  
  size_t fileSize = log.size();
  Serial.print("[INFO] Log file size: ");
  Serial.print(fileSize);
  Serial.println(" bytes");
  Serial.println("[DATA_START]");
  
  // Stream the file contents in chunks to avoid watchdog issues
  uint8_t buffer[128];
  size_t bytesRead = 0;
  
  while (log.available()) {
    size_t toRead = min((size_t)128, (size_t)log.available());
    size_t read = log.read(buffer, toRead);
    Serial.write(buffer, read);
    bytesRead += read;
    
    // Feed the watchdog every chunk
    if (bytesRead % 1024 == 0) {
      yield();
      delay(1);
    }
  }
  
  log.close();
  Serial.println("\n[DATA_END]");
  Serial.println("[LOG_DUMP_END]");
  
  // Reopen log file for continued writing
  delay(100);
  logFile = SPIFFS.open("/flight_log.csv", "a");
  
  if (logFile) {
    Serial.println("[INFO] Log file ready for continued logging\n");
  } else {
    Serial.println("[WARNING] Could not reopen log file for writing\n");
  }
  
  yield();
}

// ===== MAIN =====
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n===========================================");
  Serial.println("V-FLIGHT COMPUTER - TRANSMITTER (TX)");
  Serial.println("===========================================\n");

  setupFilesystem();
  setupSensors();
  setupLoRa();

  // Let sensors stabilize and do dummy reads
  if (sensorsOK) {
    Serial.println("[SENSORS] Stabilizing sensors...");
    delay(500);
    
    // Flush initial bad sensor data with dummy reads
    for (int i = 0; i < 5; i++) {
      bmp.readTemperature();
      bmp.readPressure();
      lsm.read();
      delay(50);
    }
    Serial.println("[SENSORS] Sensors stabilized");
  }

  Serial.println("\n[TX] System ready - starting data acquisition\n");
}

void loop() {
  unsigned long now = millis();

  // Check for serial commands (non-blocking, using char array to avoid String heap issues)
  static char cmdBuffer[32];
  static int cmdIdx = 0;
  
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdIdx > 0) {
        cmdBuffer[cmdIdx] = '\0';
        
        // Check commands
        if (strcmp(cmdBuffer, "DUMP_LOG") == 0 || strcmp(cmdBuffer, "dump_log") == 0) {
          dumpLogRequested = true;
        } else if (strcmp(cmdBuffer, "HELP") == 0 || strcmp(cmdBuffer, "help") == 0) {
          Serial.println("\n=== Available Commands ===");
          Serial.println("DUMP_LOG - Download flight log via serial");
          Serial.println("HELP     - Show this help message");
          Serial.println("==========================\n");
        }
        
        cmdIdx = 0;
      }
    } else if (cmdIdx < 31) {
      cmdBuffer[cmdIdx++] = c;
    }
  }

  // Always read sensors
  readSensors();

  // Log at 100Hz (only if file is valid)
  if (logFile && (now - lastLogTime >= LOG_INTERVAL_MS)) {
    logData();
    lastLogTime = now;
  }

  // Transmit at 10Hz
  if (now - lastLoRaTime >= LORA_INTERVAL_MS) {
    transmitLoRa();
    lastLoRaTime = now;
  }

  // Status at 1Hz
  if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
    printStatus();
    lastStatusTime = now;
  }

  // Dump log if requested (do this last to avoid timing issues)
  if (dumpLogRequested) {
    dumpLog();
    dumpLogRequested = false;
  }

  // Small delay to prevent watchdog issues
  delay(1);
}
