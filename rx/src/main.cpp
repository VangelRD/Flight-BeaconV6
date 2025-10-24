/**
 * V-Flight Computer - Receiver (RX)
 * Barebones implementation for ground station
 *
 * Features:
 * - LoRa packet reception
 * - Local data logging
 * - Terminal display
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <LittleFS.h>

// ===== PIN DEFINITIONS =====
// LoRa SPI (same as TX)
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS 25
#define LORA_RST 14
#define LORA_DIO0 4

// ===== LORA SETTINGS (MUST MATCH TX) =====
#define LORA_FREQ 433E6         // 433 MHz
#define LORA_BANDWIDTH 125E3    // 125 kHz
#define LORA_SPREADING 9        // SF9
#define LORA_CODING_RATE 6      // 4/6

// ===== GLOBALS =====
File logFile;
unsigned long packetsReceived = 0;
unsigned long lastPacketTime = 0;
int lastRSSI = 0;
float lastSNR = 0;

// Received data structure (matches TX)
struct ReceivedData {
  unsigned long timestamp;
  float pressure;
  float altitude;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float gpsLat, gpsLon;
} rxData;

// ===== SETUP FUNCTIONS =====
void setupFilesystem() {
  Serial.println("[FS] Initializing LittleFS...");
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] ERROR: Failed to mount filesystem");
    return;
  }

  // Create timestamped log file
  char filename[32];
  snprintf(filename, sizeof(filename), "/rx_log_%lu.csv", millis());
  logFile = LittleFS.open(filename, "w");

  if (!logFile) {
    Serial.println("[FS] ERROR: Failed to create log file");
    return;
  }

  // Write CSV header
  logFile.println("rx_time,tx_time,pressure,altitude,ax,ay,az,gx,gy,gz,lat,lon,rssi,snr");
  logFile.flush();

  Serial.printf("[FS] Logging to: %s\n", filename);
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
  
  // Set sync word (must match TX) - 0x12 is default, but set explicitly
  LoRa.setSyncWord(0x12);
  
  // Enable CRC checking
  LoRa.enableCrc();
  
  // Set preamble length (must match TX)
  LoRa.setPreambleLength(8);

  Serial.println("[LoRa] Configuration:");
  Serial.printf("  Frequency: %.2f MHz\n", LORA_FREQ / 1E6);
  Serial.printf("  Bandwidth: %.1f kHz\n", LORA_BANDWIDTH / 1E3);
  Serial.printf("  Spreading Factor: %d\n", LORA_SPREADING);
  Serial.printf("  Coding Rate: 4/%d\n", LORA_CODING_RATE);
  Serial.println("[LoRa] Ready - Waiting for packets...");
}

// ===== RECEIVE FUNCTIONS =====
void decodePacket(uint8_t* packet, int packetSize) {
  if (packetSize < 32) {
    Serial.printf("[RX] WARNING: Short packet (%d bytes) - skipping\n", packetSize);
    return;  // Don't decode incomplete packets
  }

  // Decode binary packet (matches TX format)
  memcpy(&rxData.timestamp, &packet[0], 4);
  memcpy(&rxData.pressure, &packet[4], 4);
  memcpy(&rxData.altitude, &packet[8], 4);

  // Validate floats - reject if NaN or Inf (corrupted packet)
  if (isnan(rxData.pressure) || isinf(rxData.pressure) ||
      isnan(rxData.altitude) || isinf(rxData.altitude)) {
    Serial.println("[RX] WARNING: Corrupted packet (NaN/Inf values) - skipping");
    return;
  }

  // Unpack accel/gyro
  int16_t ax, ay, az, gx, gy, gz;
  memcpy(&ax, &packet[12], 2);
  memcpy(&ay, &packet[14], 2);
  memcpy(&az, &packet[16], 2);
  memcpy(&gx, &packet[18], 2);
  memcpy(&gy, &packet[20], 2);
  memcpy(&gz, &packet[22], 2);

  rxData.accelX = ax / 100.0;
  rxData.accelY = ay / 100.0;
  rxData.accelZ = az / 100.0;
  rxData.gyroX = gx / 100.0;
  rxData.gyroY = gy / 100.0;
  rxData.gyroZ = gz / 100.0;

  // GPS
  memcpy(&rxData.gpsLat, &packet[24], 4);
  memcpy(&rxData.gpsLon, &packet[28], 4);

  // Validate GPS floats
  if (isnan(rxData.gpsLat) || isinf(rxData.gpsLat) ||
      isnan(rxData.gpsLon) || isinf(rxData.gpsLon)) {
    // Set to zero if invalid (don't reject entire packet for bad GPS)
    rxData.gpsLat = 0.0;
    rxData.gpsLon = 0.0;
  }

  // Get signal quality
  lastRSSI = LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
}

void logReceivedData() {
  if (!logFile) return;

  // Extra safety: validate all values before logging
  if (isnan(rxData.pressure) || isinf(rxData.pressure) ||
      isnan(rxData.altitude) || isinf(rxData.altitude) ||
      isnan(rxData.gpsLat) || isinf(rxData.gpsLat)) {
    Serial.println("[LOG] Skipping log entry - invalid data");
    return;
  }

  // CSV: rx_time,tx_time,pressure,altitude,ax,ay,az,gx,gy,gz,lat,lon,rssi,snr
  logFile.printf("%lu,%lu,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%d,%.2f\n",
    millis(), rxData.timestamp,
    rxData.pressure, rxData.altitude,
    rxData.accelX, rxData.accelY, rxData.accelZ,
    rxData.gyroX, rxData.gyroY, rxData.gyroZ,
    rxData.gpsLat, rxData.gpsLon,
    lastRSSI, lastSNR
  );

  // Flush every 10 packets
  if (packetsReceived % 10 == 0) {
    logFile.flush();
  }
}

void displayData() {
  // Extra safety: validate before displaying
  if (isnan(rxData.pressure) || isinf(rxData.pressure) ||
      isnan(rxData.altitude) || isinf(rxData.altitude)) {
    Serial.println("[DISPLAY] Skipping corrupted packet display");
    return;
  }

  Serial.println("\n========================================");
  Serial.printf("PKT #%lu | RSSI: %d dBm | SNR: %.1f dB\n", packetsReceived, lastRSSI, lastSNR);
  Serial.println("----------------------------------------");
  Serial.printf("TX Time    : %lu ms\n", rxData.timestamp);
  Serial.printf("Pressure   : %.2f Pa\n", rxData.pressure);
  Serial.printf("Altitude   : %.1f m\n", rxData.altitude);
  Serial.printf("Accel      : X:%.2f Y:%.2f Z:%.2f m/s2\n", rxData.accelX, rxData.accelY, rxData.accelZ);
  Serial.printf("Gyro       : X:%.2f Y:%.2f Z:%.2f rad/s\n", rxData.gyroX, rxData.gyroY, rxData.gyroZ);
  Serial.printf("GPS        : %.6f, %.6f\n", rxData.gpsLat, rxData.gpsLon);
  Serial.println("========================================\n");
}

// ===== PACKET PROCESSING =====
void processPacket(int packetSize) {
  if (packetSize == 0) return;

  // Read packet
  uint8_t packet[256];
  int idx = 0;
  while (LoRa.available() && idx < 256) {
    packet[idx++] = LoRa.read();
  }

  // Increment counter first
  packetsReceived++;
  lastPacketTime = millis();

  // Only process if packet is complete
  if (packetSize >= 32) {
    decodePacket(packet, packetSize);
    logReceivedData();
    displayData();
  } else {
    Serial.printf("[RX] Skipped short packet (%d bytes)\n", packetSize);
  }
}

// ===== MAIN =====
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n===========================================");
  Serial.println("V-FLIGHT COMPUTER - RECEIVER (RX)");
  Serial.println("===========================================\n");

  setupFilesystem();
  setupLoRa();

  // Put receiver in continuous mode (using polling, not callback)
  LoRa.receive();

  Serial.println("\n[RX] Ground station ready - waiting for telemetry\n");
}

void loop() {
  // Poll for packets (no callback - simpler and more reliable)
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    processPacket(packetSize);
  }
  
  // Status update every second
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 1000) {
    lastStatusTime = millis();
    
    if (packetsReceived == 0) {
      Serial.println("[RX] Listening... (no packets yet)");
    } else {
      unsigned long timeSinceLastPacket = millis() - lastPacketTime;
      Serial.printf("[RX] Total packets: %lu | Last: %lu ms ago\n", 
                    packetsReceived, timeSinceLastPacket);
    }
  }

  // No delay - maximize packet capture
  yield();
}
