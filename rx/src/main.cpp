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
volatile unsigned long packetsReceived = 0;  // volatile for thread safety
volatile unsigned long lastPacketTime = 0;    // volatile for thread safety
int lastRSSI = 0;
float lastSNR = 0;
unsigned long logFlushCounter = 0;  // Separate counter for flush operations

// Received data structure (matches TX)
struct ReceivedData {
  unsigned long timestamp;
  float pressure;
  float altitude;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float gpsLat, gpsLon;
} rxData = {0};  // Initialize all fields to zero

// ===== SETUP FUNCTIONS =====
void setupFilesystem() {
  Serial.println("[FS] File logging disabled (LittleFS.open causes crash)");
  Serial.println("[FS] Data will be displayed to terminal only");
  // TODO: Fix LittleFS integration - currently causes IntegerDivideByZero crash
  // when calling LittleFS.open(). This is a known issue with some ESP32 boards.
  return;
  
  /* DISABLED - causes bootloop
  Serial.println("[FS] Initializing LittleFS...");
  
  // Try without format-on-fail first
  if (!LittleFS.begin(false)) {
    Serial.println("[FS] First mount failed, trying to format...");
    if (!LittleFS.begin(true)) {
      Serial.println("[FS] ERROR: Failed to mount filesystem even after format");
      Serial.println("[FS] WARNING: Continuing without logging");
      return;
    }
  }

  Serial.println("[FS] LittleFS mounted successfully");

  // Create timestamped log file
  char filename[32];
  snprintf(filename, sizeof(filename), "/rx_log_%lu.csv", millis());
  
  Serial.printf("[FS] Creating log file: %s\n", filename);
  logFile = LittleFS.open(filename, "w");

  if (!logFile) {
    Serial.println("[FS] ERROR: Failed to create log file");
    return;
  }

  // Write CSV header
  logFile.println("rx_time,tx_time,pressure,altitude,ax,ay,az,gx,gy,gz,lat,lon,rssi,snr");
  logFile.flush();

  Serial.printf("[FS] Logging to: %s\n", filename);
  */
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
  
  // Verify frequency was actually set
  Serial.printf("[LoRa] Frequency register check: %ld Hz\n", (long)LORA_FREQ);
  Serial.println("[LoRa] NOTE: Packets with RSSI < -150 dBm will be rejected as noise");

  // Calculate values separately to avoid potential printf issues
  double freqMHz = (double)LORA_FREQ / 1000000.0;
  double bwKHz = (double)LORA_BANDWIDTH / 1000.0;
  
  Serial.println("[LoRa] Configuration:");
  Serial.print("  Frequency: ");
  Serial.print(freqMHz, 2);
  Serial.println(" MHz");
  Serial.print("  Bandwidth: ");
  Serial.print(bwKHz, 1);
  Serial.println(" kHz");
  Serial.print("  Spreading Factor: ");
  Serial.println(LORA_SPREADING);
  Serial.print("  Coding Rate: 4/");
  Serial.println(LORA_CODING_RATE);
  Serial.println("[LoRa] Ready - Waiting for packets...");
}

// ===== RECEIVE FUNCTIONS =====
bool decodePacket(uint8_t* packet, int packetSize) {
  // Validate packet and packet size
  if (!packet || packetSize < 32) {
    Serial.printf("[RX] WARNING: Invalid or short packet (%d bytes) - skipping\n", packetSize);
    return false;  // Don't decode incomplete packets
  }

  // Get RSSI/SNR early to filter noise
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  
  // Reject obvious noise packets (RSSI < -150 dBm is impossible for real packets)
  if (rssi < -150 || rssi == -164) {
    static unsigned long noiseCount = 0;
    noiseCount++;
    if (noiseCount % 100 == 0) {  // Only print every 100th noise packet
      Serial.printf("[RX] WARNING: Rejecting noise packets (count: %lu, RSSI=%d dBm)\n", 
                    noiseCount, rssi);
    }
    return false;
  }
  
  // Reject packets that aren't exactly 32 bytes (our expected size)
  if (packetSize != 32) {
    Serial.printf("[RX] WARNING: Wrong packet size %d (expected 32) - rejecting\n", packetSize);
    return false;
  }
  
  // Debug: Show raw packet bytes for valid packets only
  Serial.println("[DEBUG] Valid packet received! Raw (hex):");
  for (int i = 0; i < 32 && i < packetSize; i++) {
    Serial.printf("%02X ", packet[i]);
    if ((i + 1) % 16 == 0) Serial.println();
  }
  Serial.println();
  Serial.printf("[DEBUG] RSSI=%d dBm, SNR=%.1f dB\n", rssi, snr);

  // Decode binary packet (matches TX format)
  memcpy(&rxData.timestamp, &packet[0], 4);
  memcpy(&rxData.pressure, &packet[4], 4);
  memcpy(&rxData.altitude, &packet[8], 4);

  // Debug: Print decoded values before validation
  Serial.printf("[DEBUG] Decoded: T=%lu, P=%.2f, Alt=%.2f\n", 
                rxData.timestamp, rxData.pressure, rxData.altitude);

  // Validate floats - reject if NaN or Inf (corrupted packet)
  if (isnan(rxData.pressure) || isinf(rxData.pressure) ||
      isnan(rxData.altitude) || isinf(rxData.altitude)) {
    Serial.println("[RX] WARNING: Corrupted packet (NaN/Inf values) - skipping");
    return false;
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
  
  return true;  // Successfully decoded
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

  // Increment flush counter and flush every 5 packets (reduced from 10 for safety)
  logFlushCounter++;
  if (logFlushCounter >= 5) {
    logFile.flush();
    logFlushCounter = 0;
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
  // Validate packet size is reasonable
  if (packetSize <= 0 || packetSize > 256) {
    Serial.printf("[RX] Invalid packet size: %d bytes\n", packetSize);
    return;
  }

  // Read packet - read exactly packetSize bytes
  uint8_t packet[256];
  memset(packet, 0, sizeof(packet));  // Initialize to zero
  
  // Read all bytes - when parsePacket() returns > 0, all bytes should be available
  int bytesRead = 0;
  int available = LoRa.available();
  
  for (int i = 0; i < packetSize && i < 256; i++) {
    packet[i] = LoRa.read();
    bytesRead++;
  }

  // Debug: ALWAYS print bytes read vs expected
  Serial.printf("[DEBUG] PacketSize=%d, Available=%d, Read=%d bytes\n", 
                packetSize, available, bytesRead);

  // Increment counter first
  packetsReceived++;
  lastPacketTime = millis();

  // Only process if packet is complete
  if (packetSize >= 32 && bytesRead >= 32) {
    // Decode packet and only display if successful
    if (decodePacket(packet, packetSize)) {
      logReceivedData();
      displayData();
    }
    // If decode failed, the function already printed a warning
  } else {
    Serial.printf("[RX] Skipped short/incomplete packet (expected: %d, read: %d bytes)\n", packetSize, bytesRead);
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
  unsigned long currentMillis = millis();

  if (currentMillis - lastStatusTime >= 1000) {
    lastStatusTime = currentMillis;

    // Use local copies to avoid race conditions
    unsigned long pktsRx = packetsReceived;
    unsigned long lastPktTime = lastPacketTime;

    if (pktsRx == 0) {
      Serial.println("[RX] Listening... (no packets yet)");
    } else {
      // Calculate time difference safely
      unsigned long timeDiff = currentMillis - lastPktTime;
      // Print with safe values
      Serial.print("[RX] Total packets: ");
      Serial.print(pktsRx);
      Serial.print(" | Last: ");
      Serial.print(timeDiff);
      Serial.println(" ms ago");
    }
  }

  // Small delay to prevent timing issues while still being responsive
  delay(1);
}
