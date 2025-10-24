// Minimal LoRa RX Test
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS 25
#define LORA_RST 14
#define LORA_DIO0 4
#define LORA_FREQ 433E6

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== LoRa RX Test ===");
  
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init FAILED!");
    while (1);
  }
  
  Serial.println("LoRa OK! Listening...");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    
    Serial.printf("*** RECEIVED: '%s' | Size: %d | RSSI: %d dBm | SNR: %.1f dB ***\n", 
                  received.c_str(), packetSize, rssi, snr);
  }
  
  delay(10);
}

