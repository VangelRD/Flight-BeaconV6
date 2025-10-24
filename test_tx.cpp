// Minimal LoRa TX Test
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
  Serial.println("\n=== LoRa TX Test ===");
  
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init FAILED!");
    while (1);
  }
  
  LoRa.setTxPower(20);
  Serial.println("LoRa OK! Sending packets...");
}

int counter = 0;
void loop() {
  Serial.printf("Sending: PING %d\n", counter);
  
  LoRa.beginPacket();
  LoRa.print("PING ");
  LoRa.print(counter);
  LoRa.endPacket();
  
  counter++;
  delay(2000);  // Send every 2 seconds
}

