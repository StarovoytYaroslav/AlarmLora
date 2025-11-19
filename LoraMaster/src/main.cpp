#include <Arduino.h>
#include <SPI.h>       // <--- THIS IS THE KEY. This forces PlatformIO to link the library.
#include <RadioLib.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

void setup() {
  Serial.begin(115200);
  while(!Serial); // Wait for USB connection
  
  Serial.println("\n\n--------------------------------");
  Serial.printf("Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("Cores: %d\n", ESP.getChipCores());
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.println("--------------------------------");
}

void loop() {
  // put your main code here, to run repeatedly:
}

