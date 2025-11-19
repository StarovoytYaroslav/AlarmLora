#include <Arduino.h>
#include <SPI.h> // <--- THIS IS THE KEY. This forces PlatformIO to link the library.
#include <RadioLib.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <heltec-eink-modules.h>
#include "Fonts/FreeSans9pt7b.h"
#include "Fonts/FreeSansBold9pt7b.h"

EInkDisplay_WirelessPaperV1_1 display;
SPIClass loraSPI(1);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);
AsyncWebServer server(80);

struct NodeData {
  int id;
  String message;
  int rssi;
  bool updated; 
};
NodeData latestData;
SemaphoreHandle_t dataMutex;

#define NUM_NODES 3
#define RX_TIMEOUT 2000

void taskLoRa(void *pvParameters);



void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for USB connection
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Turn On (3.3V)

  Serial.println("\n\n--------------------------------");
  Serial.printf("Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("Cores: %d\n", ESP.getChipCores());
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.println("--------------------------------");

  display.clearMemory(); // Start a new drawing
  display.update();
  display.landscape();
  display.setFont(&FreeSans9pt7b);
  display.setCursor(1, 24); // Text Cursor - x:5 y:20
  display.fastmodeOn();
  display.println("first line");
  display.update();
  delay(4000);
  display.println("second line");
  display.update();
  delay(4000);
  display.println("third line");
  display.update();
  delay(4000);
  display.println("fourth line");
  display.update();
  delay(4000);
  display.println("fifth line");
  display.update();
  delay(4000);
  display.clearMemory(); // Start a new drawing
  display.update();
  display.setCursor(1, 24); // Text Cursor - x:5 y:20
  display.println("sixth line");
  display.update();
  delay(4000);
  display.println("seventh line");
  display.update();
  digitalWrite(LED_PIN, LOW); // Turn On (3.3V)
  delay(4000);
}

void loop()
{
  // put your main code here, to run repeatedly:
}
