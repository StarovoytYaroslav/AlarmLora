#include <Arduino.h>
#include "main.h"
#include <SPI.h>
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
AsyncWebSocket ws("/ws");

struct NodeData
{
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
  display.clearMemory(); // Start a new drawing
  display.update();
  display.landscape();
  display.setFont(&FreeSans9pt7b);
  display.setCursor(1, 24); // Text Cursor - x:5 y:20
  display.fastmodeOn();

  Serial.begin(115200);

  // 1. Wait for Serial (Optional, useful for debugging startup)
  while (!Serial)
    Serial.println("\n--- Master Node Starting ---");

  // 2. Create Mutex
  dataMutex = xSemaphoreCreateMutex();

  // 3. Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, pass); // <--- CHANGE THIS
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("Web Server IP: ");
  Serial.println(WiFi.localIP());
  display.println("WiFi Connected!");
  display.print("Web Server IP: ");
  display.println(WiFi.localIP());
  display.update();

  // 4. Setup Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      String html = "<html><head><meta http-equiv='refresh' content='2'></head><body>";
      html += "<h1>LoRa Map Master</h1>";
      
      if(xSemaphoreTake(dataMutex, 100)) {
         html += "<h3>Last Signal:</h3>";
         html += "<p><b>Node ID:</b> " + String(latestData.id) + "</p>";
         html += "<p><b>Message:</b> " + latestData.message + "</p>";
         html += "<p><b>RSSI:</b> " + String(latestData.rssi) + " dBm</p>";
         xSemaphoreGive(dataMutex);
      } else {
        html += "<p>Loading data...</p>";
      }
      html += "</body></html>";
      request->send(200, "text/html", html); });
  server.begin();

  // 5. Launch LoRa Task on Core 1
  xTaskCreatePinnedToCore(taskLoRa, "LoRaTask", 8192, NULL, 1, NULL, 1);
}

void taskLoRa(void *pvParameters)
{
  // 1. Init SPI
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, -1);

  // 2. Init Radio (EU868)
  Serial.print("[LoRa] Initializing... ");
  int state = radio.begin(868.0, 125.0, 9, 8, 0x12, 22);

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println("Success!");
  }
  else
  {
    Serial.printf("Failed, code %d\n", state);
    vTaskDelete(NULL); // Stop task if radio fails
  }

  uint8_t targetNode = 2;

  for (;;)
  {
    // Loop through nodes 2 and 3
    targetNode++;
    if (targetNode > NUM_NODES)
      targetNode = 2;

    String packet = String(targetNode) + ":Ping";
    Serial.printf("[LoRa] Pinging Node %d... ", targetNode);

    // Transmit
    int state = radio.transmit(packet);

    if (state == RADIOLIB_ERR_NONE)
    {
      // Receive Window
      String str;
      state = radio.receive(str, 0, RX_TIMEOUT);

      if (state == RADIOLIB_ERR_NONE)
      {
        Serial.printf("Reply: %s (RSSI: %f)\n", str.c_str(), radio.getRSSI());

        // Save to Shared Data
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
          latestData.id = targetNode;
          latestData.message = str;
          latestData.rssi = radio.getRSSI();
          latestData.updated = true;
          xSemaphoreGive(dataMutex);
        }
      }
      else if (state == RADIOLIB_ERR_RX_TIMEOUT)
      {
        Serial.println("Timeout.");
      }
      else
      {
        Serial.printf("RX Error: %d\n", state);
      }
    }
    else
    {
      Serial.printf("TX Failed: %d\n", state);
    }

    // Wait 1 second before pinging the next node
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loop()
{
  vTaskDelay(1000);
}
