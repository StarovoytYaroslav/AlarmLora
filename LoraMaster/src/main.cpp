#include <Arduino.h>
#include "main.h"
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <heltec-eink-modules.h>
#include "html.h"
#include "display.h"

SPIClass loraSPI(1);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Display display;



struct NodeData
{
  int id;
  String message;
  int rssi;
  bool updated;
};
NodeData lastKnownData;
SemaphoreHandle_t dataMutex;

#define NUM_NODES 3
#define RX_TIMEOUT 2000

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    // User just opened the page! Check if we have old data.
    if (xSemaphoreTake(dataMutex, 100))
    {
      if (lastKnownData.id != 0)
      { // If we have data
        String msg = String(lastKnownData.id) + "," + lastKnownData.message + "," + String(lastKnownData.rssi);
        client->text(msg); // Send ONLY to this new client
      }
      xSemaphoreGive(dataMutex);
    }
  }
}

void taskLoRa(void *pvParameters);

void setup()
{
  display.setupDisplay();

  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

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
  display.printWiFiOnDisplay(WiFi.localIP());

  // 4. Setup Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html", index_html); });

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              request->send(204); // "No Content" - tells browser to stop waiting
            });

  ws.onEvent(onEvent);

  server.addHandler(&ws);

  server.begin();

  // 5. Launch LoRa Task on Core 1
  xTaskCreatePinnedToCore(taskLoRa, "LoRaTask", 8192, NULL, 1, NULL, 1);
  ws.onEvent(onEvent);
}

void taskLoRa(void *pvParameters)
{
  // Init SPI
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, -1);

  // Init Radio
  int state = radio.begin(868.0, 125.0, 9, 7, 0x12, 22);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.printf("LoRa Failed: %d\n", state);
    vTaskDelete(NULL);
  }

  uint8_t targetNode = 2;

  for (;;)
  {
    // 1. Rotate Target
    targetNode++;
    if (targetNode > NUM_NODES)
      targetNode = 2;

    String packet = String(targetNode) + ":Ping";
    Serial.printf("Pinging %d... ", targetNode);

    // 2. Transmit
    digitalWrite(LED_PIN, HIGH); // LED On
    int state = radio.transmit(packet);
    digitalWrite(LED_PIN, LOW); // LED Off

    if (state == RADIOLIB_ERR_NONE)
    {
      // 3. Receive
      String str;
      state = radio.receive(str, 0, RX_TIMEOUT);

      if (state == RADIOLIB_ERR_NONE)
      {
        Serial.printf("Reply: %s\n", str.c_str());

        // --- A. LIVE PUSH (WebSocket) ---
        // Send immediately to connected browsers
        String wsData = String(targetNode) + "," + str + "," + String(radio.getRSSI());
        ws.textAll(wsData);
        // --- B. SAVE STATE (Mutex) ---
        // Save for any NEW clients that connect later
        if (xSemaphoreTake(dataMutex, 100))
        {
          lastKnownData.id = targetNode;
          lastKnownData.message = str;
          lastKnownData.rssi = radio.getRSSI();
          xSemaphoreGive(dataMutex);
        }
      }
      else if (state == RADIOLIB_ERR_RX_TIMEOUT)
      {
        Serial.println("Timeout");
        // Optional: Send "Timeout" status to Web via WebSocket?
        // ws.textAll(String(targetNode) + ",Timeout,0");
      }
    }
    else
    {
      Serial.printf("TX Err: %d\n", state);
    }

    // 4. Wait before next ping
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loop()
{
  ws.cleanupClients();
  vTaskDelay(1000);
}
