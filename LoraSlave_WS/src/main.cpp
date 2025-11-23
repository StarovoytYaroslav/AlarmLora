#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#define NODE_ID 2

SPIClass loraSPI(1);

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);

int transmissionState = RADIOLIB_ERR_NONE;

bool transmitFlag = false;

volatile bool operationDone = false;

ICACHE_RAM_ATTR

void setFlag(void) {
  // we sent or received a packet, set the flag
  operationDone = true;
}


void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  // --- 4. Initialize your custom SPI bus ---
  // The -1 for CS is important! It tells SPIClass
  // "RadioLib will manage the CS pin, not you."
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, -1);

  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin(868.0, 125.0, 9, 8, 0x12, 22);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // set the function that will be called
  // when new packet is received
  radio.setDio1Action(setFlag);

  // start listening for LoRa packets on this node
  Serial.print(F("[SX1262] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

}

void loop() {
  // check if the previous operation finished
  if (operationDone) {
    // reset flag
    operationDone = false;

    if (transmitFlag) {
      // the previous operation was transmission, listen for response
      // print the result
      if (transmissionState == RADIOLIB_ERR_NONE) {
        // packet was successfully sent
        Serial.println(F("transmission finished!"));

      } else {
        Serial.print(F("failed, code "));
        Serial.println(transmissionState);
      }

      // listen for response
      radio.startReceive();
      transmitFlag = false;

    } else {
      // the previous operation was reception
      // print data and send another packet
      String str;
      int state = radio.readData(str);

      if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        Serial.println(F("[SX1262] Received packet!"));

        // print data of the packet
        Serial.print(F("[SX1262] Data:\t\t"));
        Serial.println(str);

        // --- MODIFIED SECTION ---
        // Check the address
        // Find the "address" part (before the ":")
        int separator = str.indexOf(':');
        int destinationID = str.substring(0, separator).toInt();

        // Check if the packet is for this node
        if (destinationID == NODE_ID) {
          Serial.println(F("[SX1262] Packet is for me!"));
          Serial.print(F("[SX1262] RSSI:\t\t"));
          Serial.print(radio.getRSSI());
          Serial.println(F(" dBm"));

          // print RSSI (Received Signal Strength Indicator)
          Serial.print(F("[SX1262] RSSI:\t\t"));
          Serial.print(radio.getRSSI());
          Serial.println(F(" dBm"));

          // print SNR (Signal-to-Noise Ratio)
          Serial.print(F("[SX1262] SNR:\t\t"));
          Serial.print(radio.getSNR());
          Serial.println(F(" dB"));

          // wait a second before transmitting again
          delay(100);

          // Send a reply
          // (Replies could also be addressed back to node 1)
          String reply = "1:Got it!";
          Serial.print(F("[SX1262] Sending reply ... "));
          digitalWrite(LED_PIN, LOW); // LED On
          transmissionState = radio.startTransmit(reply);
          digitalWrite(LED_PIN, HIGH); // LED On
          transmitFlag = true;
        } else {
          // Packet was not for this node, stay silent
          Serial.print(F("[SX1262] Packet was for node "));
          Serial.print(destinationID);
          Serial.println(F(", ignoring."));

          // Go back to listening without replying
          radio.startReceive();
          transmitFlag = false;
        }
      }
    }
  }
}
