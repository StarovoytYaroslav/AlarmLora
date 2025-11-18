/*
  RadioLib SX126x Ping-Pong Example

  This example is intended to run on two SX126x radios,
  and send packets between the two.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>
#include <SPI.h>

// uncomment the following only on one
// of the nodes to initiate the pings
#define INITIATING_NODE

// SX1262 has the following connections:
#define LORA_CS 8
#define LORA_DIO1 14
#define LORA_RST 12
#define LORA_BUSY 13
// SPI Pins
#define LORA_SCK 9
#define LORA_MISO 11
#define LORA_MOSI 10

SPIClass loraSPI(1);

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);

// or detect the pinout automatically using RadioBoards
// https://github.com/radiolib-org/RadioBoards

// #define RADIO_BOARD_AUTO
// #include <RadioBoards.h>
// Radio radio = new RadioModule();

uint8_t targetNode = 2;
#define NUM_NODES 3
#define RX_TIMEOUT 2000  // 2-second timeout

// We will use these to manage our own timeout
bool waitingForReply = false;
uint32_t replyStartTime = 0;

// // save transmission states between loops
// int transmissionState = RADIOLIB_ERR_NONE;

// // flag to indicate transmission or reception state
// bool transmitFlag = false;

// // flag to indicate that a packet was sent or received
volatile bool operationDone = false;

// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
// void setFlag(void) {
//   // we sent or received a packet, set the flag
//   operationDone = true;
// }

// void pingNextNode(){
//   targetNode++;
//   if(targetNode > NUM_NODES)
//   {
//     targetNode = 2;
//   }
//   String packet = "";
//   packet += targetNode;
//   packet += ":Hello World!";

//   Serial.print(F("[SX1262] Sending packet to node "));
//   Serial.print(targetNode);
//   Serial.println(F("..."));
//   Serial.println(packet);

//   transmissionState = radio.startTransmit(packet);
//   transmitFlag = true;
// }

void pingNextNode() {
  targetNode++;
  if (targetNode > NUM_NODES) {
    targetNode = 2;
  }
  String packet = "";
  packet += targetNode;
  packet += ":Hello World!";

  Serial.print(F("[SX1262] Sending packet to node "));
  Serial.print(targetNode);
  Serial.println(F("..."));
  Serial.println(packet);

  // Use the SIMPLE BLOCKING transmit


  if (int transmissionState = radio.transmit(packet); transmissionState == RADIOLIB_ERR_NONE) {
    Serial.println(F("transmission finished!"));

    // NOW, start listening and set our software timer
    radio.startReceive();  // Listen forever (no timeout)
    waitingForReply = true;
    replyStartTime = millis();  // Start our watchdog

  } else {
    Serial.print(F("transmission failed, code "));
    Serial.println(transmissionState);
    // Wait and try the next node
    delay(1000);
    pingNextNode();
  }
}

void setup() {
  Serial.begin(115200);

  // --- 4. Initialize your custom SPI bus ---
  // The -1 for CS is important! It tells SPIClass
  // "RadioLib will manage the CS pin, not you."
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, -1);

  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));

  if (int state = radio.begin(); state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // pingNextNode();

  // set the function that will be called
  // when new packet is received
  // radio.setDio1Action(setFlag);

  // #if defined(INITIATING_NODE)
  // pingNextNode();
  // String packet = "";
  // packet += targetNode;       // Add the address (e.g., "2")
  // packet += ":Hello World!";  // Add a separator and the message
  // // send the first packet on this node
  // Serial.print(F("[SX1262] Sending packet to node "));
  // Serial.print(targetNode);
  // Serial.println(F(" ... "));

  // transmissionState = radio.startTransmit(packet);
  // transmitFlag = true;
  // #else
  //   // start listening for LoRa packets on this node
  //   Serial.print(F("[SX1262] Starting to listen ... "));
  //   state = radio.startReceive();
  //   if (state == RADIOLIB_ERR_NONE) {
  //     Serial.println(F("success!"));
  //   } else {
  //     Serial.print(F("failed, code "));
  //     Serial.println(state);
  //     while (true) { delay(10); }
  //   }
  // #endif
}

void loop() {
  // check if the previous operation finished
  if(++targetNode > NUM_NODES) targetNode = 2;
  
  String packet = "";
  packet += targetNode;
  packet += ":Hello World!";

  Serial.print(F("[SX1262] Sending to node "));
  Serial.print(targetNode);
  Serial.print(F("... "));

  int state = radio.transmit(packet);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("sent!"));
    
    // 3. RECEIVE (Blocking with Timeout)
    // The radio waits here for 'RX_TIMEOUT' ms
    // It automatically handles flags, standby, etc.
    String str;
    Serial.println(F("[SX1262] Waiting for reply..."));
    
    state = radio.receive(str, 0, RX_TIMEOUT);

    if (state == RADIOLIB_ERR_NONE) {
      // --- SUCCESS ---
      Serial.println(F("[SX1262] Reply received!"));
      Serial.print(F("[SX1262] Data: "));
      Serial.println(str);
      Serial.print(F("[SX1262] RSSI: "));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));
      
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      // --- TIMEOUT ---
      Serial.println(F("[SX1262] Timed out!"));
      
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // --- CRC ERROR ---
      // This means we heard something, but it was corrupted
      Serial.println(F("[SX1262] Error: CRC Mismatch (Bad signal)"));
      
    } else {
      // --- OTHER ERROR ---
      Serial.print(F("[SX1262] Error code: "));
      Serial.println(state);
    }

  } else {
    Serial.print(F("Transmit failed, code "));
    Serial.println(state);
  }

  // Wait 1 second before next ping
  delay(1000);



  // if (waitingForReply) {
  //   if (radio.available() > 0) {
  //     String str;
  //     int state = radio.readData(str);
  //     radio.standby();  // Good practice

  //     waitingForReply = false;  // Stop the timer

  //     if (state == RADIOLIB_ERR_NONE) {
  //       // packet was successfully received
  //       Serial.println(F("[SX1262] Received packet!"));

  //       // print data of the packet
  //       Serial.print(F("[SX1262] Data:\t\t"));
  //       Serial.println(str);

  //       // print RSSI (Received Signal Strength Indicator)
  //       Serial.print(F("[SX1262] RSSI:\t\t"));
  //       Serial.print(radio.getRSSI());
  //       Serial.println(F(" dBm"));

  //       // print SNR (Signal-to-Noise Ratio)
  //       Serial.print(F("[SX1262] SNR:\t\t"));
  //       Serial.print(radio.getSNR());
  //       Serial.println(F(" dB"));
  //     } else {
  //       // --- Other error ---
  //       Serial.print(F("[SX1262] Receive failed, code "));
  //       Serial.println(state);
  //     }

  //     delay(1000);
  //     pingNextNode();
  //   }

  //   if (millis() - replyStartTime > RX_TIMEOUT) {
  //     waitingForReply = false;  // Stop the timer

  //     radio.standby();  // Force the radio out of receive mode

  //     Serial.print(F("[SX1262] Timed out waiting for reply from node "));
  //     Serial.println(targetNode);

  //     // Wait and move to the next node
  //     delay(1000);
  //     pingNextNode();
  //   }
  // }


  // } else {
  //   // the previous operation was reception
  //   // print data and send another packet
  //   String str;
  //   int state = radio.readData(str);

  //   radio.standby(); // Force the radio into a clean idle state

  //   if (state == RADIOLIB_ERR_NONE) {
  //     // packet was successfully received
  //     Serial.println(F("[SX1262] Received packet!"));

  //     // print data of the packet
  //     Serial.print(F("[SX1262] Data:\t\t"));
  //     Serial.println(str);

  //     // print RSSI (Received Signal Strength Indicator)
  //     Serial.print(F("[SX1262] RSSI:\t\t"));
  //     Serial.print(radio.getRSSI());
  //     Serial.println(F(" dBm"));

  //     // print SNR (Signal-to-Noise Ratio)
  //     Serial.print(F("[SX1262] SNR:\t\t"));
  //     Serial.print(radio.getSNR());
  //     Serial.println(F(" dB"));
  //   } else if(state == RADIOLIB_ERR_RX_TIMEOUT) {
  //     // --- TIMEOUT: No reply ---
  //     Serial.print(F("[SX1262] Timed out waiting for reply from node "));
  //     Serial.println(targetNode);
  //   } else {
  //     // --- Other error ---
  //     Serial.print(F("[SX1262] Receive failed, code "));
  //     Serial.println(state);
  //   }

  //   // wait a second before transmitting again
  //   delay(1000);
  //   pingNextNode();
  // }
}
