#include <Arduino.h>
#include <SPI.h>
#include <SX126x.h>

#include "PacketBuilder.hpp" // Ensure this file is in the same folder
#include "transport.h"       // The struct above
// —————————————————————————————————————————————————————————————
// 1. PINS & CONFIG
// —————————————————————————————————————————————————————————————
const int8_t NSS_PIN = 1;
const int8_t RESET_PIN = 4;
const int8_t BUSY_PIN = 5;
const int8_t IRQ_PIN = 15;
const int8_t TXEN_PIN = -1;
const int8_t RXEN_PIN = -1;

const uint8_t MY_NODE_ID = 1;
uint8_t msgCounter = 0;

// —————————————————————————————————————————————————————————————
// 2. GLOBALS
// —————————————————————————————————————————————————————————————
SX126x LoRa;
bool LoraInitialized = true;
uint8_t currentPacketID = 0;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000;

// Create Builder Instance (Sender)
PacketBuilder payload(50); // Max 50 bytes for data

// Buffer for Receiving (Max 64 bytes)
uint8_t rxBuffer[64];

// Function Prototypes
void sendLoRaData(uint8_t targetNode);
void readLoraPacket();

// —————————————————————————————————————————————————————————————
// 3. SETUP
// —————————————————————————————————————————————————————————————
void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println(F("--- LLCC68 PACKET BUILDER/PARSER DEMO ---"));

    // SPI SETUP
    SPI.setRX(0);
    SPI.setSCK(2);
    SPI.setTX(3);
    SPI.begin();

    Serial.print(F("Initializing LoRa... "));
    if (!LoRa.begin(NSS_PIN, RESET_PIN, BUSY_PIN, IRQ_PIN, TXEN_PIN, RXEN_PIN)) {
        Serial.println(F("FAILED!"));
        LoraInitialized = false;
    } else {
        Serial.println(F("OK"));

        // LoRa Config
        LoRa.setTxPower(14, SX126X_TX_POWER_SX1262);
        LoRa.setFrequency(868100000);
        LoRa.setLoRaModulation(9, 125000, 5);
        LoRa.setLoRaPacket(SX126X_HEADER_EXPLICIT, 8, 255, true, false);
        LoRa.setSyncWord(0x12);

        // Start Listening
        LoRa.request(SX126X_RX_CONTINUOUS);
    }
}

// —————————————————————————————————————————————————————————————
// 4. LOOP
// —————————————————————————————————————————————————————————————
void loop() {
    if (!LoraInitialized) return;

    // Check for incoming data
   // readLoraPacket();

    // Send data periodically
    if (millis() - lastSendTime >= sendInterval) {
        lastSendTime = millis();
        sendLoRaData(100);
    }
}

// —————————————————————————————————————————————————————————————
// 5. SENDING (Builder)
// —————————————————————————————————————————————————————————————
void sendLoRaData(uint8_t targetNode) {
    Serial.println(F("\n[TX] Building Packet..."));

    // 1. Reset Builder
    payload.reset();
    // Add your data
    payload.addTemp(1, 24.5);     // Channel 1, 24.5 C
    payload.addHumidity(2, 60.5); // Channel 2, 60.5 %
    payload.addBattery(3, 3.7);   // Channel 3, 3.7 V

    // Define Header
    TransportHeader header;
    header.netId = MY_NET_ID;
    header.senderId = MY_NODE_ID;
    header.targetId = targetNode;
    header.packetId = msgCounter++;

    // Create Final Buffer (Header Size + Payload Size)
    int totalSize = sizeof(TransportHeader) + payload.getSize();
    uint8_t txBuffer[totalSize];

    // Copy Header to start
    memcpy(txBuffer, &header, sizeof(TransportHeader));

    // Copy Payload after Header
    memcpy(txBuffer + sizeof(TransportHeader), payload.getBuffer(), payload.getSize());

    // 3. Transmit
    // Ensure we are in Standby before writing to buffer
    LoRa.standby(SX126X_STANDBY_RC);
    LoRa.beginPacket();
    // B. Write Binary Payload
    LoRa.write(txBuffer, totalSize);

    LoRa.endPacket();
    LoRa.wait(); // Wait for TX done

    Serial.print(F("[TX] Sent "));
    Serial.print(totalSize);
    Serial.println(F(" bytes payload."));

    // 4. Return to RX Mode
    LoRa.request(SX126X_RX_CONTINUOUS);
}

// —————————————————————————————————————————————————————————————
// 6. RECEIVING (WITH ECHO CHECK)
// —————————————————————————————————————————————————————————————
//void readLoraPacket() {
//    if (!LoRa.available()) return;
//
//    // 1. FAST HEADER CHECK <111>
//    int b = LoRa.read();
//    if (b != '<') return;
//
//    String idString = "";
//    while (LoRa.available()) {
//        char c = (char)LoRa.read();
//        if (c == '>') break;
//        idString += c;
//    }
//    // Only process packets meant for "111"
//    if (idString != String(myID)) return;
//
//    // 2. READ BINARY DATA
//    int bytesRead = 0;
//    while (LoRa.available() && bytesRead < 64) {
//        rxBuffer[bytesRead++] = LoRa.read();
//    }
//
//    // 3. START PARSING
//    PacketParser parser(rxBuffer, bytesRead);
//
//    // --- STEP A: ECHO CANCELLATION (First item MUST be Packet ID) ---
//    if (parser.next()) {
//        if (parser.getType() == PacketDataType::TYPE_COUNTER) {
//            uint8_t rxID = parser.getByte();
//
//            // *** ECHO CHECK ***
//            // If the received ID matches the one WE just sent, it's a reflection.
//            if (parser.isEcho(currentPacketID, rxID)) {
//                Serial.print(F("[!] Reflection Detected (ID: "));
//                Serial.print(rxID);
//                Serial.println(F("). Ignoring."));
//                return; // ABORT -> Don't process commands from our own echo
//            }
//
//            // If ID is DIFFERENT, it is a valid command from Node-RED
//            Serial.print(F("[OK] Command Received (ID: "));
//            Serial.print(rxID);
//            Serial.println(F(")"));
//
//        } else {
//            // Optional warning if packet format is bad
//            // Serial.println(F("Warning: Packet did not start with ID"));
//        }
//    } else {
//        return; // Empty packet
//    }
//
//    // --- STEP B: PROCESS COMMANDS ---
//    // We use do-while because next() was already called once above.
//    do {
//        PacketDataType type = parser.getType();
//
//        // Skip the ID (Counter) as we handled it
//        if (type == PacketDataType::TYPE_COUNTER) continue;
//
//        if (type == PacketDataType::TYPE_STATUS) {
//            uint8_t status = parser.getByte();
//            switch (status) {
//                case 1:
//                    Serial.println(F("ACTION: PUMP ON"));
//                    // digitalWrite(PUMP_PIN, HIGH);
//                    break;
//                case 0:
//                    Serial.println(F("ACTION: PUMP OFF"));
//                    // digitalWrite(PUMP_PIN, LOW);
//                    break;
//                case 3: // Status 3: Set Threshold (expects Temp value next)
//                    if (parser.next() && parser.getType() == PacketDataType::TYPE_TEMP) {
//                        float val = parser.getTemp();
//                        Serial.print(F("ACTION: Set Threshold -> "));
//                        Serial.println(val);
//                    }
//                    break;
//            }
//        }
//    } while (parser.next());
//}