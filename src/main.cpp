// CONTROLLER BAZIN APA 50000L

// ——————————————————————————————————
// 1) INCLUDES & DEFINES
// ——————————————————————————————————

// pin assignments
#define DS18B20_PIN 7
#define LED_PIN 15
#define BOARD_LED_PIN 16

#define PUMP1_PIN 12
#define PUMP2_PIN 11

#define BTN_AUTO 10
#define BTN_PUMP1 8
#define BTN_PUMP2 9

// water level inputs 0..2
#define WL_IN_0 13
#define WL_IN_1 14
#define WL_IN_2 26

// wrap LoRa sender ID in “<ID>”
#define LORA_WRITE_NUM_WRAPPED(id)                             \
    {                                                          \
        LoRa.write('<');                                       \
        char _buf[10];                                         \
        itoa(id, _buf, 10);                                    \
        for (int _i = 0; _buf[_i]; _i++) LoRa.write(_buf[_i]); \
        LoRa.write('>');                                       \
    }

// bit-flags
#define BIT_ON 1
#define BIT_OFF 0

#include <Arduino.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <OneWire.h>
#include <SX126x.h>
#include <TaskScheduler.h>
#include <math.h> // for log()

#include "IndicatorControll.hpp"
#include "PumpControl.hpp"
#include "StorageConfig.hpp"
#include "WaterLevel.hpp"
#include "hardware/watchdog.h"
#include "structures.h"

// LoRa Pin Configuration
const int8_t NSS_PIN = 1;
const int8_t RESET_PIN = 4;
const int8_t BUSY_PIN = 5;
const int8_t IRQ_PIN = 6;
const int8_t TXEN_PIN = -1;
const int8_t RXEN_PIN = -1;

// node IDs
#define myID 111
#define secondNodeID 112
#define GatewayID 100

// Task intervals (ms)
#define getSensorDataInterval 5000
#define printDataInterval 6000
#define sendLoRaDataInterval 6000

// ——————————————————————————————————
// 2) FORWARD DECLARATIONS
// ——————————————————————————————————
void setup();
void loop();

void ReadSerialData();
void processSerialInput(const String &data);
void resetMySerial();

void ReadMainSerialData();
void processMainSerialInput(const String &cmd);

void getSensorData();
void printSensorData();

void sendLoRaData();
void readLoraPacket();
void processReceivedData();
void resetLoRaRequest();

void printByteBinary(uint8_t val);
void printDataPacket(const DataPacket &packet);

// ——————————————————————————————————
// 3) GLOBAL OBJECTS & STATE
// ——————————————————————————————————
SX126x LoRa;
bool LoraInitialized = true;

OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

// up to 5 DS18B20 sensors
DeviceAddress sensorAddresses[5];
int ds18b20Count = 0;

DataPacket LocalDataStruct;
DataPacket receivedData;
bool do_processReceivedData = false;

bool dataWasMeasured = false;

String inputString = "";
String mainInputString = "";
bool mainInputComplete = false;
unsigned long lastSerialReceiveTime = 0;
const unsigned long serialTimeout = 5000;

uint8_t currentPacketID = 1;

// Scheduler + tasks
Scheduler runner;
Task T_get_sensor_data(getSensorDataInterval, TASK_FOREVER, &getSensorData);
Task T_print_sensor_data(printDataInterval, TASK_FOREVER, &printSensorData);
Task T_send_lora_data(sendLoRaDataInterval, TASK_FOREVER, &sendLoRaData);

// ——————————————————————————————————
// 4) ARDUINO LIFECYCLE
// ——————————————————————————————————
void setup() {
    // USB-serial
    Serial.begin(115200);
    // ——— wait for up to 5 s for Serial to connect ———
    const unsigned long SERIAL_TIMEOUT_MS = 2000;
    unsigned long start = millis();
    while (!Serial && (millis() - start < SERIAL_TIMEOUT_MS)) {
        delay(10); // avoid busy-spin
    }

    if (Serial) {
        Serial.println(F("Serial connected."));
    } else {
        // timed-out, serial never connected
        // you can blink an LED here or just carry on silently
    }

    WaterLevel::begin(WL_IN_0, WL_IN_1, WL_IN_2); // bottom→pin2, middle→pin3, top→pin4

    // 1) initialize & load stored status + threshold
    StorageConfig::begin();
    LocalDataStruct.status = StorageConfig::loadStatus();

    // initialize pumps & buttons
    PumpControl::begin();
    PumpControl::updatelocalstates(LocalDataStruct);
    // initialize LED indicator
    IndicatorControl::begin();

    // SPI pins on RP2040
    SPI.setRX(0);
    SPI.setCS(1);
    SPI.setSCK(2);
    SPI.setTX(3);
    SPI.begin();

    // initialize LoRa module
    Serial.println("Initializing LoRa module...");
    if (!LoRa.begin(NSS_PIN, RESET_PIN, BUSY_PIN, IRQ_PIN, TXEN_PIN, RXEN_PIN)) {
        Serial.println("Error: Unable to initialize LoRa module.");
        LoraInitialized = false;
    }

    ds18b20.begin();
    if (LoraInitialized) {
        // configure LoRa TX/RX
      // LoRa.setDio3TcxoCtrl(SX126X_DIO3_OUTPUT_1_8, SX126X_TCXO_DELAY_10);
        LoRa.setRegulator(SX126X_REGULATOR_DC_DC); 
        LoRa.setCurrentProtection(140); 
        LoRa.setFrequency(868100000);
        LoRa.setTxPower(14, SX126X_TX_POWER_SX1262);
        LoRa.setLoRaModulation(9, 125000, 5);
        LoRa.setLoRaPacket(SX126X_HEADER_EXPLICIT, 8, 255, true, false);
        LoRa.setSyncWord(0x12);
        LoRa.request(SX126X_RX_CONTINUOUS);
        Serial.println("-- LORA TRANSMITTER READY --");
    } else {
        Serial.println("LoRa module not initialized, skipping LoRa setup.");
    }

    // scheduler tasks
    runner.init();
    runner.addTask(T_get_sensor_data);
    runner.addTask(T_print_sensor_data);
    runner.addTask(T_send_lora_data);
    T_get_sensor_data.enableDelayed(0);
    T_print_sensor_data.enable();
    T_send_lora_data.enable();
    Serial.println("Tasks enabled");
    IndicatorControl::Running(); // start in “running” (blinking) mode
}

void loop() {
    runner.execute(); // run scheduled tasks

    ReadMainSerialData();                 // read USB-serial commands
    readLoraPacket();                     // handle inbound LoRa
    PumpControl::update(LocalDataStruct); // update pump logic + packet.status
    // push updated threshold into indicator each cycle
    // IndicatorControl::setThreshold(PumpControl::getWaterLevelThreshold());
    IndicatorControl::update(LocalDataStruct); // LED indicator
    resetLoRaRequest();                        // re-enter RX if needed
    // only write back if either field actually changed
    StorageConfig::saveIfChanged(LocalDataStruct.status);
}

// ——————————————————————————————————
// 11) FUNCTION TO READ WATER LEVEL INPUT AND PROCESS WATER LEVEL
// ——————————————————————————————————

// ——————————————————————————————————
// 6) USB-SERIAL COMMANDS READ/PROCESSING
// ——————————————————————————————————
void ReadMainSerialData() {
    while (Serial.available()) {
        char ch = (char)Serial.read();
        if (ch == '\r' || ch == '\n') {
            if (mainInputString.length()) {
                processMainSerialInput(mainInputString);
                mainInputString = "";
            }
        } else {
            mainInputString += ch;
        }
    }
}

void processMainSerialInput(const String &cmd) {
    String s = cmd;
    s.trim();
    s.toLowerCase();
    if (s == "s") {
        // print raw local data packet print leading zeros
        Serial.print("LocalDataStruct: ");
        for (size_t i = 0; i < sizeof(LocalDataStruct); ++i) {
            if (i > 0) Serial.print(' ');
            if (i < sizeof(LocalDataStruct) - 1) {
                Serial.print(((uint8_t *)&LocalDataStruct)[i], HEX);
            } else {
                Serial.print(((uint8_t *)&LocalDataStruct)[i], DEC);
            }
        }
        // Serial.printf("Pump1:%s Pump2:%s Auto:%s\n",
        //               PumpControl::getPump1State() ? "ON" : "OFF",
        //               PumpControl::getPump2State() ? "ON" : "OFF",
        //               PumpControl::isAutoMode() ? "EN" : "OFF");
        printDataPacket(LocalDataStruct);
    } else if (s == "d") {
        printSensorData();
    } else if (s == "r") {
        Serial.println("→ Resetting Pico now!");
        delay(50);
        watchdog_reboot(0, 0, 0);
    } else if (s == "p11") {
        PumpControl::setPump1(true);
    } else if (s == "p10") {
        PumpControl::setPump1(false);
    } else if (s == "p21") {
        PumpControl::setPump2(true);
    } else if (s == "p20") {
        PumpControl::setPump2(false);
    } else if (s == "auto on") {
        PumpControl::setAutoMode(true);
    } else if (s == "auto off") {
        PumpControl::setAutoMode(false);
    } else if (s == "auto toggle") {
        PumpControl::toggleAutoMode();
    } else {
        Serial.print("Unknown cmd: ");
        Serial.println(cmd);
    }
}

// ——————————————————————————————————
// 7) SENSOR TASKS
// ——————————————————————————————————

    // 2) stash the raw byte (bits 0–1 = level, bit 2 = abnormal)
    uint8_t waterStatus = 0;

    // 3) split it out if you want:
    uint8_t lvl = 0;        // 0..3
    bool isAbnorm = false; // true/false

void getSensorData() {
    // 1) refresh the sensor state
    WaterLevel::update();

    // 2) stash the raw byte (bits 0–1 = level, bit 2 = abnormal)
    waterStatus = WaterLevel::waterState;

    // 3) split it out if you want:
    lvl = waterStatus & WaterLevel::LEVEL_MASK;        // 0..3
    isAbnorm = (waterStatus & WaterLevel::ABNORMAL_FLAG); // true/false

    LocalDataStruct.waterLevel = lvl;
    // store the isAbnorm flag in the status and bit ERROR_STATE
    if (isAbnorm) {
        LocalDataStruct.status |= PacketFlags::ERROR_STATE;
    } else {
        LocalDataStruct.status &= ~PacketFlags::ERROR_STATE;
    }

    Serial.println();

    // 1) no devices on the bus?
    if (ds18b20.getDeviceCount() == 0) {
        ds18b20.begin(); // re-scan & re-init the bus
        LocalDataStruct.temperature = 255;
        dataWasMeasured = true;
        return;
    }

    // 2) ask for a new reading
    ds18b20.requestTemperatures();
    float temp = ds18b20.getTempCByIndex(0);

    // 3) check for errors (disconnected, NaN or out-of-range)
    if (temp == DEVICE_DISCONNECTED_C || isnan(temp) || temp < -55.0f || temp > 125.0f) {
        ds18b20.begin(); // reset the bus
        LocalDataStruct.temperature = DEVICE_DISCONNECTED_C;
    } else {
        LocalDataStruct.temperature = temp;
    }

    dataWasMeasured = true;
}

void printSensorData() {
    // ——— 1) Human-readable values ———
    Serial.print(F("T="));
    Serial.print(LocalDataStruct.temperature, 2);
    Serial.print(F(" WL="));
    Serial.println(LocalDataStruct.waterLevel);

    // ——— 2) Raw struct bytes as hex ———
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&LocalDataStruct);
    size_t n = sizeof(LocalDataStruct);

    Serial.print(F("LocalDataStruct:"));
    for (size_t i = 0; i < n; ++i) {
        Serial.print(' ');
        // two-digit hex, leading zero if needed
        if (p[i] < 0x10) Serial.print('0');
        Serial.print(p[i], HEX);
    }
    Serial.println();

    // ——— 3) Water-level description ———
    Serial.print(F("Level: "));
    switch (WaterLevel::level()) {
      case WaterLevel::LEVEL_EMPTY: Serial.println(F("Empty"));   break;
      case WaterLevel::LEVEL_LOW:   Serial.println(F("Low"));     break;
      case WaterLevel::LEVEL_MID:   Serial.println(F("Mid"));     break;
      case WaterLevel::LEVEL_FULL:  Serial.println(F("Full"));    break;
      default:                      Serial.println(F("Unknown")); break;
    }

    // ——— 4) Abnormal flag ———
    if (WaterLevel::abnormal()) {
        Serial.println(F("⚠️ Abnormal!"));
    }

    // ——— 5) Status bits ———
    Serial.print(F("Status: "));
    printByteBinary(LocalDataStruct.status);
    Serial.println();
}


// ——————————————————————————————————
// 8) LoRa SEND / RECEIVE
// ——————————————————————————————————
void sendLoRaData() {

    Serial.println("Sending LoRa packet...Thread");
    currentPacketID = (currentPacketID % 255) + 1;
    LocalDataStruct.packetID = currentPacketID;
    LocalDataStruct.receiverID = GatewayID;
    //for test 
    LocalDataStruct.status &= ~(PacketFlags::OTHER_STATION_PUMP);


    //generate CRC8
    LocalDataStruct.updateCRC8();


    if (!LoraInitialized) return;
    if (LoRa.available() || !dataWasMeasured) return;
    IndicatorControl::Transmitting();
    LoRa.beginPacket();
    LORA_WRITE_NUM_WRAPPED(myID); // Write sender ID
    LoRa.write((uint8_t *)&LocalDataStruct, sizeof(LocalDataStruct));
    LoRa.endPacket();
    LoRa.wait();
    LoRa.purge();
    LoRa.request(SX126X_RX_CONTINUOUS);
    dataWasMeasured = false;
}

void readLoraPacket() {
    if (!LoraInitialized) return;
    if (!LoRa.available()) return;
    IndicatorControl::Receiving();
    Serial.println("Received LoRa packet...");
    String idString = "";
    char ch;
    bool headerStarted = false;
    unsigned long start = millis();
    while (millis() - start < 1000) {
        if (!LoRa.available()) continue;
        ch = LoRa.read();
        Serial.print(ch, HEX);
        if (ch == '<') {
            headerStarted = true;
            idString = "";
        } else if (ch == '>' && headerStarted) {
            break;
        } else if (headerStarted) {
            idString += ch;
        }
    }

    int senderID = idString.toInt();
    Serial.print("Parsed sender ID: ");
    Serial.println(senderID);
    if (senderID == myID) {
        Serial.println("→ own packet, ignoring");
        return;
    } else if (senderID == secondNodeID) {
        Serial.println("→ from second node");
    } else if (senderID == GatewayID) {
        do_processReceivedData = true;
        Serial.println("→ from gateway");
    }

    Serial.print("Bytes left: ");
    Serial.println(LoRa.available());
    if (LoRa.available() >= sizeof(DataPacket)) {
        uint8_t buf[sizeof(DataPacket)];
        for (size_t i = 0; i < sizeof(buf); ++i) buf[i] = LoRa.read();

        // print raw bytes
        for (auto b : buf) {
            if (b < 0x10) Serial.print('0');
            Serial.print(b, HEX);
            Serial.print(' ');
        }
        Serial.println();

        memcpy(&receivedData, buf, sizeof(receivedData));
        Serial.print("RSSI: ");
        Serial.println(LoRa.packetRssi());
        Serial.print("SNR:  ");
        Serial.println(LoRa.snr());
        printDataPacket(receivedData);
        if (receivedData.validateCRC8()) {
            Serial.println("CRC OK");
            processReceivedData();

        } else {
            Serial.println("CRC ERROR!");
        }
    } else {
        Serial.println("Not enough data for full DataPacket");
    }
}

void processReceivedData() {
    // if(do_processReceivedData){
    //   do_processReceivedData = false;
    // Print all the status bits
    Serial.println("Status Bits:");
    Serial.print("AUTO_MODE: ");
    Serial.println(receivedData.status & PacketFlags::AUTO_MODE ? "ON" : "OFF");
    Serial.print("PUMP1: ");
    Serial.println(receivedData.status & PacketFlags::PUMP1 ? "ON" : "OFF");
    Serial.print("PUMP2: ");
    Serial.println(receivedData.status & PacketFlags::PUMP2 ? "ON" : "OFF");
    Serial.print("STOP_FILL: ");
    Serial.println(receivedData.status & PacketFlags::OTHER_STATION_PUMP ? "ON" : "OFF");
    Serial.print("START_FILL: ");
    Serial.println(receivedData.status & PacketFlags::OTHER_STATION_STARTSTOP_FILL ? "ON" : "OFF");
    Serial.print("P1_COM: ");
    Serial.println(receivedData.status & PacketFlags::P1_COM ? "ON" : "OFF");
    Serial.print("P2_COM: ");
    Serial.println(receivedData.status & PacketFlags::P2_COM ? "ON" : "OFF");
    Serial.print("OVERRIDE_EN: ");
    Serial.println(receivedData.status & PacketFlags::OVERRIDE_EN ? "ON" : "OFF");
    Serial.print("ERROR_STATE: ");
    Serial.println(receivedData.status & PacketFlags::ERROR_STATE ? "ON" : "OFF");
    Serial.print("ERROR_CODE: ");
    Serial.println(receivedData.status & PacketFlags::ERROR_CODE ? "ON" : "OFF");

    if (receivedData.status & PacketFlags::P1_COM) {
        Serial.println("→ Pump 1 command received");
        PumpControl::setPump1(receivedData.status & PacketFlags::PUMP1);
    }
    if (receivedData.status & PacketFlags::P2_COM) {
        Serial.println("→ Pump 2 command received");
        PumpControl::setPump2(receivedData.status & PacketFlags::PUMP2);
    }
    LocalDataStruct.status = receivedData.status & PacketFlags::OVERRIDE_EN;
    if (receivedData.status & PacketFlags::OVERRIDE_EN) {
        Serial.println("→ Auto mode command received");
        PumpControl::setAutoMode(receivedData.status & PacketFlags::AUTO_MODE);
    }

    // }
    // reserved for future use
}

void resetLoRaRequest() {
    // ensure RX continuous after TX
    static unsigned long lastTx = 0;
    if (millis() - lastTx > 400) {
        LoRa.request(SX126X_RX_CONTINUOUS);
        lastTx = millis();
    }
}

// ——————————————————————————————————
// 9) UTILITY PRINT FUNCTIONS
// ——————————————————————————————————
void printByteBinary(uint8_t val) {
    for (int8_t i = 7; i >= 0; --i) Serial.print(bitRead(val, i));
}

void printDataPacket(const DataPacket &p) {
    Serial.println("===== Data Packet =====");
    Serial.print("Receiver ID: 0x");
    if (p.receiverID < 0x10) Serial.print('0');
    Serial.println(p.receiverID, HEX);

    Serial.print("Status: B");
    printByteBinary(p.status);

    Serial.print("Packet ID: ");
    Serial.println(p.packetID);
    Serial.print("WL: ");
    Serial.println(p.waterLevel);
    Serial.print("T:   ");
    Serial.println(p.temperature);

    Serial.print("CRC8: 0x");
    if (p.crc8 < 0x10) Serial.print('0');
    Serial.println(p.crc8, HEX);
    Serial.println("=======================\n");
}
