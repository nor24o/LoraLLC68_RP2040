// CONTROLLER BAZIN APA 50000L

// ——————————————————————————————————
// 1) INCLUDES & DEFINES
// ——————————————————————————————————

// pin assignments
#define DS18B20_PIN 7
#define LED_PIN 15
#define PUMP1_PIN 12
#define PUMP2_PIN 11
#define BTN_AUTO 10
#define BTN_PUMP1 8
#define BTN_PUMP2 9
#define SECOND_SERIAL_RX 13
#define SECOND_SERIAL_TX 14

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
#include <SoftwareSerial.h>
#include <TaskScheduler.h>
#include <math.h> // for log()

#include "IndicatControll.hpp"
#include "StorageConfig.hpp"
#include "controller.hpp"
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
#define myID 101
#define secondNodeID 102
#define GatewayID 100

// Task intervals (ms)
#define getSensorDataInterval 10000
#define printDataInterval 30000
#define sendLoRaDataInterval 20000

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
SoftwareSerial mySerial(SECOND_SERIAL_RX, SECOND_SERIAL_TX);

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
    // while (!Serial)
    //   ; // wait for USB

    // 1) initialize & load stored status + threshold
    StorageConfig::begin();
    LocalDataStruct.status = StorageConfig::loadStatus();
    PumpControl::setWaterLevelThreshold(StorageConfig::loadThreshold());

    // initialize pumps & buttons
    PumpControl::begin();
    PumpControl::updatelocalstates(LocalDataStruct);
    // initialize LED indicator
    IndicatorControl::begin();

    // serial for external sensors
    mySerial.begin(9600);

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
        while (1);
    }

    ds18b20.begin();
    ReadSerialData(); // read sensor serial

    // configure LoRa TX/RX
    LoRa.setDio3TcxoCtrl(SX126X_DIO3_OUTPUT_1_8, SX126X_TCXO_DELAY_10);
    LoRa.setFrequency(867300000);
    LoRa.setTxPower(14, SX126X_TX_POWER_SX1262);
    LoRa.setLoRaModulation(9, 125000, 5);
    LoRa.setLoRaPacket(SX126X_HEADER_EXPLICIT, 8, 15, true);
    LoRa.setSyncWord(0x34);
    LoRa.request(SX126X_RX_CONTINUOUS);

    Serial.println("-- LORA TRANSMITTER READY --");

    // scheduler tasks
    runner.init();
    runner.addTask(T_get_sensor_data);
    runner.addTask(T_print_sensor_data);
    runner.addTask(T_send_lora_data);
    T_get_sensor_data.enable();
    T_print_sensor_data.enable();
    T_send_lora_data.enable();
    Serial.println("Tasks enabled");
    IndicatorControl::Running();   // start in “running” (blinking) mode
}

void loop() {
    //IndicatorControl::setMode(IndicatorControl::RUNNING);
    ReadSerialData();                     // read sensor serial
    ReadMainSerialData();                 // read USB-serial commands
    readLoraPacket();                     // handle inbound LoRa
    PumpControl::update(LocalDataStruct); // update pump logic + packet.status
    // push updated threshold into indicator each cycle
    IndicatorControl::setThreshold(PumpControl::getWaterLevelThreshold());
    IndicatorControl::update(LocalDataStruct); // LED indicator
    runner.execute();                          // run scheduled tasks
    resetLoRaRequest();                        // re-enter RX if needed
    // only write back if either field actually changed
    StorageConfig::saveIfChanged(LocalDataStruct.status, PumpControl::getWaterLevelThreshold());
}

// ——————————————————————————————————
// 5) SERIAL COLLECTION (external sensors)
// ——————————————————————————————————
void ReadSerialData() {
    while (mySerial.available()) {
        char ch = mySerial.read();
        // ignore stray '\n' or '\r' in the middle
        if (ch == '\n' || ch == '\r') {
            if (inputString.length() > 0) {
                processSerialInput(inputString);
                inputString = "";
                lastSerialReceiveTime = millis();
            }
        } else {
            inputString += ch;
        }
    }

    if (millis() - lastSerialReceiveTime > serialTimeout) {
        resetMySerial();
    }
}
const float MID_READING = 827.0; // ADC value that equals 21 °C
const float MID_TEMP = 21.0;     // °C at that midpoint reading
const float SCALE = 0.1;         // °C per ADC step (tweak this)

void processSerialInput(const String &data) {
    // ---- 1) Print the raw line ----
    // Serial.print(F("RAW Received: "));
    // Serial.println(data);

    // ---- 2) Split into fields ----
    int i1 = data.indexOf(',');
    int i2 = data.indexOf(',', i1 + 1);
    int i3 = data.indexOf(',', i2 + 1);
    if (i1 < 0 || i2 < 0 || i3 < 0) return; // malformed

    // ---- 3) Parse raw integers ----
    uint16_t dist_mm = data.substring(0, i1).toInt();
    uint16_t t10 = data.substring(i1 + 1, i2).toInt();
    uint16_t h10 = data.substring(i2 + 1, i3).toInt();
    uint8_t status = data.substring(i3 + 1).toInt();
    // ---- 4) Store into LocalDataStruct ----
    LocalDataStruct.waterLevel = map(constrain(dist_mm, 0, 2000), 0, 2000, 255, 0);
    LocalDataStruct.temperature[1] = (float(t10) / 10.0f) - 5.0f; // 5.0 offset for DS18B20
    // LocalDataStruct.humidity       = float(h10) / 10.0f;
    LocalDataStruct.status = status;
    dataWasMeasured = true;

    // ---- 5) Print formatted values ----
    // Serial.print(F("DIST="));
    // Serial.print(dist_mm);
    // Serial.print(F(" mm  TEMP="));
    // Serial.print(LocalDataStruct.temperature[1], 1);
    ////Serial.print(F(" °C  HUM="));
    ////Serial.print(LocalDataStruct.humidity, 1);
    // Serial.print(F(" %  STATUS=0b"));
    //// Print status as 4-bit binary:
    // for (int bit = 3; bit >= 0; --bit) {
    //   Serial.print( (status & (1 << bit)) ? '1' : '0' );
    // }
    // Serial.println();
}

void resetMySerial() {
    mySerial.end();
    delay(100);
    mySerial.begin(9600);
    lastSerialReceiveTime = millis();
}

// ——————————————————————————————————
// 6) USB-SERIAL COMMANDS
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
    } else if (s.startsWith("w")) {
        int t = s.substring(1).toInt();
        PumpControl::setWaterLevelThreshold(constrain(t, 0, 255));
        Serial.printf("Threshold set to: %u\n", map(PumpControl::getWaterLevelThreshold(), 0, 255, 0, 2500));
    } else if (s.startsWith("q")) {
        int t = s.substring(1).toInt();
        LocalDataStruct.waterLevel = constrain(t, 0, 255);
        Serial.printf("Debug level set to: %u\n", map(LocalDataStruct.waterLevel, 0, 255, 0, 2500));
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
void getSensorData() {
    // 1) no devices on the bus?
     if (ds18b20.getDeviceCount() == 0) {
         ds18b20.begin(); // re-scan & re-init the bus
         LocalDataStruct.temperature[0] = 255;
         dataWasMeasured = true;
         return;
     }
    
     // 2) ask for a new reading
     ds18b20.requestTemperatures();
     float temp = ds18b20.getTempCByIndex(0);
    
     // 3) check for errors (disconnected, NaN or out-of-range)
     if (temp == DEVICE_DISCONNECTED_C || isnan(temp) || temp < -55.0f || temp > 125.0f) {
         ds18b20.begin(); // reset the bus
         LocalDataStruct.temperature[0] = DEVICE_DISCONNECTED_C;
     } else {
        LocalDataStruct.temperature[0] = temp;
     }


    dataWasMeasured = true;
}

void printSensorData() {
    Serial.printf("T0=%.2f  T1=%.2f  WL=%u  WT=%u\n", LocalDataStruct.temperature[0], LocalDataStruct.temperature[1], LocalDataStruct.waterLevel,
                  PumpControl::getWaterLevelThreshold());

    const uint8_t *p = reinterpret_cast<const uint8_t *>(&LocalDataStruct);
    size_t n = sizeof(LocalDataStruct);

    Serial.print("LocalDataStruct: ");
    for (size_t i = 0; i < n; ++i) {
        if (i) Serial.print(' ');
        if (i < n - 1) {
            // %02X → two-digit hex, leading zero if needed
            Serial.printf("%02X", p[i]);
        } else {
            // last byte in decimal
            Serial.printf("%u", p[i]);
        }
    }
    Serial.println();
}

// ——————————————————————————————————
// 8) LoRa SEND / RECEIVE
// ——————————————————————————————————
void sendLoRaData() {
    if (LoRa.available() || !dataWasMeasured) return;
    dataWasMeasured = false;
    IndicatorControl::Transmitting();


    currentPacketID = (currentPacketID % 255) + 1;
    LocalDataStruct.packetID = currentPacketID;
    LocalDataStruct.receiverID = GatewayID;
    LocalDataStruct.updateCRC8();

    LoRa.beginPacket();
    LORA_WRITE_NUM_WRAPPED(myID); // Write sender ID
    LoRa.write((uint8_t *)&LocalDataStruct, sizeof(LocalDataStruct));
    LoRa.endPacket();
    LoRa.wait();
    LoRa.purge();
    LoRa.request(SX126X_RX_CONTINUOUS);
}

void readLoraPacket() {
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
    Serial.println(receivedData.status & PacketFlags::STOP_FILL ? "ON" : "OFF");
    Serial.print("START_FILL: ");
    Serial.println(receivedData.status & PacketFlags::START_FILL ? "ON" : "OFF");
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
    Serial.print("Water Lvl: ");
    Serial.println(map(p.waterLevel, 0, 255, 0, 2500));
    Serial.print("Water Lvl RAW: ");
    Serial.println(p.waterLevel);
    Serial.print("Temp[0]:   ");
    Serial.println(p.temperature[0]);
    Serial.print("Temp[1]:   ");
    Serial.println(p.temperature[1]);

    Serial.print("CRC8: 0x");
    if (p.crc8 < 0x10) Serial.print('0');
    Serial.println(p.crc8, HEX);
    Serial.println("=======================\n");
}

// ——————————————————————————————————
// 10) UNUSED / PLACEHOLDERS
// ——————————————————————————————————
