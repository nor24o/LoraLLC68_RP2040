// CONTROLLER BAZIN APA 50000L

// functions:
// - read DS18B20 temperature sensors
// - read Humidity sensor
// - read Light sensor over Serial
// - read Water level sensor using TOF over Serial
// - read IO input states (e.g.,  Button state)
// - read IO output states (e.g., Pump 1, Pump 2)
// - send data over LoRa
// - receive data over LoRa
// - process received data (e.g., control pumps, override mode)

// Global definitions
#define LORA_WRITE_NUM_WRAPPED(id)   \
  {                                  \
    LoRa.write('<');                 \
    char _buf[10];                   \
    itoa(id, _buf, 10);              \
    for (int _i = 0; _buf[_i]; _i++) \
      LoRa.write(_buf[_i]);          \
    LoRa.write('>');                 \
  }
#define BIT_ON 1
#define BIT_OFF 0

#include <Arduino.h>
#include <SX126x.h>

#include <FastLED.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

#include "structures.h"
#include <TaskScheduler.h>
#include <SoftwareSerial.h>
#include "controller.hpp"
#include "IndicatControll.hpp"

SX126x LoRa;

// === Pins ===
#define DS18B20_PIN 7 // Change if needed

SoftwareSerial mySerial(13, 14); // RX, TX

#define myID 100
#define secondNodeID 101
#define GatewayID 3

uint8_t currentPacketID = 1;

// LoRa Pin Configuration
const int8_t NSS_PIN = 1;
const int8_t RESET_PIN = 4;
const int8_t BUSY_PIN = 5;
const int8_t IRQ_PIN = 6;
const int8_t TXEN_PIN = -1;
const int8_t RXEN_PIN = -1;

// DS18B20 Configuration
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
DeviceAddress sensorAddresses[5]; // Supports up to 5 DS18B20 sensors
int ds18b20Count = 0;
#define EEPROM_ADDR 0 // EEPROM address to store sensor data

// Global instance
DataPacket LocalDataStruct;
DataPacket receivedData;

bool dataWasMeasured = false;
float ds18b20_temps[5];

String inputString = "";
int waterLevelSerial = 0;  // Water level sensor value
int TemperatureSerial = 0; // Temperature sensor value
char b[16];
byte j;

unsigned long lastSerialReceiveTime = 0;  // Timestamp of the last received data
const unsigned long serialTimeout = 5000; //

void printDataPacket(const DataPacket &packet);
void getSensorData();
void readLoraPacket();
void sendLoRaData();
void printSensorData();
void processSerialInput(String data);
void ReadSerialData();

// Task to read sensor data
#define getSensorDataInterval 10000
Task T_get_sensor_data(getSensorDataInterval, TASK_FOREVER, &getSensorData);

// Task to print data to serial
#define printDataInterval 30000
Task T_print_sensor_data(printDataInterval, TASK_FOREVER, &printSensorData);

#define sendLoRaDataInterval 20000
Task T_send_lora_data(sendLoRaDataInterval, TASK_FOREVER, &sendLoRaData);

Scheduler runner;

void setup()
{
  mySerial.begin(9600);
  SPI.setRX(0);
  SPI.setCS(1);
  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.begin();

  Serial.begin(115200);

  unsigned long start = millis();
  while (!Serial && millis() - start < 8000)
    ; // Wait max 3 seconds

  PumpControl::begin();

  Serial.println("Initializing LoRa module...");
  if (!LoRa.begin(NSS_PIN, RESET_PIN, BUSY_PIN, IRQ_PIN, TXEN_PIN, RXEN_PIN))
  {
    Serial.println("Error: Unable to initialize LoRa module.");
    while (1)
      ;
  }

  ds18b20.begin();

  getSensorData(); // Get sensor data once at startup

  Serial.println("Set RF module to use TCXO as clock reference");
  uint8_t dio3Voltage = SX126X_DIO3_OUTPUT_1_8;
  uint32_t tcxoDelay = SX126X_TCXO_DELAY_10;
  LoRa.setDio3TcxoCtrl(dio3Voltage, tcxoDelay);
  LoRa.setFrequency(867300000);
  LoRa.setTxPower(14, SX126X_TX_POWER_SX1262);
  LoRa.setLoRaModulation(9, 125000, 5);
  LoRa.setLoRaPacket(SX126X_HEADER_EXPLICIT, 8, 15, true);
  LoRa.setSyncWord(0x34);

  // Set RX gain to boosted gain
  // Serial.println("Set RX gain to boosted gain");
  // LoRa.setRxGain(SX126X_RX_GAIN_BOOSTED);

  Serial.println("\n-- LORA TRANSMITTER READY --\n");

  // Request for receiving new LoRa packet in RX continuous mode
  LoRa.request(SX126X_RX_CONTINUOUS);

  //Start IndicatorControl
  IndicatorControl::begin();

  // Initialize tasks
  runner.init();
  Serial.println("Initialized scheduler");
  runner.addTask(T_print_sensor_data); // Add print sensor data task
  runner.addTask(T_get_sensor_data);   // Add get sensor data task
  runner.addTask(T_send_lora_data);    // Add send LoRa data task
  T_get_sensor_data.enable();          // Enable get sensor data task
  T_print_sensor_data.enable();        // Enable print sensor data task
  T_send_lora_data.enable();           // Enable send LoRa data task
  Serial.println("Enabled tasks");
}

unsigned long lastTransmitTime = 0;
const unsigned long rxResumeDelay = 400; // ms after transmission to re-enable RX
bool shouldResumeRx = false;

void resetLoRaRequest()
{
  // Delay re-entering RX mode until tx is well clear
  if (shouldResumeRx && millis() - lastTransmitTime >= rxResumeDelay)
  {
    LoRa.request(SX126X_RX_CONTINUOUS);
    IndicatorControl::setLED(CRGB::Blue, 100);
    shouldResumeRx = false;
  }
}

void sendLoRaData()
{
  if (!dataWasMeasured)
  {
    IndicatorControl::setLED(CRGB::Red, 200);
    return;
  }

  IndicatorControl::setLED(CRGB::Green, 200);
  dataWasMeasured = false;
  Serial.println("Sending LoRa Data...");

  currentPacketID++;
  // if the current packet ID is maxed out, reset it to 1
  if (currentPacketID == 255)
  {
    currentPacketID = 1;
  }

  LocalDataStruct.packetID = currentPacketID;
  LocalDataStruct.receiverID = secondNodeID; // Set receiver ID to second node
  LocalDataStruct.updateCRC();            // Calculate CRC for the packet
  LoRa.beginPacket();
  LORA_WRITE_NUM_WRAPPED(myID); // Write sender ID
  LoRa.write((uint8_t *)&LocalDataStruct, sizeof(LocalDataStruct));
  LoRa.endPacket();
  LoRa.wait();
  LoRa.purge();
  bitWrite(LocalDataStruct.flags, 7, BIT_OFF); // Bit 0: stop filling command

  //  //print raw datapacket in hex make it print all even the 0 aka 0A instead of A
  //  for (uint8_t i = 0; i < sizeof(LocalDataStruct); i++)
  //  {
  //    uint8_t val = ((uint8_t *)&LocalDataStruct)[i];
  //    if (val < 0x10) Serial.print("0");  // Add leading zero if needed
  //    Serial.print(val, HEX);
  //  }
  //  Serial.println();

  // Packet sent successfully, turn LED green for 1 second
  IndicatorControl::setLED(CRGB::Green, 1000);

  // Delay RX re-entry
  lastTransmitTime = millis();
  shouldResumeRx = true;
}

void printByteBinary(uint8_t val)
{
  for (int8_t i = 7; i >= 0; i--)
  {
    Serial.print(bitRead(val, i));
  }
  Serial.println();
}

void resetMySerial()
{
  mySerial.end();                   // End the current SoftwareSerial instance
  delay(100);                       // Small delay before reinitializing
  mySerial.begin(9600);             // Reinitialize mySerial
  lastSerialReceiveTime = millis(); // Reset the last received time
  Serial.println("mySerial has been reset.");
}

// funtion to print the datapacket received in human readable format
void printDataPacket(const DataPacket &packet)
{
  Serial.println("===== Data Packet =====");
  Serial.print("Debug ID: ");
  Serial.println(random(0, 100)); // Random debug ID for testing

  Serial.print("Receiver ID: ");
  Serial.println(packet.receiverID);

  Serial.print("Flags: B");
  printByteBinary(packet.flags);

  Serial.print("Temperature 1: ");
  Serial.println(packet.temperature[0]);
  Serial.print("Temperature 2: ");
  Serial.println(packet.temperature[1]);

  Serial.print("Water Level: ");
  Serial.println(packet.waterLevel);

  Serial.print("IO Output: B");
  printByteBinary(packet.IO_O);

  Serial.print("Packet ID: ");
  Serial.println(packet.packetID);

  Serial.println("================================");
}



// Main loop
void loop()
{
  ReadSerialData(); // Read data from mySerial
  PumpControl::update(LocalDataStruct); // Check buttons using AceButton
  IndicatorControl::update();           // Update LED state

  runner.execute(); // Execute tasks

  readLoraPacket(); // Read LoRa packet

  resetLoRaRequest(); // Reset LoRa request to receive new packets
}


void processSerialInput(String data)
{
  int commaIndex = data.indexOf(',');
  if (commaIndex != -1)
  {
    String levelStr = data.substring(0, commaIndex);
    String ntcStr = data.substring(commaIndex + 1);

    waterLevelSerial = levelStr.toInt();
    TemperatureSerial = ntcStr.toInt();
  }
}
void ReadSerialData()
{
  // Check if data is available on mySerial
  while (mySerial.available())
  {
    char ch = mySerial.read();
    if (ch == '\n')
    {
      processSerialInput(inputString);
      inputString = "";                 // Clear for next read
      lastSerialReceiveTime = millis(); // Update the last received time
    }
    else
    {
      inputString += ch;
    }
  }

  // Check for serial timeout
  if (millis() - lastSerialReceiveTime > serialTimeout)
  {
    Serial.println("Warning: mySerial is stuck. Attempting to reset...");
    resetMySerial();
  }
}

// Function to read LoRa packet
void readLoraPacket()
{
  if (LoRa.available())
  {
    static bool readData = true;

    // Read header starting with '<' and ending with '>'
    String idString = "";
    char ch;
    bool headerStarted = false;

    // Read until we find a full <ID> or timeout
    unsigned long start = millis();
    while ((millis() - start) < 1000) // timeout after 1 sec
    {
      if (LoRa.available())
      {
        ch = LoRa.read();

        if (ch == '<')
        {
          idString = ""; // Reset
          headerStarted = true;
        }
        else if (ch == '>' && headerStarted)
        {
          break; // End of header
        }
        else if (headerStarted)
        {
          idString += ch;
        }
      }
    }

    // Convert to int
    int senderID = idString.toInt();
    Serial.print("Parsed sender ID from header: ");
    Serial.println(senderID);

    if (senderID == myID)
    {
      readData = false; // Ignore own packets
      Serial.println("Received packet from own node, ignore.");
      return; // Ignore own packets
    }
    else if (senderID == secondNodeID)
    {
      Serial.println("Received packet from second node, update acordingly.");
    }
    else if (senderID == GatewayID)
    {
      Serial.println("Received packet from gateway, update acordingly.");
    }

    Serial.print("Remaining bytes after header: ");
    Serial.println(LoRa.available());

    // Now read the binary payload (DataPacket)
    if (LoRa.available() >= sizeof(DataPacket) && readData)
    {
      uint8_t buffer[sizeof(DataPacket)];
      for (uint8_t i = 0; i < sizeof(DataPacket); i++)
      {
        buffer[i] = LoRa.read();
      }

      // Print raw data in HEX with leading zeros
      for (uint8_t i = 0; i < sizeof(DataPacket); i++)
      {
        if (buffer[i] < 0x10)
          Serial.print("0");
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      memcpy(&receivedData, buffer, sizeof(DataPacket));

      Serial.print("RSSI: ");
      Serial.println(LoRa.packetRssi());
      Serial.print("SNR: ");
      Serial.println(LoRa.snr());

      printDataPacket(receivedData);
    }
    else
    {
      Serial.println("Not enough data for full DataPacket");
    }
  }
}

void processReceivedData()
{
  // Process the received data here
  // For example, you can control pumps or update local data structure
  // based on the received packet
  Serial.println("Processing received data...");
  // Add your processing logic here
}

// Function to get sensor data Task Handled by TaskScheduler
void getSensorData()
{
  IndicatorControl::setLED(CRGB::Red, 1000); // Turns LED red for 1000 milliseconds (1 second)

  ds18b20.requestTemperatures();
  float temp = ds18b20.getTempCByIndex(0); // Read first (and only) sensor

  // Update sensor struct
  LocalDataStruct.temperature[0] = temp;
  LocalDataStruct.temperature[1] = (1000 - TemperatureSerial) * (50.0 / 400.0); // (maxRaw - raw) * (range / span); // Map raw to 0–50 °C; // Use the value from the serial input
  LocalDataStruct.waterLevel = waterLevelSerial;                                // Use the value from the serial input
  printDataPacket(LocalDataStruct);
  dataWasMeasured = true;
}

// Print sensor data to serial Task Handled by TaskScheduler
void printSensorData()
{
  // setLEDBlink(CRGB::OrangeRed, 2000, 500, 6); // Blink blue for a total of 2000 ms, with 250 ms interval, 3 full blinks

  //printDataPacket(LocalDataStruct);
}