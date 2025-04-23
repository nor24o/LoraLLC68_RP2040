

#pragma once
#include <Arduino.h>


// 2A    FF    19 1A 32 FA 01     FF    FF    FF    01    7C   3A 



// Example packet: 
//  3c 313031 3e 65    FF    00 1A 32 FA B1     FF    FF    FF    08
//  <   ID     > 2A    07    1E 20 55 33 44     C1    3F    81    12    
//               |     |     |                  |     |     |     |     
//               |     |     |                  |     |     |     |      
//               |     |     |                  |     |     |     |     
//               |     |     |                  |     |     |     └──────────── packetID = 0x12 (18)
//               |     |     |                  |     |     └────────────────── customCommands = 0x81 (10000001)
//               |     |     |                  |     └──────────────────────── IO_I = 0x3F (00111111)
//               |     |     |                  └────────────────────────────── IO_O = 0xC1 (11000001)
//               |     |     └── AnalogSensorData (9 bytes):
//               |     |          temperature[0] = 0x1E (30°C)
//               |     |          temperature[1] = 0x20 (32°C)
//               |     |          humidity = 0x55 (85%)
//               |     |          waterLevel = 0x33 (51%)
//               |     |          light = 0x44 (68%)
//               |     └──────── flags = 0x07 (bits: Auto Mode = 1, Override = 1, Error State = 1)
//               └────────────── receiverID = 0x2A
//              



// Set 1-byte alignment for all structures
#pragma pack(push, 1)
#define NUM_ANALOG 2 // Number of analog sensors (e.g., 2 temperature sensors)



// **Common Data Structure**
struct __attribute__((packed)) DataPacket
{
  uint8_t receiverID;    // Receiver ID (e.g., Tank Module ID)
  uint16_t crc; // CRC16
  uint8_t flags;         // Bitwise flags for various states // Bit 0: Auto Mode, Bit 1: Override Enabled, Bit 2: Error State Bit 3: Error Code Bit 4-7: Reserved
  uint8_t IO_O;          // IO_output states (e.g., Pump 1, Pump 2)
  uint8_t packetID;      // Packet ID for tracking
  uint16_t waterLevel;   // Water level data 
  uint16_t temperature[NUM_ANALOG]; // Temperature data (e.g., Sensor 1, Sensor 2)

  // Calculate CRC16 (excluding crc field itself)
  uint16_t calculateCRC() const {
    const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
    size_t len = sizeof(DataPacket) - sizeof(crc); // Exclude crc field

    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
      crc ^= data[i];
      for (uint8_t j = 0; j < 8; ++j) {
        if (crc & 1)
          crc = (crc >> 1) ^ 0xA001;
        else
          crc >>= 1;
      }
    }
    return crc;
  }

  void updateCRC() {
    crc = calculateCRC();
  }

  bool validateCRC() const {
    return calculateCRC() == crc;
  }
};

#pragma pack(pop) // Restore default alignment

