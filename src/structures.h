// Example packet (packed B-format):
//
//   2A    01 C1   12     FA      1E 00       28 00      A9
//  <ID> <status> <pkt> <wLvl> < temp[0] > < temp[1] > <crc8>
//   |      |        |      |       |           |         └─ CRC-8 = 0xA9 (Dallas/Maxim)
//   |      |        |      |       |           └─ temperature[1] = 0x0028 (40 °C)
//   |      |        |      |       └─ temperature[0] = 0x001E (30 °C)
//   |      |        |      └─ waterLevel      = 0xFA   (250)
//   |      |        └─ packetID         = 0x12   (18)
//   |      └─ status byte     = 0xC1   (1100 0001: AUTO_MODE + PUMP3 + PUMP4 ON)
//   └─ receiverID           = 0x2A   (42)
// DataPacket.h

// Example packet (packed with 16-bit status):
//   2A  01 C1  12  FA  1E 00  28 00  A9
//  <ID> <status_low status_high> <pkt> <wLvl> < temp[0] > < temp[1] > <crc8>
//   |      |      |       |       |           |           └─ CRC-8 = 0xA9
//   |      |      |       |       |           └─ temperature[1] = 0x0028 (40 °C)
//   |      |      |       |       └─ temperature[0] = 0x001E (30 °C)
//   |      |      |       └─ waterLevel      = 0xFA   (250)
//   |      |      └─ packetID         = 0x12   (18)
//   |      └─ status_low   = 0x01    (AUTO_MODE bit set)
//   |      └─ status_high  = 0xC1    (PUMP1 + PUMP2 + STOP_FILL bits in high byte)
//   └─ receiverID        = 0x2A   (42)

#pragma once
#include <Arduino.h>

#define NUM_ANALOG 2 // ← your number of temp sensors

#pragma pack(push, 1)
struct DataPacket {
    uint8_t receiverID;              // 1 B
    uint16_t status;                 // 4-bit flags + 4-bit IO_O
    uint8_t packetID;                // 1 B
    uint8_t waterLevel;              // 1 B (0–255)
    float temperature[NUM_ANALOG]; // 4 bytes * 2 = 8 bytes
    uint8_t crc8;                    // 1 B

    // —— CRC-8 (Dallas/Maxim, poly 0x31) ——
    uint8_t calculateCRC8() const {
        uint8_t crc = 0;
        auto ptr = reinterpret_cast<const uint8_t*>(this);
        size_t len = sizeof(*this) - sizeof(crc8);
        for (size_t i = 0; i < len; ++i) {
            crc ^= ptr[i];
            for (int b = 0; b < 8; ++b) {
                crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
            }
        }
        return crc;
    }
    void updateCRC8() { crc8 = calculateCRC8(); }
    bool validateCRC8() const { return calculateCRC8() == crc8; }
};
#pragma pack(pop)
static_assert(sizeof(DataPacket) == 6 + 4 * NUM_ANALOG, "Unexpected DataPacket size");

// in structures.h / DataPacket.h:
namespace PacketFlags {
using Flags = uint16_t;
static constexpr Flags OVERRIDE_EN = Flags(1) << 0;
static constexpr Flags AUTO_MODE = Flags(1) << 1;
static constexpr Flags ERROR_STATE = Flags(1) << 2;
static constexpr Flags ERROR_CODE = Flags(1) << 3;
static constexpr Flags PUMP1 = Flags(1) << 4;
static constexpr Flags PUMP2 = Flags(1) << 5;
static constexpr Flags STOP_FILL = Flags(1) << 6;
static constexpr Flags START_FILL = Flags(1) << 7;
static constexpr Flags P1_COM = Flags(1) << 8;
static constexpr Flags P2_COM = Flags(1) << 9;
// bits 10–15 free
} // namespace PacketFlags
