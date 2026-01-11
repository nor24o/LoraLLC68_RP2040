// StorageConfig.hpp
#pragma once

#include <Arduino.h>
#include <EEPROM.h>

#include "structures.h" // for PacketFlags

namespace StorageConfig {

// — total flash page reserved for “EEPROM” (4 KiB) —
inline constexpr size_t EEPROM_SIZE = 4096;
inline constexpr int EEPROM_ADDR = 0;

// — layout in flash: status(2B), threshold(1B), crc8(1B) —
#pragma pack(push, 1)
struct Config {
    uint16_t status;
    uint8_t crc8;

    uint8_t calculateCRC() const {
        uint8_t crc = 0;
        const uint8_t* p = reinterpret_cast<const uint8_t*>(this);
        // cover status + threshold (3 bytes)
        for (size_t i = 0; i < sizeof(status); ++i) {
            crc ^= p[i];
            for (int b = 0; b < 8; ++b) {
                crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
            }
        }
        return crc;
    }
    void updateCRC() { crc8 = calculateCRC(); }
    bool validateCRC() const { return calculateCRC() == crc8; }
};
#pragma pack(pop)

// —in-RAM mirror of what’s in flash—
namespace {
Config _cfg;
}

// —1) call once in setup() to init the flash copy & defaults—
inline void begin() {
    EEPROM.begin(EEPROM_SIZE); // <— void, no if-check :contentReference[oaicite:2]{index=2}
    EEPROM.get(EEPROM_ADDR, _cfg);
    if (!_cfg.validateCRC()) {
        Serial.println("No valid config → loading defaults");
        _cfg.status = PacketFlags::AUTO_MODE;
        _cfg.updateCRC();
        EEPROM.put(EEPROM_ADDR, _cfg);
        EEPROM.commit(); // writes back, returns bool if you care :contentReference[oaicite:3]{index=3}
    }
}

// —2) accessors for your two values—
inline uint16_t loadStatus() { return _cfg.status; }

// —3) whenever either field may have changed—
inline void saveIfChanged(uint16_t newStatus) {
    if (newStatus != _cfg.status) {
        _cfg.status = newStatus;
        _cfg.updateCRC();
        EEPROM.put(EEPROM_ADDR, _cfg);
        if (EEPROM.commit()) {
            Serial.println("Config changed → saved ✔");
        } else {
            Serial.println("Config change → EEPROM commit FAILED!");
        }
    }
}

} // namespace StorageConfig
