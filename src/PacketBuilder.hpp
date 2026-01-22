#ifndef PACKET_BUILDER_HPP
#define PACKET_BUILDER_HPP

#include <Arduino.h>

// ——————————————————————————————————————————————————
// SHARED DATA TYPES
// ——————————————————————————————————————————————————
enum PacketDataType : uint8_t {
    TYPE_STATUS     = 0x01,  // 1 Byte
    TYPE_TEMP       = 0x02,  // 2 Bytes (x100)
    TYPE_HUMIDITY   = 0x03,  // 2 Bytes (x10)
    TYPE_WATER_LVL  = 0x04,  // 1 Byte
    TYPE_BATTERY    = 0x05,  // 2 Bytes (mV)
    TYPE_PRESSURE   = 0x06,  // 2 Bytes (hPa)
    TYPE_COUNTER    = 0x07,  // 1 Byte
    TYPE_ERROR      = 0x08,  // 1 Byte
    // Raw Data Types
    TYPE_GENERIC_1B = 0x10,  // 1 Byte 
    TYPE_GENERIC_2B = 0x11,  // 2 Bytes
    TYPE_GENERIC_4B = 0x12   // 4 Bytes
};

// ——————————————————————————————————————————————————
// CLASS 1: BUILDER (SENDER)
// ——————————————————————————————————————————————————
class PacketBuilder {
public:
    PacketBuilder(uint8_t size = 64) {
        _maxSize = size;
        _buffer = (uint8_t*) malloc(size);
        _cursor = 0;
    }

    ~PacketBuilder() {
        if (_buffer) free(_buffer);
    }

    void reset() { _cursor = 0; }
    uint8_t* getBuffer() { return _buffer; }
    uint8_t getSize() { return _cursor; }

    // --- ADDERS ---

        // Adds Packet ID (Must be called first usually)
    bool addPacketID(uint8_t id) {
        // Channel 99 is standard for Metadata
        if (!_addHeader(99, TYPE_COUNTER)) return false;
        _buffer[_cursor++] = id;
        return true;
    }
    
    // [Channel] [Type] [Value]
    bool addTemp(uint8_t ch, float celsius) {
        if (!_addHeader(ch, TYPE_TEMP)) return false;
        _write16((int16_t)(celsius * 100.0));
        return true;
    }

    bool addStatus(uint8_t ch, uint8_t status) {
        if (!_addHeader(ch, TYPE_STATUS)) return false;
        _buffer[_cursor++] = status;
        return true;
    }

    bool addWaterLevel(uint8_t ch, uint8_t level) {
        if (!_addHeader(ch, TYPE_WATER_LVL)) return false;
        _buffer[_cursor++] = level;
        return true;
    }

    bool addCounter(uint8_t ch, uint8_t count) {
        if (!_addHeader(ch, TYPE_COUNTER)) return false;
        _buffer[_cursor++] = count;
        return true;
    }

    bool addBattery(uint8_t ch, float volts) {
        if (!_addHeader(ch, TYPE_BATTERY)) return false;
        _write16((int16_t)(volts * 1000.0)); // Send mV
        return true;
    }

    bool addHumidity(uint8_t ch, float percent) {
        if (!_addHeader(ch, TYPE_HUMIDITY)) return false;
        _write16((int16_t)(percent * 10.0));
        return true;
    }

    bool addGeneric(uint8_t ch, int32_t val, uint8_t numBytes) {
        PacketDataType t = (numBytes == 4) ? TYPE_GENERIC_4B : (numBytes == 2 ? TYPE_GENERIC_2B : TYPE_GENERIC_1B);
        if (!_addHeader(ch, t)) return false;
        for (int i = numBytes - 1; i >= 0; i--) _buffer[_cursor++] = (val >> (i * 8)) & 0xFF;
        return true;
    }

private:
    uint8_t* _buffer;
    uint8_t  _maxSize;
    uint8_t  _cursor;

    bool _addHeader(uint8_t ch, PacketDataType type) {
        if (_cursor + 3 > _maxSize) return false; // Ensure space for Ch, Type, + at least 1 byte data
        _buffer[_cursor++] = ch;
        _buffer[_cursor++] = (uint8_t)type;
        return true;
    }

    void _write16(int16_t val) {
        _buffer[_cursor++] = (val >> 8) & 0xFF;
        _buffer[_cursor++] = (val & 0xFF);
    }
};

// ——————————————————————————————————————————————————
// CLASS 2: PARSER (RECEIVER)
// ——————————————————————————————————————————————————
class PacketParser {
public:
    // Initialize with the received buffer and size
    PacketParser(uint8_t* buf, uint8_t size) {
        _buffer = buf;
        _size = size;
        _cursor = 0;
    }

        // INTERNAL TRACKING for De-Duplication
    // Static so it persists across different loop calls
    static uint8_t lastID; 

    // Call this in a while loop. Returns true as long as there is data to read.
    bool next() {
        if (_cursor + 2 > _size) return false; // Need at least Ch + Type

        // Read Header
        _currentChannel = _buffer[_cursor++];
        _currentType = (PacketDataType)_buffer[_cursor++];

        // Calculate Data Length based on Type
        _dataLen = _getTypeSize(_currentType);

        // Check safety
        if (_cursor + _dataLen > _size) return false;

        // Point to data start
        _dataPtr = &_buffer[_cursor];
        
        // Advance cursor for next loop
        _cursor += _dataLen;
        return true;
    }


    // --- ECHO DETECTION ---
    // Returns TRUE if the received ID matches MY current outgoing ID (Reflection)
    bool isEcho(uint8_t myCurrentID, uint8_t receivedID) {
        return (receivedID == myCurrentID);
    }


    // --- GETTERS (Valid after calling next()) ---

    uint8_t getChannel() { return _currentChannel; }
    PacketDataType getType() { return _currentType; }

    // Returns extracted float for Temp (Auto scales /100.0)
    float getTemp() {
        if (_currentType != TYPE_TEMP) return 0.0;
        int16_t raw = (_dataPtr[0] << 8) | _dataPtr[1];
        return raw / 100.0;
    }

    // Returns extracted float for Humidity (Auto scales /10.0)
    float getHumidity() {
        if (_currentType != TYPE_HUMIDITY) return 0.0;
        int16_t raw = (_dataPtr[0] << 8) | _dataPtr[1];
        return raw / 10.0;
    }

    // Returns extracted float for Battery (Auto scales mV -> V)
    float getBattery() {
        if (_currentType != TYPE_BATTERY) return 0.0;
        int16_t raw = (_dataPtr[0] << 8) | _dataPtr[1];
        return raw / 1000.0;
    }

    // Returns raw byte (Status, WaterLvl, Counter, Generic1B)
    uint8_t getByte() {
        return _dataPtr[0];
    }

    // Returns raw int16 (Generic 2B, Pressure)
    int16_t getInt() {
        return (_dataPtr[0] << 8) | _dataPtr[1];
    }

    // Returns raw int32 (Generic 4B)
    int32_t getLong() {
        return ((int32_t)_dataPtr[0] << 24) | ((int32_t)_dataPtr[1] << 16) | 
               ((int32_t)_dataPtr[2] << 8)  | (int32_t)_dataPtr[3];
    }

private:
    uint8_t* _buffer;
    uint8_t  _size;
    uint8_t  _cursor;
    
    // Current Item State
    uint8_t        _currentChannel;
    PacketDataType _currentType;
    uint8_t* _dataPtr;
    uint8_t        _dataLen;

    uint8_t _getTypeSize(PacketDataType t) {
        switch(t) {
            case TYPE_STATUS: case TYPE_WATER_LVL: case TYPE_COUNTER: 
            case TYPE_ERROR: case TYPE_GENERIC_1B:
                return 1;
            case TYPE_TEMP: case TYPE_HUMIDITY: case TYPE_BATTERY: 
            case TYPE_PRESSURE: case TYPE_GENERIC_2B:
                return 2;
            case TYPE_GENERIC_4B:
                return 4;
            default: return 0;
        }
    }
};

#endif