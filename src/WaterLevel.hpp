#pragma once
#include <Arduino.h>

namespace WaterLevel {
// ─────── Pin definitions (inline to avoid ODR/linker issues) ───────
inline uint8_t pinWL0 = 0; // bottom float
inline uint8_t pinWL1 = 0; // middle float
inline uint8_t pinWL2 = 0; // top float

// ─────── Bit-masks & Levels ───────
constexpr uint8_t LEVEL_EMPTY = 0;
constexpr uint8_t LEVEL_LOW = 1;
constexpr uint8_t LEVEL_MID = 2;
constexpr uint8_t LEVEL_FULL = 3;
constexpr uint8_t LEVEL_MASK = 0x03;
constexpr uint8_t ABNORMAL_FLAG = 0x04; // bit-2

// ─────── Public state ───────
// bits [1:0] = 0..3; bit-2 = abnormal pattern
inline uint8_t waterState = 0;

// ─────── Initialize (call once in setup) ───────
// wl0Pin..wl2Pin: your actual GPIO numbers
inline void begin(uint8_t wl0Pin, uint8_t wl1Pin, uint8_t wl2Pin) {
    pinWL0 = wl0Pin;
    pinWL1 = wl1Pin;
    pinWL2 = wl2Pin;
    // use INPUT if you have external pull-ups, or INPUT_PULLUP to enable internal ones
    pinMode(pinWL0, INPUT_PULLDOWN);
    pinMode(pinWL1, INPUT_PULLDOWN);
    pinMode(pinWL2, INPUT_PULLDOWN);
}

// imput from sensor is HIGH If water is present

// imput from sensor is LOW If water is not present

// ─────── Sample & update ───────
inline void update() {
    // HIGH = water present, LOW = no water
    bool wl_low  = digitalRead(pinWL0)  == HIGH;
    bool wl_mid  = digitalRead(pinWL1)  == HIGH;
    bool wl_full = digitalRead(pinWL2)  == HIGH;

    // pack to b2b1b0 for abnormal check
    uint8_t pattern = (uint8_t(wl_full) << 2)
                    | (uint8_t(wl_mid)  << 1)
                    | (uint8_t(wl_low)  << 0);

    uint8_t lvl;
    // ─── only the four “normal” combos ───
    if (    wl_low  &&  wl_mid  &&  wl_full ) lvl = LEVEL_FULL;   // 111
    else if (wl_low  &&  wl_mid  && !wl_full ) lvl = LEVEL_MID;    // 011
    else if (wl_low  && !wl_mid  && !wl_full ) lvl = LEVEL_LOW;    // 001
    else if (!wl_low && !wl_mid  && !wl_full ) lvl = LEVEL_EMPTY;  // 000
    // ─── any other pattern → treat as empty ───
    else                                      lvl = LEVEL_LOW;

    // flag abnormal if not one of 000,001,011,111
    bool ab = !(pattern == 0b000 ||
                pattern == 0b001 ||
                pattern == 0b011 ||
                pattern == 0b111);

    waterState = lvl | (ab ? ABNORMAL_FLAG : 0);
}


// ─────── Helpers ───────
inline uint8_t level() { return waterState & LEVEL_MASK; }
inline bool abnormal() { return waterState & ABNORMAL_FLAG; }
} // namespace WaterLevel
