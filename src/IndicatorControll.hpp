// IndicatControll.hpp
#pragma once

#include <Arduino.h>
#include <FastLED.h>

#include "structures.h"
#include "WaterLevel.hpp"
// Change this if your LED strip is on a different pin:
#ifndef LED_PIN
#define LED_PIN 6
#endif
#ifndef BOARD_LED_PIN
#define BOARD_LED_PIN 16
#endif

#define NUM_LEDS 7
#define WATER_BAR_LEDS 4 // idx 0..3

namespace IndicatorControl {
inline CRGB leds[NUM_LEDS];
inline CRGB leds_board[1];

// threshold below which we consider “low water”

// flash state (unchanged)
inline bool flashActive = false;
inline uint8_t flashLedIndex = 0;
inline uint8_t flashCount = 0;
static constexpr uint8_t flashTotal = 4;
static constexpr unsigned long flashInterval = 100;
inline unsigned long flashLastToggle = 0;
inline bool flashPhase = false;

enum LedPos { BOTTOM = 0, LOWER_MID = 1, UPPER_MID = 2, TOP = 3 };

static constexpr unsigned long NORMAL_UPDATE_INTERVAL = 200;
static constexpr unsigned long SLOW_BLINK_INTERVAL = 1000;

inline unsigned long lastNormalUpdate = 0;

/// Power‐on test + brightness
inline void begin() {
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);

    FastLED.addLeds<WS2812B, BOARD_LED_PIN, GRB>(leds_board, 1);

    FastLED.setBrightness(32);
    for (int i = 0; i < NUM_LEDS; ++i) {
        leds[i] = CHSV((i * 255) / NUM_LEDS, 160, 255);
        leds_board[0] = CHSV((i * 255) / NUM_LEDS, 160, 255);
        FastLED.show();
        delay(150);
    }

    FastLED.show();
}

inline void startFlash(uint8_t idx) {
    flashActive = true;
    flashLedIndex = idx;
    flashCount = 0;
    flashPhase = false;
    flashLastToggle = millis();
}
// 1) Define the possible modes
enum Mode : uint8_t { IDLE, TRANSMITTING, RECEIVING, RUNNING };

// 2) Track the current mode and blink state
inline Mode currentMode = IDLE;
inline bool blinkState = false;
inline unsigned long blinkTS = 0;

inline unsigned long modeTS = 0;   // when we last changed mode
inline unsigned long modeHold = 0; // how long to stay in TX/RX
static constexpr unsigned long TRANSMIT_HOLD = 200;
static constexpr unsigned long RECEIVE_HOLD = 800;

inline void setMode(Mode m, unsigned long holdMs = 0) {
    currentMode = m;
    modeTS = millis();
    modeHold = holdMs;
    // if you immediately go into RUNNING, reset its blink timer too:
    if (m == RUNNING) {
        blinkState = false;
        blinkTS = modeTS;
    }
}

// 4) Call this at the top of your update() (or right before FastLED.show())
inline void updateBoardLED() {
    unsigned long now = millis();
    // auto-reset TX/RX back to RUNNING when timeout expires
    if ((currentMode == TRANSMITTING && now - modeTS >= modeHold) || (currentMode == RECEIVING && now - modeTS >= modeHold)) {
        setMode(RUNNING);
    }

    switch (currentMode) {
        case TRANSMITTING:
            leds_board[0] = CRGB::Aqua;
            break;
        case RECEIVING:
            leds_board[0] = CRGB::SandyBrown;
            break;
        case RUNNING:
            if (now - blinkTS >= SLOW_BLINK_INTERVAL) {
                blinkTS = now;
                blinkState = !blinkState;
            }
            leds_board[0] = blinkState ? CRGB::Green : CRGB::Black;
            break;
        default: // IDLE, or anything else
            leds_board[0] = CRGB::Black;
            break;
    }
}

inline void Transmitting() { setMode(TRANSMITTING, TRANSMIT_HOLD); }
inline void Receiving() { setMode(RECEIVING, RECEIVE_HOLD); }
inline void Running() { setMode(RUNNING); }

inline void update(DataPacket &data) {
    unsigned long now = millis();

    // 1) pump‐flash override
    if (flashActive && (now - flashLastToggle >= flashInterval)) {
        flashLastToggle = now;
        flashPhase = !flashPhase;
        if (!flashPhase && ++flashCount >= flashTotal) flashActive = false;
        leds[flashLedIndex] = flashPhase ? CRGB::Red : CRGB::Black;
        // FastLED.show();
    }

    // 2) every NORMAL_UPDATE_INTERVAL ms, redraw bar + status
    if (now - lastNormalUpdate < NORMAL_UPDATE_INTERVAL) return;
    lastNormalUpdate = now;

    // slow‐blink for warnings
    static unsigned long lastSlow = 0;
    static bool slowState = false;
    if (now - lastSlow >= SLOW_BLINK_INTERVAL) {
        lastSlow = now;
        slowState = !slowState;
    }

    // compute how many LEDs (0..4) should be lit:
    // (unchanged static bins)
    uint8_t numLit;
    if (data.waterLevel == WaterLevel::LEVEL_EMPTY)
        numLit = 0;
    else if (data.waterLevel == WaterLevel::LEVEL_LOW)
        numLit = 1;
    else if (data.waterLevel == WaterLevel::LEVEL_LOW && WaterLevel::LEVEL_MID)
        numLit = 2;
    else if (data.waterLevel == WaterLevel::LEVEL_MID)
        numLit = 3;
    else if (data.waterLevel == WaterLevel::LEVEL_FULL)
        numLit = WATER_BAR_LEDS;

    // determine “low‐water” using the shared threshold
    bool lowWater = (data.waterLevel == WaterLevel::LEVEL_EMPTY);
    bool HighWater = (data.waterLevel == WaterLevel::LEVEL_FULL);

    // mode/pump flags
    bool autoMode = data.status & PacketFlags::AUTO_MODE;
    bool p1 = data.status & PacketFlags::PUMP1;
    bool p2 = data.status & PacketFlags::PUMP2;

    // LED 6: AUTO
    leds[6] = autoMode ? CRGB::Green : CRGB::Black;

    // LED 5: PUMP1
    if (!(flashActive && flashLedIndex == 5)) {
        if (autoMode && lowWater && p1)
            leds[5] = slowState ? CRGB(255, 165, 0) : CRGB::Black;
        else
            leds[5] = p1 ? CRGB::Green : CRGB::Red;
    }

    // LED 4: PUMP2
    if (!(flashActive && flashLedIndex == 4)) {
        if (autoMode && lowWater && p2)
            leds[4] = slowState ? CRGB(255, 165, 0) : CRGB::Black;
        else
            leds[4] = p2 ? CRGB::Green : CRGB::Red;
    }

    // LEDs 0–3: water‐level bar (only when not flashing a pump LED)
    if (!flashActive) {
        if (lowWater) {
            leds[BOTTOM] = slowState ? CRGB::Red : CRGB::Black;
            for (uint8_t i = LOWER_MID; i <= TOP; ++i) leds[i] = CRGB::Black;
        } else if (HighWater) {
            leds[TOP] = slowState ? CRGB::Blue : CRGB::Black;
            for (uint8_t i = BOTTOM; i <= UPPER_MID; ++i) {
                if (i < numLit) {
                    float t = float(i) / float(TOP);
                    uint8_t r = (1.0f - t) * 255;
                    uint8_t g = t * 255;
                    leds[i] = CRGB(r, g, 0);
                } else {
                    leds[i] = CRGB::Black;
                }
            }
        } else {
            for (uint8_t i = BOTTOM; i <= TOP; ++i) {
                if (i < numLit) {
                    float t = float(i) / float(TOP);
                    uint8_t r = (1.0f - t) * 255;
                    uint8_t g = t * 255;
                    leds[i] = CRGB(r, g, 0);
                } else {
                    leds[i] = CRGB::Black;
                }
            }
        }
    }
    updateBoardLED();
    FastLED.show();
}

} // namespace IndicatorControl
