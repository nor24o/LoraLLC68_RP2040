// controller.hpp
#pragma once

#include <Arduino.h>
#include "IndicatorControll.hpp"  // for IndicatorControl::startFlash
#include "OneButton.h"
#include "WaterLevel.hpp"
#include "structures.h"

namespace PumpControl {

// === Control States ===
static bool cmdStopFillingPump = false;
static bool cmdStartFillingPump = false;

static uint8_t  currentWaterLevel    = 0;
// sentinel for “we haven’t seen a valid level yet”
static uint8_t  prevWaterLevel       = 0xFF;

static uint32_t WATER_GRACE_MS       = 60000ul;
static uint32_t lowWaterStartTime    = 0;
static bool     waterLowTimerRunning = false;

bool    autoMode   = false;
bool    pump1State = false;
bool    pump2State = false;

OneButton btnAuto(BTN_AUTO, false);
OneButton btnP1  (BTN_PUMP1, false);
OneButton btnP2  (BTN_PUMP2, false);

// === Interface ===
inline bool getIsGraceTimerRunning() { return waterLowTimerRunning; }

void setPump1(bool state) {
    pump1State = state;
    Serial.print("[PumpControl] Pump 1: ");
    Serial.println(state ? "ON" : "OFF");
    digitalWrite(PUMP1_PIN, state ? LOW : HIGH);
}
void setPump2(bool state) {
    pump2State = state;
    Serial.print("[PumpControl] Pump 2: ");
    Serial.println(state ? "ON" : "OFF");
    digitalWrite(PUMP2_PIN, state ? LOW : HIGH);
}
inline void stopAll() {
    setPump1(false);
    setPump2(false);
}

void updatelocalstates(const DataPacket &data) {
    pump1State = !!(data.status & PacketFlags::PUMP1);
    pump2State = !!(data.status & PacketFlags::PUMP2);
    autoMode   = !!(data.status & PacketFlags::AUTO_MODE);
    setPump1(pump1State);
    setPump2(pump2State);
    Serial.printf("[PumpControl] Local states: P1:%s P2:%s Auto:%s\n",
                  pump1State ? "ON" : "OFF",
                  pump2State ? "ON" : "OFF",
                  autoMode   ? "EN" : "OFF");
}

void setAutoMode(bool f) {
    autoMode = f;
    Serial.printf("[PumpControl] Auto mode %s\n", f ? "ENABLED" : "DISABLED");
}
void toggleAutoMode() { setAutoMode(!autoMode); }

// === Initialization ===
void begin() {
    pinMode(PUMP1_PIN, OUTPUT);
    pinMode(PUMP2_PIN, OUTPUT);
    digitalWrite(PUMP1_PIN, HIGH);
    digitalWrite(PUMP2_PIN, HIGH);

    pinMode(BTN_AUTO, INPUT_PULLDOWN);
    pinMode(BTN_PUMP1, INPUT_PULLDOWN);
    pinMode(BTN_PUMP2, INPUT_PULLDOWN);
    // Pump1 manual & long-press
    btnP1.setLongPressIntervalMs(10000);
    btnP1.setClickMs(2000);

    btnP1.setPressMs(2000);
    btnP2.setPressMs(2000);

    // Auto button
    btnAuto.setLongPressIntervalMs(10000);
    btnAuto.setClickMs(2000);
    btnAuto.setPressMs(2000);

    // Auto toggle
    btnAuto.attachClick([](){
        toggleAutoMode();
    });



    btnP1.attachClick([](){
        if (!autoMode || currentWaterLevel != WaterLevel::LEVEL_EMPTY) {
            setPump1(!pump1State);
        } else {
            Serial.println("[PumpControl] Pump1 blocked: Auto mode + low water");
            IndicatorControl::startFlash(5);
            if (pump1State) setPump1(false);
        }
    });
    btnP1.attachLongPressStart([](){
        // **manual** STOP_FILL, immediate
        cmdStopFillingPump = true;
        Serial.println("[PumpControl] Long-press P1 → manual STOP_FILL");
    });

    // Pump2 manual & long-press
    btnP2.setLongPressIntervalMs(10000);
    btnP2.setClickMs(500);
    btnP2.attachClick([](){
        if (!autoMode || currentWaterLevel != WaterLevel::LEVEL_EMPTY) {
            setPump2(!pump2State);
        } else {
            Serial.println("[PumpControl] Pump2 blocked: Auto mode + low water");
            IndicatorControl::startFlash(4);
            if (pump2State) setPump2(false);
        }
    });
    btnP2.attachLongPressStart([](){
        // **manual** START_FILL, immediate
        cmdStartFillingPump = true;
        Serial.println("[PumpControl] Long-press P2 → manual START_FILL");
    });
}

// === Runtime update ===
void update(DataPacket &data) {
    // 1) Poll buttons
    btnAuto.tick();
    btnP1.tick();
    btnP2.tick();

    // 2) Refresh water level
    currentWaterLevel = data.waterLevel;

    // 3) Automatic one-shot on threshold crossings (auto mode only)
    if (autoMode) {
        // on EMPTY transition
        if (currentWaterLevel == WaterLevel::LEVEL_EMPTY
            && prevWaterLevel != WaterLevel::LEVEL_EMPTY) {
            cmdStartFillingPump = true;
            Serial.println("[PumpControl] Auto: Water EMPTY → START_FILL");
        }
        // on FULL transition
        if (currentWaterLevel == WaterLevel::LEVEL_FULL
            && prevWaterLevel != WaterLevel::LEVEL_FULL) {
            cmdStopFillingPump = true;
            Serial.println("[PumpControl] Auto: Water FULL → STOP_FILL");
        }
    }

    // 4) Grace-timer when low
    if (autoMode && currentWaterLevel == WaterLevel::LEVEL_EMPTY) {
        if (!waterLowTimerRunning) {
            waterLowTimerRunning = true;
            lowWaterStartTime = millis();
            Serial.println("[PumpControl] Water LOW → starting grace timer");
        } else if (millis() - lowWaterStartTime >= WATER_GRACE_MS) {
            stopAll();
            waterLowTimerRunning = false;
            Serial.println("[PumpControl] Grace expired → pumps stopped");
        }
    } else {
        if (waterLowTimerRunning) {
            waterLowTimerRunning = false;
            Serial.println("[PumpControl] Water OK → canceled grace timer");
        }
    }

    // remember for next loop
    prevWaterLevel = currentWaterLevel;

    // 5) Pack bits into outgoing packet
    uint16_t st = data.status;
    // auto-mode bit
    if (autoMode) st |=  PacketFlags::AUTO_MODE;
    else          st &= ~PacketFlags::AUTO_MODE;
    // pump bits
    if (pump1State) st |= PacketFlags::PUMP1; else st &= ~PacketFlags::PUMP1;
    if (pump2State) st |= PacketFlags::PUMP2; else st &= ~PacketFlags::PUMP2;
    // other-station commands
    if (cmdStopFillingPump) {
        st |=  PacketFlags::OTHER_STATION_PUMP;
        st &= ~PacketFlags::OTHER_STATION_STARTSTOP_FILL;
        cmdStopFillingPump = false;
    }
    if (cmdStartFillingPump) {
        st |=  PacketFlags::OTHER_STATION_PUMP;
        st |=  PacketFlags::OTHER_STATION_STARTSTOP_FILL;
        cmdStartFillingPump = false;
    }
    data.status = st;
}

}  // namespace PumpControl
