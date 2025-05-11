// controller.hpp
#pragma once

#include <Arduino.h>
#include "OneButton.h"
#include "structures.h"
#include "IndicatControll.hpp"  // only for IndicatorControl::startFlash if you still use it

namespace PumpControl
{
    // === Control States ===
    static bool cmdStopFillingPump = false;
    static bool cmdStartFillingPump = false;

    static uint8_t currentWaterLevel    = 0;
    static uint8_t waterLevelThreshold  = 15;       // raw 0–255
    static uint32_t WATER_GRACE_MS       = 60000ul;

    static uint32_t lowWaterStartTime    = 0;
    static bool     waterLowTimerRunning = false;

    bool autoMode    = false;
    bool pump1State  = false;
    bool pump2State  = false;
    byte flags       = 0; // Bitwise flags

    OneButton btnAuto(BTN_AUTO, false);
    OneButton btnP1  (BTN_PUMP1, false);
    OneButton btnP2  (BTN_PUMP2, false);

    // === Interface ===
    inline void setWaterLevelThreshold(uint8_t threshold) {
        waterLevelThreshold = threshold;
    }
    inline uint8_t getWaterLevelThreshold() {
        return waterLevelThreshold;
    }
    inline bool getIsGraceTimerRunning() {
        return waterLowTimerRunning;
    }

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
            pump1State?"ON":"OFF", pump2State?"ON":"OFF", autoMode?"EN":"OFF");
    }

    void setAutoMode(bool f) {
        autoMode = f;
        Serial.printf("[PumpControl] Auto mode %s\n", f?"ENABLED":"DISABLED");
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

        btnAuto.attachClick([](){
            toggleAutoMode();
        });

        btnP1.attachClick([](){
            // allow pump only if not auto OR water above threshold
            if (!autoMode || currentWaterLevel > waterLevelThreshold) {
                setPump1(!pump1State);
            } else {
                Serial.println("[PumpControl] Pump1 blocked: Auto mode + low water");
                IndicatorControl::startFlash(5); // flash LED 4 (PUMP1)
                if (pump1State) {
                    setPump1(false);
                }
            }
        });
        btnP1.attachLongPressStart([](){
            cmdStopFillingPump = true;
            Serial.println("[PumpControl] Long press P1 → STOP FILL COMMAND");
        });

        btnP2.attachClick([](){
            if (!autoMode || currentWaterLevel > waterLevelThreshold) {
                setPump2(!pump2State);
            } else {
                Serial.println("[PumpControl] Pump2 blocked: Auto mode + low water");
                IndicatorControl::startFlash(4); // flash LED 5
                if (pump2State) {
                    setPump2(false);
                }
            }
        });
        btnP2.attachLongPressStart([](){
            cmdStartFillingPump = true;
            Serial.println("[PumpControl] Long press P2 → START FILL COMMAND");
        });
    }

    // === Runtime update ===
    void update(DataPacket &data) {
        btnAuto.tick();
        btnP1.tick();
        btnP2.tick();

        currentWaterLevel = data.waterLevel;

        if (autoMode && (pump1State || pump2State)) {
            if (currentWaterLevel <= waterLevelThreshold) {
                // water low → start grace timer
                if (!waterLowTimerRunning) {
                    waterLowTimerRunning = true;
                    lowWaterStartTime    = millis();
                    Serial.println("[PumpControl] Water LOW → starting grace timer");
                } else if (millis() - lowWaterStartTime >= WATER_GRACE_MS) {
                    stopAll();
                    waterLowTimerRunning = false;
                    Serial.println("[PumpControl] Grace expired → pumps stopped");
                }
            } else {
                // water OK → cancel any running grace timer
                if (waterLowTimerRunning) {
                    waterLowTimerRunning = false;
                    Serial.println("[PumpControl] Water OK → canceled grace timer");
                }
            }
        }

        // build status flags
        uint16_t st = 0;
        if (autoMode)     st |= PacketFlags::AUTO_MODE;
        if (pump1State)   st |= PacketFlags::PUMP1;
        if (pump2State)   st |= PacketFlags::PUMP2;
        if (cmdStopFillingPump) { st |= PacketFlags::STOP_FILL;  cmdStopFillingPump = false; }
        if (cmdStartFillingPump){ st |= PacketFlags::START_FILL; cmdStartFillingPump = false; }

        data.status = st;
    }

} // namespace PumpControl
