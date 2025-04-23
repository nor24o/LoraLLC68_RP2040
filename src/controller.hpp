#pragma once

#include <Arduino.h>
#include "OneButton.h"
#include "structures.h"

// === Pins ===
#define PUMP1_PIN 12
#define PUMP2_PIN 11

#define BTN_AUTO 8
#define BTN_PUMP1 9
#define BTN_PUMP2 10

namespace PumpControl
{

  // === Control States ===
  static bool cmdStopFillingPump = false;
  static bool cmdStartFillingPump = false;

  static uint16_t currentWaterLevel = 0;
  static uint16_t waterLevelThreshold = 1500;
  static uint32_t WATER_GRACE_MS = 60000;

  static uint32_t lowWaterStartTime = 0;
  static bool waterLowTimerRunning = false;

  bool autoMode = true;
  bool pump1State = true;
  bool pump2State = false;
  byte flags = 0; // Bitwise flags for various states

  OneButton btnAuto(BTN_AUTO, true); // true = active LOW (pull-up)
  OneButton btnP1(BTN_PUMP1, true);
  OneButton btnP2(BTN_PUMP2, true);

  // === Interface ===
  void setWaterLevelThreshold(uint8_t threshold) { waterLevelThreshold = threshold; }
  void setWaterWATER_GRACE_MS(uint32_t grace_ms) { WATER_GRACE_MS = grace_ms; }

  void setPump1(bool state)
  {
    pump1State = state;
    digitalWrite(PUMP1_PIN, state ? HIGH : LOW);
  }

  void setPump2(bool state)
  {
    pump2State = state;
    digitalWrite(PUMP2_PIN, state ? HIGH : LOW);
  }

  void stopAll()
  {
    setPump1(false);
    setPump2(false);
  }

  bool getPump1State() { return pump1State; }
  bool getPump2State() { return pump2State; }
  bool isAutoMode() { return autoMode; }
  byte getFlags() { return flags; }

  // === Initialization ===
  void begin()
  {
    pinMode(PUMP1_PIN, OUTPUT);
    pinMode(PUMP2_PIN, OUTPUT);
    digitalWrite(PUMP1_PIN, LOW);
    digitalWrite(PUMP2_PIN, LOW);

    pinMode(BTN_AUTO, INPUT_PULLUP);
    pinMode(BTN_PUMP1, INPUT_PULLUP);
    pinMode(BTN_PUMP2, INPUT_PULLUP);

    // AUTO MODE toggle
    btnAuto.attachClick([]()
                        {
    autoMode = !autoMode;
    Serial.print("[PumpControl] Auto mode is now ");
    Serial.println(autoMode ? "ENABLED" : "DISABLED"); });

    // Pump 1 short press: toggle
    btnP1.attachClick([]()
                      {
    if (!autoMode || currentWaterLevel < waterLevelThreshold) {
      pump1State = !pump1State;
      digitalWrite(PUMP1_PIN, pump1State ? HIGH : LOW);
      Serial.print("[PumpControl] Pump 1: ");
      Serial.println(pump1State ? "ON" : "OFF");
    } else {
      Serial.println("[PumpControl] Pump 1 blocked: Auto mode + No water");
    } });

    // Pump 1 long press: stop filling
    btnP1.attachLongPressStart([]()
                               {
    Serial.println("[PumpControl] Long press on Pump 1 → STOP FILLING COMMAND");
    cmdStopFillingPump = true; });

    // Pump 2 short press: toggle
    btnP2.attachClick([]()
                      {
    if (!autoMode || currentWaterLevel < waterLevelThreshold) {
      pump2State = !pump2State;
      digitalWrite(PUMP2_PIN, pump2State ? HIGH : LOW);
      Serial.print("[PumpControl] Pump 2: ");
      Serial.println(pump2State ? "ON" : "OFF");
    } else {
      Serial.println("[PumpControl] Pump 2 blocked: Auto mode + No water");
    } });

    // Pump 2 long press: start filling
    btnP2.attachLongPressStart([]()
                               {
    Serial.println("[PumpControl] Long press on Pump 2 → START FILLING COMMAND");
    cmdStartFillingPump = true; 
  });
  }

  // === Runtime update ===
  void update(DataPacket &data)
  {
    btnAuto.tick();
    btnP1.tick();
    btnP2.tick();

    currentWaterLevel = data.waterLevel;

    if (autoMode)
    {
      // Only care about stop condition
      if (pump1State || pump2State)
      {
        if (currentWaterLevel >= waterLevelThreshold)
        {
          // Tank is empty → start grace timer
          if (!waterLowTimerRunning)
          {
            waterLowTimerRunning = true;
            lowWaterStartTime = millis();
            Serial.println("[PumpControl] Tank is empty → starting grace timer.");
          }
          // Grace period expired → stop pumps
          else if (millis() - lowWaterStartTime >= WATER_GRACE_MS)
          {
            stopAll();
            waterLowTimerRunning = false;
            Serial.println("[PumpControl] Grace expired → pumps stopped.");
          }
        }
        // Cancel grace timer if water came back up
        else if (waterLowTimerRunning)
        {
          waterLowTimerRunning = false;
          Serial.println("[PumpControl] Tank not empty anymore → canceled grace timer.");
        }
      }
    }

    // === Update outgoing packet ===
    bitWrite(data.IO_O, 0, pump1State); // Bit 0: Pump 1 state
    bitWrite(data.IO_O, 1, pump2State); // Bit 1: Pump 2 state

    bitWrite(data.flags, 0, autoMode); // Bit 0: auto mode

    if (cmdStopFillingPump)
    {
      bitWrite(data.flags, 7, BIT_ON); // Bit 0: stop filling command
      bitWrite(data.flags, 6, BIT_OFF); // Bit 6: start filling command
      cmdStopFillingPump = false; // Reset command flag
    }
    if (cmdStartFillingPump)
    {
      bitWrite(data.flags, 7, BIT_ON);  // Bit 0: stop filling command
      bitWrite(data.flags, 6, BIT_ON);  // Bit 6: start filling command
      cmdStartFillingPump = false; // Reset command flag
    }
  }
}
