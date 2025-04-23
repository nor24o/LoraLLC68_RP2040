#pragma once

#include <Arduino.h>
#include <FastLED.h>


// === Pins ===
#define LED_PIN 16 // RP2040-Zero onboard LED

#define NUM_LEDS 1 // Only one LED

namespace IndicatorControl
{

    CRGB leds[NUM_LEDS];
    unsigned long ledStartTime = 0;
    unsigned long ledDuration = 0;
    bool ledOn = false;

    // Variables for LED blinking
    unsigned long lastBlinkTime = 0;
    unsigned long blinkInterval = 0;
    unsigned int blinkCount = 0;
    unsigned int currentBlink = 0;
    bool isBlinking = false;
    CRGB blinkColor;

    // Call this function to turn LED on for a set time
    void setLED(CRGB color, unsigned long duration)
    {
        leds[0] = color;
        FastLED.show();
        ledOn = true;
        ledStartTime = millis();
        ledDuration = duration;
    }

    // Runs in `loop()` to manage LED timing
    void handleLED()
    {
        if (ledOn && millis() - ledStartTime >= ledDuration)
        {
            leds[0] = CRGB::Black; // Turn off LED
            FastLED.show();
            ledOn = false;
            isBlinking = false;
        }

        if (isBlinking)
        {
            if (millis() - lastBlinkTime >= blinkInterval)
            {
                leds[0] = (leds[0] == CRGB::Black) ? blinkColor : CRGB::Black;
                FastLED.show();
                lastBlinkTime = millis();
                currentBlink++;

                if (currentBlink >= blinkCount * 2) // On + off = 2 steps per blink
                {
                    isBlinking = false;
                    leds[0] = CRGB::Black; // Ensure LED is off after blinking
                    FastLED.show();
                }
            }
        }
    }
    void setLEDBlink(CRGB color, unsigned long duration, unsigned long interval, unsigned int count)
    {
        leds[0] = color;
        FastLED.show();
        ledOn = true;
        ledStartTime = millis();
        ledDuration = duration;

        // Set blinking parameters
        blinkColor = color;
        blinkInterval = interval;
        blinkCount = count;
        currentBlink = 0;
        lastBlinkTime = millis();
        isBlinking = true;
    }
    void indicatorSetup()
    {
        // Initialize FastLED
        FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
        leds[0] = CRGB::Blue;      // Initial LED color (Blue = Normal Mode)
        FastLED.setBrightness(32); // Range: 0 (off) to 255 (full brightness)
        FastLED.show();
    }

    void begin()
    {
        indicatorSetup();
    }
    void update()
    {
        handleLED();
    }

}