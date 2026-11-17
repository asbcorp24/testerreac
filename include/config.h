#pragma once
#include <Arduino.h>

// ESP32-WROOM-32 DevKit V1; one end of each NO button goes to GND.
static constexpr uint8_t BUTTON_COUNT = 9;
static constexpr uint8_t BUTTON_PINS[BUTTON_COUNT] = {
    13, 14, 16, 17, 18, 19, 21, 22, 23
};

// DFPlayer TX -> ESP32 RX GPIO26; ESP32 TX GPIO25 -> DFPlayer RX.
// A 1 kOhm series resistor on ESP32 TX is recommended.
static constexpr int DFPLAYER_RX_PIN = 26;
static constexpr int DFPLAYER_TX_PIN = 25;
static constexpr uint8_t DFPLAYER_VOLUME = 23; // 0..30

static constexpr char WIFI_SSID[] = "ReactionTrainer";
static constexpr char WIFI_PASSWORD[] = "reaction123";
static constexpr uint32_t DEBOUNCE_US = 18000;
static constexpr uint32_t RESPONSE_TIMEOUT_MS = 10000;
static constexpr uint32_t BETWEEN_TRIALS_MS = 1100;
static constexpr uint32_t CUE_DELAY_MIN_MS = 900;
static constexpr uint32_t CUE_DELAY_MAX_MS = 1700;
