#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <HardwareSerial.h>
#include "config.h"
#include "imu_receiver.h"

// MVP: button timestamps are captured in the foreground loop.
// For validated sub-millisecond timing use GPIO interrupts and calibrate cue onset.
WebServer web(80);
HardwareSerial player(2);

struct ButtonState {
    bool stable = HIGH;
    bool raw = HIGH;
    uint32_t changedUs = 0;
};
ButtonState buttons[BUTTON_COUNT];

enum class Stage : uint8_t { Idle, BeforeCue, Waiting, Interval };
Stage stage = Stage::Idle;
String source = "screen";
String exercise = "random";
uint16_t goal = 0;
uint16_t completed = 0;
uint8_t fixedTarget = 1;
uint8_t target = 0;
uint32_t cueUs = 0;
uint32_t stageAtMs = 0;
uint32_t eventSequence = 0;
String lastEvent = "{\"seq\":0}";
// Competition: all players complete identical target sequences in separate turns.
bool competition = false;
uint8_t playerCount = 0;
uint8_t currentPlayer = 0;
uint8_t roundCount = 0;
uint8_t commonTargets[30] = {};
struct Score { uint32_t sumMs = 0; uint16_t correct = 0; uint16_t errors = 0; };
Score scores[4];
String playerNames[4] = {"Игрок 1", "Игрок 2", "Игрок 3", "Игрок 4"};
bool waitingForNextPlayer = false;


void json(const String &s, int status = 200) {
    web.send(status, "application/json; charset=utf-8", s);
}

void dfCommand(uint8_t command, uint16_t parameter) {
    // DFPlayer serial protocol: 7E FF 06 CMD 00 PARAM_H PARAM_L CHK_H CHK_L EF
    uint16_t sum = static_cast<uint16_t>(0xFF + 0x06 + command +
                                         (parameter >> 8) + (parameter & 0xFF));
    uint16_t checksum = static_cast<uint16_t>(0 - sum);
    uint8_t frame[10] = {0x7E, 0xFF, 0x06, command, 0x00,
                         static_cast<uint8_t>(parameter >> 8),
                         static_cast<uint8_t>(parameter & 0xFF),
                         static_cast<uint8_t>(checksum >> 8),
                         static_cast<uint8_t>(checksum & 0xFF), 0xEF};
    player.write(frame, sizeof(frame));
}

bool allReleased() {
    for (const ButtonState &b : buttons) {
        if (b.stable == LOW || b.raw == LOW) return false;
    }
    return true;
}

void scheduleNext() {
    if (completed >= goal) {
        stage = Stage::Idle;
        target = 0;
        if (competition) waitingForNextPlayer = currentPlayer + 1 < playerCount;
        return;
    }
    if (competition) target = commonTargets[completed];
    else target = exercise == "single" ? fixedTarget :
             exercise == "sequence" ? static_cast<uint8_t>((completed % 9) + 1) :
             static_cast<uint8_t>(random(1, 10));
    stageAtMs = millis() + static_cast<uint32_t>(
        random(CUE_DELAY_MIN_MS, CUE_DELAY_MAX_MS + 1));
    stage = Stage::BeforeCue;
}

void issueCue() {
    // This is command dispatch time, NOT verified loudspeaker/screen onset.
    cueUs = micros();
    imuMarkCue(cueUs);
    if (source == "audio") dfCommand(0x12, target); // /MP3/0001.mp3 ... 0009.mp3
    stage = Stage::Waiting;
    stageAtMs = millis();
}

void finishTrial(uint8_t pressed, uint32_t pressUs, bool timedOut) {
    if (stage != Stage::Waiting) return;
    uint8_t expected = target;
    float elapsed = timedOut ? -1.0f :
                              static_cast<uint32_t>(pressUs - cueUs) / 1000.0f;
    bool correct = !timedOut && pressed == expected;
    imuMarkPress(timedOut ? micros() : pressUs);
    ++completed;
    ++eventSequence;
    if (competition) {
        if (correct) {
            scores[currentPlayer].correct++;
            scores[currentPlayer].sumMs += static_cast<uint32_t>(elapsed + 0.5f);
        } else {
            scores[currentPlayer].errors++;
        }
    }

    lastEvent = "{\"seq\":" + String(eventSequence) +
                ",\"type\":\"" + (timedOut ? String("timeout") : String("press")) +
                "\",\"target\":" + String(expected) +
                ",\"pressed\":" + String(pressed) +
                ",\"correct\":" + (correct ? String("true") : String("false")) +
                ",\"elapsed_ms\":" + String(elapsed, 2) +
                ",\"completed\":" + String(completed) +
                ",\"goal\":" + String(goal) +
                ",\"player\":" + String(competition ? currentPlayer + 1 : 0) + "}";
    Serial.println(lastEvent);
    stage = Stage::Interval;
    stageAtMs = millis() + BETWEEN_TRIALS_MS;
}

void scanButtons() {
    uint32_t now = micros();
    for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
        bool raw = digitalRead(BUTTON_PINS[i]);
        ButtonState &b = buttons[i];
        if (raw != b.raw) {
            b.raw = raw;
            b.changedUs = now;
        }
        if (raw != b.stable &&
            static_cast<uint32_t>(now - b.changedUs) >= DEBOUNCE_US) {
            b.stable = raw;
            if (raw == LOW) {
                // Use the first stable edge timestamp, before debounce delay.
                finishTrial(i + 1, b.changedUs, false);
            }
        }
    }
}

void tickSession() {
    uint32_t now = millis();
    if (stage == Stage::BeforeCue &&
        static_cast<int32_t>(now - stageAtMs) >= 0) {
        // Held buttons must not become a new trial.
        if (allReleased()) issueCue();
    } else if (stage == Stage::Waiting &&
               static_cast<uint32_t>(now - stageAtMs) >= RESPONSE_TIMEOUT_MS) {
        finishTrial(0, micros(), true);
    } else if (stage == Stage::Interval &&
               static_cast<int32_t>(now - stageAtMs) >= 0 &&
               allReleased()) {
        scheduleNext();
    }
}

void handleStatus() {
    bool active = stage != Stage::Idle;
    bool waiting = stage == Stage::Waiting;
    // Hide target in audio mode to avoid revealing spoken response visually.
    uint8_t visibleTarget = (waiting && source == "screen") ? target : 0;
    String body = "{\"active\":" + String(active ? "true" : "false") +
                  ",\"waiting\":" + String(waiting ? "true" : "false") +
                  ",\"target\":" + String(visibleTarget) +
                  ",\"mode\":\"" + exercise + "\",\"source\":\"" + source +
                  "\",\"completed\":" + String(completed) +
                  ",\"goal\":" + String(goal) +
                  ",\"event_seq\":" + String(eventSequence) +
                  ",\"competition\":" + String(competition ? "true" : "false") +
                  ",\"player\":" + String(competition ? currentPlayer + 1 : 0) +
                  ",\"players\":" + String(playerCount) +
                  ",\"awaiting_next\":" + String(waitingForNextPlayer ? "true" : "false") + "}";
    json(body);
}

void handleStart() {
    if (stage != Stage::Idle) {
        json("{\"error\":\"Сначала остановите текущую серию\"}", 409);
        return;
    }
    if (!allReleased()) {
        json("{\"error\":\"Отпустите все кнопки\"}", 409);
        return;
    }
    String requestedMode = web.arg("mode");
    String requestedSource = web.arg("source");
    if (requestedMode != "single" && requestedMode != "random" &&
        requestedMode != "sequence") {
        json("{\"error\":\"Неверный режим\"}", 400);
        return;
    }
    if (requestedSource != "screen" && requestedSource != "audio") {
        json("{\"error\":\"Неверный источник команды\"}", 400);
        return;
    }
    int count = web.arg("count").toInt();
    int selected = web.arg("target").toInt();
    if (count < 1 || count > 100 || selected < 1 || selected > 9) {
        json("{\"error\":\"Допустимо 1–100 заданий и кубы 1–9\"}", 400);
        return;
    }
    competition = false;
    waitingForNextPlayer = false;
    playerCount = 0;
    exercise = requestedMode;
    source = requestedSource;
    fixedTarget = static_cast<uint8_t>(selected);
    goal = exercise == "single" ? 1 : static_cast<uint16_t>(count);
    completed = 0;
    scheduleNext();
    json("{\"ok\":true}");
}

void handleCompetitionStart() {
    if (stage != Stage::Idle) {
        json("{\"error\":\"Сначала остановите текущую серию\"}", 409);
        return;
    }
    if (!allReleased()) {
        json("{\"error\":\"Отпустите кнопки\"}", 409);
        return;
    }
    int players = web.arg("players").toInt();
    int rounds = web.arg("rounds").toInt();
    String requestedSource = web.arg("source");
    if (players < 1 || players > 4 || rounds < 1 || rounds > 30 ||
        (requestedSource != "screen" && requestedSource != "audio")) {
        json("{\"error\":\"Укажите 1–4 игроков, 1–30 заданий и источник сигнала\"}", 400);
        return;
    }
    competition = true;
    waitingForNextPlayer = false;
    playerCount = static_cast<uint8_t>(players);
    currentPlayer = 0;
    roundCount = static_cast<uint8_t>(rounds);
    source = requestedSource;
    exercise = "competition";
    for (uint8_t i = 0; i < 4; ++i) {
        scores[i] = Score();
        String name = web.arg("name" + String(i + 1));
        name.trim();
        // Names are not injected into JSON: the browser holds editable names.
        playerNames[i] = name.length() ? name.substring(0, 24) : "Игрок " + String(i + 1);
    }
    for (uint8_t i = 0; i < roundCount; ++i) {
        commonTargets[i] = static_cast<uint8_t>(random(1, 10));
    }
    completed = 0;
    goal = roundCount;
    scheduleNext();
    json("{\"ok\":true}");
}

void handleCompetitionNext() {
    if (!competition || !waitingForNextPlayer || stage != Stage::Idle ||
        currentPlayer + 1 >= playerCount) {
        json("{\"error\":\"Следующий участник пока недоступен\"}", 409);
        return;
    }
    if (!allReleased()) {
        json("{\"error\":\"Отпустите все кнопки\"}", 409);
        return;
    }
    currentPlayer++;
    completed = 0;
    goal = roundCount;
    waitingForNextPlayer = false;
    scheduleNext();
    json("{\"ok\":true}");
}

void handleCompetitionScores() {
    String body = "{\"competition\":" + String(competition ? "true" : "false") +
                  ",\"players\":" + String(playerCount) + ",\"scores\":[";
    for (uint8_t i = 0; i < playerCount; ++i) {
        if (i) body += ",";
        body += "{\"player\":" + String(i + 1) +
                ",\"correct\":" + String(scores[i].correct) +
                ",\"errors\":" + String(scores[i].errors) +
                ",\"sum_ms\":" + String(scores[i].sumMs) + "}";
    }
    body += "]}";
    json(body);
}

void serveIndex() {
    File file = LittleFS.open("/index.html", "r");
    if (!file) {
        web.send(500, "text/plain; charset=utf-8",
                 "Нет index.html. Выполните: pio run -t uploadfs");
        return;
    }
    web.streamFile(file, "text/html; charset=utf-8");
    file.close();
}

void setup() {
    Serial.begin(115200);
    for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);
        buttons[i].raw = digitalRead(BUTTON_PINS[i]);
        buttons[i].stable = buttons[i].raw;
        buttons[i].changedUs = micros();
    }

    player.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
    delay(800); // Let DFPlayer boot before setting volume.
    dfCommand(0x06, DFPLAYER_VOLUME);

    if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    imuReceiverBegin();
    randomSeed(esp_random());

    web.on("/", HTTP_GET, serveIndex);
    web.on("/api/status", HTTP_GET, handleStatus);
    web.on("/api/event", HTTP_GET, []() { json(lastEvent); });
    web.on("/api/start", HTTP_POST, handleStart);
    web.on("/api/imu", HTTP_GET, []() { json(imuStatusJson()); });
    web.on("/api/imu/trial", HTTP_GET, []() { json(imuTrialJson()); });
    web.on("/api/competition/start", HTTP_POST, handleCompetitionStart);
    web.on("/api/competition/next", HTTP_POST, handleCompetitionNext);
    web.on("/api/competition/scores", HTTP_GET, handleCompetitionScores);
    web.on("/api/stop", HTTP_POST, []() {
        stage = Stage::Idle;
        target = 0;
        competition = false;
        waitingForNextPlayer = false;
        json("{\"ok\":true}");
    });
    web.begin();
    Serial.print("Open http://");
    Serial.println(WiFi.softAPIP());
}

void loop() {
    imuReceiverPoll();
    scanButtons();
    tickSession();
    web.handleClient();
}
