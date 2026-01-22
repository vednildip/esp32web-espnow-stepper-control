// Slave  for  Now_FlexiStepper_master_callback_05_task_
#include <WebServer.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_now.h>
#include <ESP_FlexyStepper.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <DHT.h>
#include <DHT_U.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <esp_wifi.h>

// --- PINS & TYPES ---
#define STEP_PIN      17
#define DIR_PIN       16
#define EN_PIN        18
#define DHTPIN        5
#define DHTTYPE       DHT11

// ----------------- DEBUG CONTROL -----------------
#define DEBUG 1  // 1 = enable debug prints, 0 = disable

#if DEBUG == 0
  // Dummy Serial class that swallows all print calls including begin()
  class DummySerialClass {
  public:
      void begin(unsigned long) {}  // swallow Serial.begin
      template<typename T> DummySerialClass& print(const T&) { return *this; }
      template<typename T> DummySerialClass& println(const T&) { return *this; }
      template<typename... T> DummySerialClass& printf(const char*, T...) { return *this; }
  };
  static DummySerialClass DummySerial;
  #define Serial DummySerial
#endif
// --------------------------------------------------

// --- Wi‑Fi & Host ---
const char* ssid      = "XXXXXXXX";
const char* password  = "XXXXXXXXXXXXX";
const char* host      = "esp32";

// --- Objects & State ---
DHT_Unified dht(DHTPIN, DHTTYPE);
bool allowEmergencyOverride = false;  // true if server says: ignore temp stop
ESP_FlexyStepper stepper;
bool autoRunEnabled = false;
uint8_t mac[6];
uint8_t masterAddress[] = {0x24, 0x58, 0x7C, 0xD3, 0x5F, 0x9C};

float latestTemp = -1000.0f; // sentinel;
float latestHumid = 0;
bool handshakeDone = false;
bool running = false;
bool wifiLost = false;
bool espNowLost = false;

// Stepper parameters
int speedSteps          = 1350;
int accelSteps          = 500;
int decelSteps          = 500;
int distance            = 75000;
int delayBetweenCycles  = 5000;
bool direction          = false;

// Update flags
bool updateSpeedPending     = false;
bool updateAccelPending     = false;
bool updateDecelPending     = false;
bool updateDistancePending  = false;
uint8_t currentChannel = 0;

typedef struct { char label[10]; int value; } Command;
static QueueHandle_t rxQueue = nullptr, txQueue = nullptr;
// flag set when ack for handshake arrives
volatile bool handshakeAck = false;
// Live tracking
static long startPos = 0, lastPos = 0, lastDistanceSent = -1;
static int lastSpeedSent = -1;
static unsigned long lastLiveSent = 0, lastTime = 0;

#define CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS 1
// Helper
static inline unsigned long nowMillis() { return millis(); }

void queueSend(const char* lbl, int val) {
  if (!txQueue) return;
  Command o{};
  strncpy(o.label, lbl, sizeof(o.label)-1);
  o.value = val;
  if (xQueueSend(txQueue, &o, 0) != pdTRUE) {
    Serial.printf("[WARN] TX queue full, dropped: %s=%d\n", o.label, o.value);
  }
}

void acknowledge(const char* l, int v) { queueSend(l, v); }
void sendLive(const char* l, long v) { queueSend(l, (int)v); }

// --- Step Control ---
void startMotion() {
  stepper.setSpeedInStepsPerSecond(speedSteps);
  stepper.setAccelerationInStepsPerSecondPerSecond(accelSteps);
  stepper.setDecelerationInStepsPerSecondPerSecond(decelSteps);

  long mv = (direction ? 1 : -1) * (long)distance;
  stepper.setTargetPositionRelativeInSteps(mv);

  digitalWrite(EN_PIN, LOW);
  running = true;

  startPos = stepper.getCurrentPositionInSteps();
  lastPos = startPos;
  lastDistanceSent = -1;
  lastSpeedSent = -1;
  lastLiveSent = 0;
  lastTime = nowMillis();

  Serial.printf("[MOTION] Start: %s, Steps: %ld\n", direction ? "Forward" : "Reverse", mv);
}

void stopMotion() {
  stepper.emergencyStop();
  digitalWrite(EN_PIN, HIGH);
  running = false;

  if (lastSpeedSent != 0) {
    sendLive("speedLive", 0);
    lastSpeedSent = 0;
  }

  long finalPos = stepper.getCurrentPositionInSteps();
  long traveled = abs(finalPos - startPos);

  if (traveled != lastDistanceSent) {
    sendLive("distanceLive", traveled);
    lastDistanceSent = traveled;
  }

  Serial.println("[MOTION] Stopped.");
}

// --- DHT Handling ---
void captureTempHumid() {
  sensors_event_t evt;
  dht.temperature().getEvent(&evt);
  if (!isnan(evt.temperature)) {
    latestTemp = evt.temperature;
    if (latestTemp > 40 && running && !allowEmergencyOverride) {
      stopMotion();
      Serial.println("[DHT] Auto-stop due to high temp");
      acknowledge("TempExceeded", 1);
    }
  }

  dht.humidity().getEvent(&evt);
  if (!isnan(evt.relative_humidity)) {
    latestHumid = evt.relative_humidity;
  }
}

// --- ESP-NOW ---
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    Serial.print("ESP-NOW send to MASTER: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAILED");
}


void onDataRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  // Log source MAC for debugging
  char macBuf[18];
  if (info && info->src_addr) {
    snprintf(macBuf, sizeof(macBuf),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             info->src_addr[0], info->src_addr[1], info->src_addr[2],
             info->src_addr[3], info->src_addr[4], info->src_addr[5]);
    Serial.printf("[ESP-NOW] Received from %s len=%d\n", macBuf, len);
  }

  // tolerate occasional extra bytes but require at least size of Command
  if (!rxQueue || len < (int)sizeof(Command)) {
    Serial.printf("[ONRECV] Bad len %d or rxQueue null\n", len);
    return;
  }
  Command cmd;
  memcpy(&cmd, data, min(len, (int)sizeof(Command)));
  cmd.label[sizeof(cmd.label)-1] = '\0';

  if (xQueueSend(rxQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.printf("[WARN] RX queue full - dropped cmd %s=%d\n", cmd.label, cmd.value);
  }
}

bool safeEspNowSend(const uint8_t* addr, const uint8_t* data, size_t len, int retries=3) {
  for (int i = 0; i < retries; i++) {
    esp_err_t err = esp_now_send(addr, data, len);
    if (err == ESP_OK) {
      return true;
    }
    // exponential backoff
    vTaskDelay(pdMS_TO_TICKS(5 + i * 10));
  }
  Serial.println("[ESP-NOW] Send failed after retries");
  return false;
}

void commsWatchdogTask(void* parameter) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      if (!wifiLost) {
        Serial.println("[WATCHDOG] WiFi lost!");
        wifiLost = true;
        WiFi.begin(ssid, password);
      }
    } else {
      wifiLost = false;
    }

    if (!esp_now_is_peer_exist(masterAddress)) {
      if (!espNowLost) {
        Serial.println("[WATCHDOG] ESP-NOW peer missing!");
        espNowLost = true;

        // Try re-adding peer
        esp_now_peer_info_t pi{};
        memcpy(pi.peer_addr, masterAddress, 6);
        pi.encrypt = false;
        pi.channel = currentChannel;
        if (esp_now_add_peer(&pi) == ESP_OK) {
          Serial.println("[WATCHDOG] Peer re-added");
        } else {
          Serial.println("[WATCHDOG] Failed to re-add peer");
        }
      }
    } else {
      espNowLost = false;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// --- Tasks ---
void commsTxTask(void*) {
  Command o;
  while (true) {
    if (xQueueReceive(txQueue, &o, portMAX_DELAY) == pdTRUE) {
      safeEspNowSend(masterAddress, (uint8_t*)&o, sizeof(o));
      // small gap to avoid flooding
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

void commsRxTask(void*) {
  Command cmd;
  while (true) {
    if (xQueueReceive(rxQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      Serial.printf("[RECV] %s=%d\n", cmd.label, cmd.value);
      Serial.printf("[RX] Got cmd %s=%d at %lu ms\n", cmd.label, cmd.value, millis());

      // Handshake special cases
      if (strcmp(cmd.label, "handshake") == 0) {
        if (cmd.value == 1) {
          handshakeAck = true;
          Serial.println("[HS] Ack received");
        } else {
          Serial.println("[HS] Handshake request received");
          acknowledge("handshake", 1);
          handshakeDone = true;
          Serial.println("[HS] Handshake established (slave side)");
        }
        continue;
      }

      if (strcmp(cmd.label, "ip") == 0) {
        uint32_t ipInt = (uint32_t)cmd.value;
        IPAddress senderIP(
          (ipInt >> 24) & 0xFF,
          (ipInt >> 16) & 0xFF,
          (ipInt >> 8) & 0xFF,
          ipInt & 0xFF
        );
        Serial.printf("[HS] Peer WebServer IP: %s\n", senderIP.toString().c_str());
        acknowledge("ipAck", 1);
        continue;
      }

      // Query requests
      if (strcmp(cmd.label, "status") == 0)    { acknowledge("status", running?1:0); continue; }
      if (strcmp(cmd.label, "temp?") == 0)     { if (latestTemp > -100) acknowledge("temp", (int)latestTemp); else acknowledge("temp", -999); continue; }
      if (strcmp(cmd.label, "humid?") == 0)    { acknowledge("humid", (int)latestHumid); continue; }
      if (strcmp(cmd.label, "speed?") == 0)    { acknowledge("speed", speedSteps); continue; }
      if (strcmp(cmd.label, "accel?") == 0)    { acknowledge("accel", accelSteps); continue; }
      if (strcmp(cmd.label, "decel?") == 0)    { acknowledge("decel", decelSteps); continue; }
      if (strcmp(cmd.label, "distance?") == 0) { acknowledge("distance", distance); continue; }
      if (strcmp(cmd.label, "delay?") == 0)    { acknowledge("delay", delayBetweenCycles); continue; }
      if (strcmp(cmd.label, "direction?") == 0){ acknowledge("direction", direction?1:0); continue; }

      // If handshake not done, ignore commands that change behavior
      if (!handshakeDone) {
        Serial.println("[RECV] Ignored command - handshake not done");
        continue;
      }

      if (strcmp(cmd.label, "override") == 0) {
        allowEmergencyOverride = (cmd.value != 0);
        Serial.printf("[CMD] override -> %d\n", allowEmergencyOverride?1:0);
        acknowledge("override", allowEmergencyOverride?1:0);
        continue;
      }

      // --- Operational commands: apply them reliably and exactly once ---
      if (strcmp(cmd.label, "run") == 0) {
        Serial.printf("[CMD] run=%d (running=%d)\n", cmd.value, running);
        if (cmd.value && !running) {
          autoRunEnabled = true;
          // Apply any pending param updates before start
          if (updateSpeedPending)   { updateSpeedPending = false; }
          if (updateAccelPending)   { updateAccelPending = false; }
          if (updateDecelPending)   { updateDecelPending = false; }
          if (updateDistancePending) updateDistancePending = false;
          startMotion();
        } else if (!cmd.value && running) {
          autoRunEnabled = false;
          allowEmergencyOverride = false; // reset override
          stopMotion();
        }
        acknowledge("run", cmd.value);
        continue;
      }

      if (strcmp(cmd.label, "stop") == 0) {
        stopMotion();
        acknowledge("stop", 1);
        continue;
      }

      // --- Parameter setters: apply immediately, careful about running state ---
      if (strcmp(cmd.label, "speed") == 0) {
        int newSpeed = cmd.value;
        Serial.printf("[CMD] Set speed request: %d (running=%d)\n", newSpeed, running);
        speedSteps = newSpeed;
        if (running) {
          stepper.setAccelerationInStepsPerSecondPerSecond(accelSteps);
          stepper.setDecelerationInStepsPerSecondPerSecond(decelSteps);
          stepper.setSpeedInStepsPerSecond(speedSteps);
        } else {
          stepper.setSpeedInStepsPerSecond(speedSteps);
        }
        updateSpeedPending = false;
        Serial.printf("[APPLY] speed -> %d\n", speedSteps);
        acknowledge("speed", speedSteps);
        continue;
      }

      if (strcmp(cmd.label, "accel") == 0) {
        int newAccel = cmd.value;
        Serial.printf("[CMD] Set accel request: %d (running=%d)\n", newAccel, running);
        accelSteps = newAccel;
        stepper.setAccelerationInStepsPerSecondPerSecond(accelSteps);
        updateAccelPending = false;
        acknowledge("accel", accelSteps);
        continue;
      }

      if (strcmp(cmd.label, "decel") == 0) {
        int newDecel = cmd.value;
        Serial.printf("[CMD] Set decel request: %d (running=%d)\n", newDecel, running);
        decelSteps = newDecel;
        stepper.setDecelerationInStepsPerSecondPerSecond(decelSteps);
        updateDecelPending = false;
        acknowledge("decel", decelSteps);
        continue;
      }

      if (strcmp(cmd.label, "distance") == 0) {
        distance = cmd.value;
        updateDistancePending = running;
        Serial.printf("[APPLY] distance -> %d\n", distance);
        acknowledge("distance", distance);
        continue;
      }

      if (strcmp(cmd.label, "delay") == 0) {
        delayBetweenCycles = cmd.value;
        Serial.printf("[APPLY] delay -> %d\n", delayBetweenCycles);
        acknowledge("delay", delayBetweenCycles);
        continue;
      }

      if (strcmp(cmd.label, "direction") == 0) {
        bool newDir = (cmd.value != 0);
        Serial.printf("[CMD] Set direction request: %d (running=%d)\n", newDir?1:0, running);
        direction = newDir;
        if (running) {
          stopMotion();
          vTaskDelay(pdMS_TO_TICKS(10));
          startMotion();
        }
        acknowledge("direction", direction?1:0);
        continue;
      }

      // Unknown command
      Serial.printf("[RECV] Unknown cmd: %s\n", cmd.label);
    }
  }
}

void telemetryTask(void*) {
  const uint32_t interval = 200;
  uint32_t nextTick = millis() + interval;

  while (true) {
    uint32_t now = millis();
    if ((int32_t)(now - nextTick) >= 0) {
      nextTick += interval;
      if (running) {
        if (lastLiveSent == 0) {
          startPos = stepper.getCurrentPositionInSteps();
          lastPos = startPos;
          lastTime = nowMillis();
          sendLive("distanceLive", 0);
          lastDistanceSent = 0;
          lastSpeedSent = -1;
        }

        long currentPos = stepper.getCurrentPositionInSteps();
        long traveled = labs(currentPos - startPos);

        unsigned long nowt = nowMillis();
        int currentSpeed = (int)((currentPos - lastPos) * 1000.0 / (nowt - lastTime + 1));
        lastPos = currentPos;
        lastTime = nowt;

        if (currentSpeed != lastSpeedSent) {
          sendLive("speedLive", currentSpeed);
          lastSpeedSent = currentSpeed;
        }
        if (traveled != lastDistanceSent) {
          sendLive("distanceLive", traveled);
          lastDistanceSent = traveled;
        }
      }
      lastLiveSent = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void dhtTask(void*) {
  while (true) {
    captureTempHumid();
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// -- WiFi ----------------

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.printf("[WiFi] Connecting to %s", ssid);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WiFi] STA failed, starting fallback AP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32_AP");
    Serial.printf("[WiFi] AP started. IP: %s\n", WiFi.softAPIP().toString().c_str());
  }

  if (MDNS.begin(host)) {
    Serial.printf("[mDNS] http://%s.local\n", host);
  } else {
    Serial.println("[mDNS] failed");
  }
}

// --- Web Server & API ---
WebServer server(80);
#include "index_html.cpp"  // include the .cpp directly

void ServerTask(void*) {
  while (true) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void sendCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// --- Web Server API Handlers ---

void handleApiLive() {
  String json = "{";
  json += "\"running\":"; json += (running ? "true" : "false"); json += ",";
  json += "\"temp\":"; json += String(latestTemp, 1); json += ",";
  json += "\"humid\":"; json += String(latestHumid, 1); json += ",";
  json += "\"speed\":"; json += String(lastSpeedSent >= 0 ? lastSpeedSent : 0); json += ",";
  json += "\"distance\":"; json += String(lastDistanceSent >= 0 ? lastDistanceSent : 0); json += ",";

  // Parameters
  json += "\"params\":{";
  json += "\"speed\":"; json += String(speedSteps); json += ",";
  json += "\"accel\":"; json += String(accelSteps); json += ",";
  json += "\"decel\":"; json += String(decelSteps); json += ",";
  json += "\"distance\":"; json += String(distance); json += ",";
  json += "\"delay\":"; json += String(delayBetweenCycles); json += ",";
  json += "\"direction\":"; json += (direction ? "1" : "0");
  json += "}";

  // Add IP + MAC + Channel
  json += ",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"mac\":\"" + WiFi.macAddress() + "\",";
  json += "\"channel\":" + String(currentChannel);

  json += "}";

  sendCORS();
  server.send(200, "application/json", json);
}

void handleApiInfo() {
  uint8_t primaryChan;
  wifi_second_chan_t secondChan;
  if (esp_wifi_get_channel(&primaryChan, &secondChan) != ESP_OK) {
    primaryChan = 0; // fallback
  }

  String json = "{";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"mac\":\"" + WiFi.macAddress() + "\",";
  json += "\"channel\":" + String(primaryChan);
  json += "}";

  sendCORS();
  server.send(200, "application/json", json);
}

void handleApiSet() {
  if (server.hasArg("speed")) {
    speedSteps = server.arg("speed").toInt();
    updateSpeedPending = running;
    if (!running)
      acknowledge("speed", speedSteps);
  }
  if (server.hasArg("accel")) {
    accelSteps = server.arg("accel").toInt();
    updateAccelPending = running;
    if (!running)
      acknowledge("accel", accelSteps);
  }
  if (server.hasArg("decel")) {
    decelSteps = server.arg("decel").toInt();
    updateDecelPending = running;
    if (!running)
      acknowledge("decel", decelSteps);
  }
  if (server.hasArg("distance")) {
    distance = server.arg("distance").toInt();
    updateDistancePending = running;
    acknowledge("distance", distance);
  }
  if (server.hasArg("delay")) {
    delayBetweenCycles = server.arg("delay").toInt();
    acknowledge("delay", delayBetweenCycles);
  }
  if (server.hasArg("direction")) {
    direction = server.arg("direction").toInt() != 0;
    acknowledge("direction", direction?1:0);
    if (running) {
      stopMotion();
      vTaskDelay(pdMS_TO_TICKS(10));
      startMotion();
    }
  }
  sendCORS();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleRun() {
  int v = server.hasArg("val") ? server.arg("val").toInt() : 0;
  if (v && !running) {
    autoRunEnabled = true;
    if (updateSpeedPending)   {  updateSpeedPending = false; }
    if (updateAccelPending)   {  updateAccelPending = false; }
    if (updateDecelPending)   {  updateDecelPending = false; }
    if (updateDistancePending) updateDistancePending = false;
    startMotion();
  } else if (!v && running) {
    autoRunEnabled = false;
    stopMotion();
  }
  acknowledge("run", v);
  sendCORS();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStop() {
  stopMotion();
  acknowledge("stop", 1);
  sendCORS();
  server.send(200, "application/json", "{\"ok\":true}");
}

// send a handshake request (Command)
void sendHandshakePacket() {
  Command cmd{};
  strncpy(cmd.label, "handshake", sizeof(cmd.label)-1);
  cmd.value = 0;
  esp_now_send(masterAddress, (uint8_t*)&cmd, sizeof(cmd));
  Serial.println("[HS] Sent handshake request");
}

// wait for handshake ack from commsRxTask
bool waitForAck(uint32_t timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (handshakeAck) {
      handshakeAck = false; // consume it
      return true;
    }
    delay(10);
  }
  return false;
}

bool doHandshake() {
  for (int i = 0; i < 5; i++) {
    sendHandshakePacket();
    if (waitForAck(200)) {   // 200ms timeout
      return true;           // ACK received
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // wait a bit before retry
  }
  return false;  // no ACK after 5 tries
}

void handleHandshake() {
  bool success = doHandshake();

  if (success) {
    Serial.println("Handshake successful!");
    handshakeDone = true;   // ✅ only set when actual ack received
    acknowledge("handshake", 1);
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    Serial.println("Handshake failed after retries.");
    handshakeDone = false;  // ❌ don’t pretend it worked
    server.send(200, "application/json", "{\"ok\":false}");
  }

  sendCORS();
}

void handleRoot() {
  sendCORS();
  server.send_P(200, "text/html", index_html);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  Serial.println("=== SLAVE REBOOTED ===");

  // --- Stepper enable pin ---
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  // --- DHT sensor ---
  dht.begin();
  delay(2000);

  // --- Queues first ---
  rxQueue = xQueueCreate(50, sizeof(Command));
  txQueue = xQueueCreate(50, sizeof(Command));
  if (rxQueue == NULL || txQueue == NULL) {
    Serial.println("[ERROR] Queue creation failed!");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));  // halt safely instead of crashing
    }
  }

  Serial.printf("[DEBUG] Free heap after queue creation: %u bytes\n", ESP.getFreeHeap());

  // --- Stepper ---
  stepper.connectToPins(STEP_PIN, DIR_PIN);
  stepper.setSpeedInStepsPerSecond(speedSteps);
  stepper.setAccelerationInStepsPerSecondPerSecond(accelSteps);
  stepper.setDecelerationInStepsPerSecondPerSecond(decelSteps);
  stepper.startAsService(1);

  Serial.printf("[DEBUG] Free heap after stepper init: %u bytes\n", ESP.getFreeHeap());

  // --- Wi-Fi first ---
  setupWiFi(); // Connect STA / fallback AP

  // wait until connected (with small heartbeat)
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("[DEBUG] Free heap after WiFi connect: %u bytes\n", ESP.getFreeHeap());

  // --- ESP-NOW init AFTER Wi-Fi ---
  esp_err_t enerr = esp_now_init();
  if (enerr != ESP_OK) {
    Serial.printf("[ESP-NOW] Init failed (err=%d)\n", enerr);
    // If init fails, optionally halt to debug:
    // while (true) vTaskDelay(pdMS_TO_TICKS(1000));
  } else {
    Serial.println("[ESP-NOW] Initialized");
  }

  // --- Add peer using the actual Wi-Fi channel (safer method) ---
  esp_now_peer_info_t pi{};
  memcpy(pi.peer_addr, masterAddress, 6);
  pi.encrypt = false;

  // Get current primary Wi-Fi channel more reliably
  uint8_t primaryChan = 0;
  wifi_second_chan_t secondChan = WIFI_SECOND_CHAN_NONE;
  if (esp_wifi_get_channel(&primaryChan, &secondChan) == ESP_OK) {
    pi.channel = primaryChan;
    Serial.printf("[DEBUG] Using WiFi channel (esp_wifi_get_channel): %u\n", primaryChan);
    currentChannel = primaryChan;
  } else {
    // fallback - use WiFi.channel() but warn
    pi.channel = WiFi.channel();
    Serial.printf("[DEBUG] esp_wifi_get_channel() failed, fallback WiFi.channel(): %u\n", pi.channel);
    currentChannel = pi.channel;
  }

  WiFi.macAddress(mac);
  String macStr = WiFi.macAddress(); // pretty string like "24:6F:28:xx:yy:zz"

  if (!esp_now_is_peer_exist(masterAddress)) {
    if (esp_now_add_peer(&pi) == ESP_OK) {
      Serial.println("[ESP-NOW] Peer added");
    } else {
      Serial.println("[ESP-NOW] Failed to add peer");
      // don't panic here — later watchdog will try again
    }
  } else {
    Serial.println("[ESP-NOW] Peer already exists");
  }

  Serial.printf("[DEBUG] Free heap after ESP-NOW peer add: %u bytes\n", ESP.getFreeHeap());

  // --- ESP-NOW callbacks ---
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // --- Acknowledge reboot safely via a delayed task (single place) ---
  xTaskCreate([](void*) {
      vTaskDelay(pdMS_TO_TICKS(1500)); // allow ESP-NOW to settle
      if (esp_now_is_peer_exist(masterAddress)) {
        acknowledge("reboot", 1);
        Serial.println("[AckTask] Reboot acknowledged via ESP-NOW");
      } else {
        Serial.println("[AckTask] Peer not present, skipping acknowledge");
      }
      vTaskDelete(nullptr);
  }, "AckTask", 4096, nullptr, 2, nullptr);

  // --- Tasks (pinned) ---
  xTaskCreatePinnedToCore(commsTxTask,   "TX",     4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(commsRxTask,   "RX",     4096, nullptr, 4, nullptr, 0); // raised priority to 4
  xTaskCreatePinnedToCore(telemetryTask, "TEL",    4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(dhtTask,       "DHT",    3072, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(ServerTask,    "Server", 4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(commsWatchdogTask, "commsWatchdogTask", 4096, NULL, 1, NULL, 1);  // run on core 1

  Serial.printf("[DEBUG] Free heap after task creation: %u bytes\n", ESP.getFreeHeap());

  // --- Web routes ---
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/live", HTTP_GET, handleApiLive);
  server.on("/api/set", HTTP_GET, handleApiSet);
  server.on("/run", HTTP_GET, handleRun);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/handshake", HTTP_GET, handleHandshake);
  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) {
      sendCORS();
      server.send(200);
    } else {
      sendCORS();
      server.send(404, "text/plain", "Not found");
    }
  });
  server.on("/api/info", HTTP_GET, handleApiInfo);
  server.begin();
  Serial.println("[WEB] Server started");
  Serial.printf("[DEBUG] Free heap at end of setup: %u bytes\n", ESP.getFreeHeap());
}

// External hook for stack overflow
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  if (pcTaskName) {
    Serial.print("[FATAL] Stack overflow in task: ");
    Serial.println(pcTaskName);
  } else {
    Serial.println("[FATAL] Stack overflow (unknown task)");
  }
  for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

void loop() {
  static unsigned long stopT0 = 0;

  if (running && stepper.motionComplete()) {
    stopMotion();

    if (updateSpeedPending)   { updateSpeedPending = false; }
    if (updateAccelPending)   { updateAccelPending = false; }
    if (updateDecelPending)   { updateDecelPending = false; }
    if (updateDistancePending){ updateDistancePending = false; }

    stopT0 = millis();
  }

  if (!running && stopT0 && (millis() - stopT0 >= (unsigned long)delayBetweenCycles)) {
    stopT0 = 0;
    if (autoRunEnabled) {
      startMotion();
    }
  }

  // Add a small delay so loop does not starve other operations
  vTaskDelay(pdMS_TO_TICKS(10));
}
