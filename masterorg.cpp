#include "masterorg.h"
LGFX tft;

uint8_t slaveAddress[6] = { 0x58, 0xBF, 0x25, 0x9F, 0x5F, 0xE8 };

bool handshakeDone = false;
bool isRunning = false;
bool directionCW = false;
unsigned long lastHandshakeTime = 0;
const unsigned long handshakeInterval = 2000;
unsigned long lastQueryTime = 0;
const unsigned long queryInterval = 3000;
unsigned long lastDirChangeTime = 0;
const unsigned long dirEchoIgnoreMs = 500;

QueueHandle_t rxQueue = nullptr;
bool inChannelScreen = false;

int lastSentSpeed = -9999;
int lastSentAccel = -9999;
int lastSentDecel = -9999;
int lastSentDistance = -9999;
int lastSentDelay = -9999;
int lastSentRun = -1;
int lastSentDirection = -1;
unsigned long lastLiveUpdateTime = 0;
const unsigned long liveTimeout = 10000;

int espChannel = 1;
char channelBuffer[3] = "";
int bufIndex = 0;

SemaphoreHandle_t sendMutex = nullptr;

String currentInput = "";
String currentLabel = "";
bool keypadVisible = false;
int touchX = 0, touchY = 0;

const char* keys[4][3] = {
  {"1","2","3"},
  {"4","5","6"},
  {"7","8","9"},
  {"C","0","K"}
};

float spinnerAngle = 0;
const int spinnerRadius = 30;
const int spinnerX = 240;
const int spinnerY = 70;
const int spokeCount = 12;
const int colors[] = { TFT_RED, TFT_GREEN, TFT_BLUE, TFT_YELLOW, TFT_CYAN, TFT_MAGENTA };
const int numColors = sizeof(colors) / sizeof(colors[0]);

volatile int paramValues[7] = {0};
volatile int liveValues[7]  = {0};
volatile int liveTempC = 0;
volatile int liveHumid = 0;
volatile int liveStatus = 0;
int lastDrawnTemp = -32768;
int lastDrawnHumid = -32768;
int lastDrawnStatus = -32768;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t spinnerTaskHandle = nullptr;
TaskHandle_t HandshakeTaskHandle = nullptr;
TaskHandle_t queries_throttledTaskHandle = nullptr;
TaskHandle_t ESPNow_setTaskHandle = nullptr;
SemaphoreHandle_t tftMutex = nullptr;
IPAddress lastReceivedIP;

Command outgoingMessage;


int safeReadLiveValue(int index) {
  int v = 0;
  portENTER_CRITICAL(&mux);
  if (index >= 0 && index < NUM_PARAMS) v = liveValues[index];
  portEXIT_CRITICAL(&mux);
  return v;
}

// Simple withLock macro if you want to reuse your TFT mutex
void withLock(auto fn) {
  if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
    fn();
    xSemaphoreGive(tftMutex);
  }
}


inline void safeCopyLabel(char* dst, const char* src) {
  strncpy(dst, src, sizeof(outgoingMessage.label));
  dst[sizeof(outgoingMessage.label) - 1] = '\0';
}

inline int findParamIndex(const char* label) {
  for (int i = 0; i < NUM_PARAMS; i++) {
    if (strcmp(label, paramLabels[i]) == 0) return i;
  }
  return -1;
}
#include "esp_err.h"  // for esp_err_to_name

portMUX_TYPE sendMux = portMUX_INITIALIZER_UNLOCKED;  // reuse or create a send mutex

esp_err_t sendPacketWithDebug(const uint8_t* peer, const char* label, int value) {
  if (!handshakeDone && strcmp(label, "handshake") != 0) return ESP_ERR_INVALID_STATE;
  safeCopyLabel(outgoingMessage.label, label);
  outgoingMessage.value = value;

  if (xSemaphoreTake(sendMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    esp_err_t ret = esp_now_send(peer, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
    xSemaphoreGive(sendMutex);
    Serial.printf("[MASTER] esp_now_send() -> %s (label=%s, val=%d)\n",
                  esp_err_to_name(ret), label, value);
    return ret;
  } else {
    // couldn't take mutex
    return ESP_ERR_TIMEOUT;
  }
}

void enterChannelScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 10);
  tft.setTextSize(2);
  tft.println("Enter Channel (1-13):");
  inChannelScreen = true;
  drawKeypad();
  bool done = false;
  bufIndex = 0;
  channelBuffer[0] = '\0';
  int lastKey = -1;  // for debouncing
  while (!done) {
    int key = readKeypad();  // 0–9, 10=C, 11=K
    if (key != -1 && key != lastKey) {  // new key pressed
      lastKey = key;                    // remember it
      if (key >= 0 && key <= 9 && bufIndex < 2) {
        channelBuffer[bufIndex++] = '0' + key;
        channelBuffer[bufIndex] = '\0';
        showChannelText(channelBuffer);
      } else if (key == 10) {  // CLR
        bufIndex = 0;
        channelBuffer[0] = '\0';
        showChannelText(channelBuffer);
      } else if (key == 11) {  // OK
        int ch = atoi(channelBuffer);
        if (ch >= 1 && ch <= 13) {
          espChannel = ch;
          esp_wifi_set_channel(espChannel, WIFI_SECOND_CHAN_NONE);
          done = true;
          inChannelScreen = false;
        } else {
          tft.println("Invalid channel!");
        }
      }
    } else if (key == -1) {
      lastKey = -1;  // reset when no touch detected
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // WDT friendly
  }

  hideKeypad();
  tft.fillScreen(TFT_BLACK);
  //drawMainScreen();  // restore original screen
}



void safeWriteLiveValue(int index, int v) {
  portENTER_CRITICAL(&mux);
  if (index >= 0 && index < NUM_PARAMS) liveValues[index] = v;
  portEXIT_CRITICAL(&mux);
}

void safeWriteParamValue(int index, int v) {
  portENTER_CRITICAL(&mux);
  if (index >= 0 && index < NUM_PARAMS) paramValues[index] = v;
  portEXIT_CRITICAL(&mux);
}

// ---------------- ESP-NOW Callbacks ----------------
void onDataRecv(const esp_now_recv_info_t* recv_info,
                const uint8_t* incomingData, int len) {
  if (len < (int)sizeof(Command)) return;
  Command cmd;
  memcpy(&cmd, incomingData, sizeof(Command));
  // copy minimal info into queue and return ASAP
  BaseType_t ok = xQueueSendFromISR(rxQueue, &cmd, NULL);
  (void)ok;  // optionally check, but don't block
}
void rxProcessorTask(void* pv) {
  Command cmd;
  uint8_t srcmac[6];
  while (true) {
    if (xQueueReceive(rxQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      // safe to do Serial prints, parsing and longer ops here
      Serial.printf("[MASTER] Received label='%s' val=%d\n", cmd.label, cmd.value);

      // --- DEBUG: print raw label and value ---
      Serial.print("[MASTER] Received raw label: '");
      Serial.print(cmd.label);
      Serial.print("', value = ");
      Serial.println(cmd.value);

      // Optional: print MAC of sender (not filled yet in your code)
      char macStr[18];
      sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
              srcmac[0], srcmac[1], srcmac[2],
              srcmac[3], srcmac[4], srcmac[5]);
      Serial.print("From MAC: ");
      Serial.println(macStr);

      // --- handshake ---
      if (strcmp(cmd.label, "handshake") == 0) {
        handshakeDone = true;
        Serial.println("[MASTER] Handshake received!");
        continue;  // don't return, keep task alive
      }

      // --- environment (DHT) & status ---
      if (strcmp(cmd.label, "temp") == 0) {
        portENTER_CRITICAL(&mux);
        liveTempC = cmd.value;
        portEXIT_CRITICAL(&mux);
        continue;

      }
      if (strcmp(cmd.label, "humid") == 0) {
        portENTER_CRITICAL(&mux);
        liveHumid = cmd.value;
        portEXIT_CRITICAL(&mux);
        continue;
      }
      if (strcmp(cmd.label, "status") == 0) {
        portENTER_CRITICAL(&mux);
        liveStatus = cmd.value;
        portEXIT_CRITICAL(&mux);
        continue;
      }
      if (strcmp(cmd.label, "ip") == 0) {
        IPAddress ip(
          (cmd.value >> 24) & 0xFF,
          (cmd.value >> 16) & 0xFF,
          (cmd.value >> 8) & 0xFF,
          cmd.value & 0xFF);
        Serial.printf("[MASTER] Received Slave IP: %s\n", ip.toString().c_str());
        lastReceivedIP = ip;
        continue;
      }

      // --- live channels ---
      if (strcmp(cmd.label, "speedLive") == 0) {
        safeWriteLiveValue(0, cmd.value);
        lastLiveUpdateTime = millis();
        continue;
      }
      if (strcmp(cmd.label, "distanceLive") == 0 || strcmp(cmd.label, "distanceL") == 0) {
        safeWriteLiveValue(3, cmd.value);
        lastLiveUpdateTime = millis();
        continue;
      }

      // --- parameter ACKs ---
      int idx = findParamIndex(cmd.label);
      if (idx >= 0) {
        if (strcmp(cmd.label, "direction") == 0) {
          if (millis() - lastDirChangeTime < dirEchoIgnoreMs) {
            continue;
          }
        }
        safeWriteParamValue(idx, cmd.value);

        if (strcmp(cmd.label, "run") == 0) {
          static unsigned long lastRunChangeTime = 0;
          if (millis() - lastRunChangeTime > 300) {
            isRunning = (cmd.value == 1);
              liveStatus = isRunning;   // <-- keep in sync
            lastRunChangeTime = millis();
          }
        }

        if (strcmp(cmd.label, "direction") == 0) {
          static unsigned long lastdirChangeTime = 0;
          if (millis() - lastdirChangeTime > 300) {
            directionCW = (cmd.value == 1);
            lastdirChangeTime = millis();
          }
        }

        Serial.printf("[MASTER] ACK %s = %d\n", cmd.label, cmd.value);
        continue;
      }

      // unknown label -> ignore
      Serial.printf("[MASTER] Unhandled label: %s = %d\n", cmd.label, cmd.value);
    }
  }
  vTaskDelete(NULL); // never reached, but good practice
}

//}
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("ESP-NOW Send OK");
    } else {
        Serial.println("ESP-NOW Send FAIL");
    }

    // OPTIONAL: compare destination MAC with your known slave
    if (tx_info && tx_info->des_addr) {
        if (memcmp(tx_info->des_addr, slaveAddress, 6) == 0) {
            Serial.println("Confirmed: sent to correct slave");
        }
    }
}

// Function to handle receiving the IP address from the slave





// ---------------- Send helpers ----------------
void sendPacket(const char* label, int value) {
  if (!handshakeDone && strcmp(label, "handshake") != 0) return;
  sendPacketWithDebug(slaveAddress, label, value);
  Serial.printf("[MASTER] Sent: %s = %d\n", label, value);
}

void liveSendTask(void* parameter) {
  while (true) {
    if (handshakeDone && isRunning && esp_now_is_peer_exist(slaveAddress)) {
      int speed = safeReadLiveValue(0);
      int distance = safeReadLiveValue(3);
      Serial.println("Before sendPacket speedLive");
      sendPacketWithDebug(slaveAddress, "speedLive", speed);
      Serial.println("After sendPacket speedLive");
      sendPacketWithDebug(slaveAddress, "distanceLive", distance);
    }
    vTaskDelay(pdMS_TO_TICKS(200));  // 5 Hz
  }
}


// Wrapper: send only if changed
void sendIfChanged(const char* label, int value) {
  int* lastPtr = nullptr;

  if (strcmp(label, "speed") == 0) lastPtr = &lastSentSpeed;
  else if (strcmp(label, "accel") == 0) lastPtr = &lastSentAccel;
  else if (strcmp(label, "decel") == 0) lastPtr = &lastSentDecel;
  else if (strcmp(label, "distance") == 0) lastPtr = &lastSentDistance;
  else if (strcmp(label, "delay") == 0) lastPtr = &lastSentDelay;
  else if (strcmp(label, "run") == 0) lastPtr = &lastSentRun;
  else if (strcmp(label, "direction") == 0) lastPtr = &lastSentDirection;

  if (lastPtr != nullptr) {
    if (*lastPtr == value) {
      // unchanged -> don’t send again
      return;
    }
    *lastPtr = value;
  }

  safeCopyLabel(outgoingMessage.label, label);
  outgoingMessage.value = value;
  //esp_now_send(slaveAddress, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
  sendPacketWithDebug(slaveAddress, outgoingMessage.label, outgoingMessage.value);
  Serial.printf("[MASTER] Sent (changed): %s = %d\n", label, value);
}
void sendHandshake() {
  safeCopyLabel(outgoingMessage.label, "handshake");
  outgoingMessage.value = 1;
  //esp_now_send(slaveAddress, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
  sendPacketWithDebug(slaveAddress, outgoingMessage.label, outgoingMessage.value);
  Serial.println("[MASTER] Sent handshake");
}



void showChannelText(const char* buf) {
  tft.fillRect(10, 80, 100, 20, TFT_BLACK);
  tft.setCursor(10, 80);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print(buf);
}

void refreshTopBar() {
  withLock([&] {
    tft.fillRect(0, 0, 340, 12, TFT_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(1);
    tft.printf("%s:%s", currentLabel.c_str(), currentInput.c_str());
  });
}

void sendValueAndEcho(const String& val, const String& label) {
  int ival = val.length() ? val.toInt() : 0;
  sendIfChanged(label.c_str(), ival);
  int idx = findParamIndex(label.c_str());
  if (idx >= 0) {
    safeWriteParamValue(idx, (label == "run" || label == "direction") ? (ival != 0) : ival);
    if (label == "run") isRunning = (ival != 0);
    if (label == "direction") directionCW = (ival != 0);
  }

  withLock([&] {
    tft.fillRect(0, STATUS_BAR_Y, 300, 20, TFT_BLACK);
    tft.setCursor(0, STATUS_BAR_Y);
    tft.setTextSize(2);
    tft.setTextColor(TFT_GREEN);
    tft.printf("%s", label.c_str());
  });
}


void drawKeypad() {
  withLock([&] {
    tft.fillRect(KEYPAD_X, KEYPAD_Y, 3 * KEYPAD_BTN_W, 4 * KEYPAD_BTN_H, TFT_BLACK);
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 3; col++) {
        int x = KEYPAD_X + col * KEYPAD_BTN_W;
        int y = KEYPAD_Y + row * KEYPAD_BTN_H;
        tft.fillRect(x, y, KEYPAD_BTN_W - 2, KEYPAD_BTN_H - 2, 0x2945);
        tft.setCursor(x + 15, y + 5);
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.print(keys[row][col]);
      }
    }
    keypadVisible = true;
    tft.fillRect(KEYPAD_X, 200, 60, 30, TFT_BLACK);
  });
}

void checkKeypadPress(int x, int y) {
  if (!handshakeDone || !keypadVisible) return;

  if (x < KEYPAD_X || x > KEYPAD_X + 3 * KEYPAD_BTN_W) return;
  if (y < KEYPAD_Y || y > KEYPAD_Y + 4 * KEYPAD_BTN_H) return;

  int col = (x - KEYPAD_X) / KEYPAD_BTN_W;
  int row = (y - KEYPAD_Y) / KEYPAD_BTN_H;

  if (row >= 0 && row < 4 && col >= 0 && col < 3) {
    const char* key = keys[row][col];

    if (strcmp(key, "C") == 0) {
      currentInput = "";
    } else if (strcmp(key, "K") == 0) {
      if (currentLabel != "") {
        withLock([&] {
          tft.fillRect(130, 10, 210, 190, TFT_BLACK);
        });
        sendValueAndEcho(currentInput, currentLabel);
      }
      currentLabel = "";
      currentInput = "";
      hideKeypad();
    } else {
      currentInput += key;
    }

    refreshTopBar();
  }
}


void hideKeypad() {
  withLock([&] {
    tft.fillRect(KEYPAD_X, KEYPAD_Y, 3 * KEYPAD_BTN_W, 4 * KEYPAD_BTN_H, TFT_BLACK);
    keypadVisible = false;
  });
}

int readKeypad() {
  int tx, ty;
  if (tft.getTouch(&tx, &ty)) {
    int col = (tx - KEYPAD_X) / KEYPAD_BTN_W;
    int row = (ty - KEYPAD_Y) / KEYPAD_BTN_H;

    if (row >= 0 && row < 4 && col >= 0 && col < 3) {
      const char* key = keys[row][col];

      if (!inChannelScreen) {
        checkKeypadPress(tx, ty);  // only run for other screens
      }

      if (strcmp(key, "C") == 0) return 10;
      if (strcmp(key, "K") == 0) return 11;
      return atoi(key);  // real 0–9
    }
  }
  return -1;  // no press
}

enum MotorDir { STOPPED, CW, CCW };
volatile MotorDir motorDir = STOPPED;

void updateMotorDir() {
  if (!isRunning) motorDir = STOPPED;
  else motorDir = directionCW ? CW : CCW;
}



void drawSpeedSpinner(bool freeze) {
  withLock([&] {
    // Erase old spinner
    for (int i = 0; i < spokeCount; i++) {
      float angle = (spinnerAngle + i * (360.0f / spokeCount)) * DEG_TO_RAD;
      int x1 = spinnerX + cos(angle) * (spinnerRadius - 5);
      int y1 = spinnerY + sin(angle) * (spinnerRadius - 5);
      int x2 = spinnerX + cos(angle) * spinnerRadius;
      int y2 = spinnerY + sin(angle) * spinnerRadius;
      // single erase pass (use black to overwrite)
      tft.drawLine(x1, y1, x2 + 15, y2 + 15, TFT_BLACK);
    }

    // Update angle based on state/direction
    // If 'freeze' is true OR not running -> do not change angle
    if (!freeze && isRunning) {
      const int stepDeg = 10; // rotation speed (degrees per frame)
      if (directionCW) {
        // clockwise -> increment angle
        spinnerAngle += stepDeg;
        if (spinnerAngle >= 360) spinnerAngle -= 360;
      } else {
        // anticlockwise -> decrement angle
        spinnerAngle -= stepDeg;
        if (spinnerAngle < 0) spinnerAngle += 360;
      }
    }

    // Draw new spinner (colored spokes)
    static int colorIndex = 0;
    for (int s = 0; s < spokeCount; s++) {
      float angle = (spinnerAngle + s * (360.0f / spokeCount)) * DEG_TO_RAD;
      int x1 = spinnerX + cos(angle) * (spinnerRadius - 5);
      int y1 = spinnerY + sin(angle) * (spinnerRadius - 5);
      int x2 = spinnerX + cos(angle) * spinnerRadius;
      int y2 = spinnerY + sin(angle) * spinnerRadius;

      tft.drawLine(x1, y1, x2 + 10, y2 + 10, colors[colorIndex]);

      colorIndex++;
      if (colorIndex >= numColors) colorIndex = 0;
    }
  });
}

//-------------------------all  Task  ----------------------------
// ---------------- TFT / UI Task ----------------
void tftTask(void* pvParameters) {
  uint32_t lastDraw = 0;  // <- Move here at top of tftTask
    // --- Init and first draw ---

  auto withLock = [&](auto fn) {
    if (xSemaphoreTake(tftMutex, portMAX_DELAY) == pdTRUE) {
      fn();
      xSemaphoreGive(tftMutex);
    }
  };

  auto tftInit = [&]() {
    //tft.init();
    //tft.setRotation(1);

    withLock([&] {
      tft.setTextSize(2);

      //tft.fillScreen(TFT_BLACK);
      tft.setCursor(30, 0);
      tft.println("ESP_Flexi_Stepper_Master");
      tft.drawCircle(spinnerX, spinnerY, spinnerRadius, TFT_GOLD);


      //tft.fillCircle(spinnerX, spinnerY, spinnerRadius, LIGHTPINK);
    });
  };



auto drawIP = [&]() {
  if (handshakeDone && lastReceivedIP != IPAddress(0, 0, 0, 0)) {
    withLock([&]() {
      tft.setCursor(0, 225);
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
      tft.setTextSize(1);
      tft.fillRect(0, 225, 120, 12, TFT_BLACK); // Clear previous
      tft.printf("Slave IP: %s", lastReceivedIP.toString().c_str());
    });
  }
};



  auto drawEnvBar = [&]() {
    int temp, hum, stat;
    portENTER_CRITICAL(&mux);
    temp = liveTempC;
    hum = liveHumid;
    stat = liveStatus;
    portEXIT_CRITICAL(&mux);

    if (temp != lastDrawnTemp || hum != lastDrawnHumid || stat != lastDrawnStatus) {
      withLock([&] {
        tft.fillRect(100, STATUS_BAR_Y, 240, 20, TFT_BLACK);
        tft.setCursor(100, STATUS_BAR_Y);
        tft.setTextSize(2);
        tft.setTextColor(temp <= 40 ? TFT_GREENYELLOW : TFT_RED, TFT_BLACK);
        tft.printf("Temp:%d C  ", temp);
        tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
        tft.printf("Hum:%d%%  ", hum);
        tft.fillRect(135, STATUS_BAR_Y - 30, 240, 20, TFT_BLACK);
        tft.setCursor(135, STATUS_BAR_Y - 30);
        tft.setTextColor(stat ? TFT_GREEN : (temp <= 40 ? TFT_BLUE : TFT_RED), TFT_BLACK);
        tft.print(stat ? "Running" : (temp <= 40 ? "Stopped" : "Motor Halted"));
      });
      lastDrawnTemp = temp;
      lastDrawnHumid = hum;
      lastDrawnStatus = stat;
    }
  };

  auto drawParamButtons = [&]() {
    static int lastButtonColors[NUM_PARAMS] = { -1 };
    int y = PARAM_BTN_Y_START;

    withLock([&] {
      for (int i = 0; i < NUM_PARAMS; i++) {
        const char* label = paramLabels[i];
        bool isRun = (strcmp(label, "run") == 0);
        uint16_t color;

        if (isRun) {
          color = isRunning ? TFT_RED : TFT_GREEN;
        } else if (strcmp(label, "direction") == 0) {
          color = directionCW ? TFT_BLUE : TFT_ORANGE;
        } else {
          color = TFT_DARKGREY;
        }

        if (lastButtonColors[i] != color) {
          tft.fillRect(PARAM_BTN_X, y, PARAM_BTN_W, PARAM_BTN_H, color);
          tft.drawRect(PARAM_BTN_X, y, PARAM_BTN_W, PARAM_BTN_H, TFT_WHITE);
          lastButtonColors[i] = color;

          tft.setCursor(PARAM_BTN_X + 5, y + 8);
          tft.setTextSize(2);
          tft.setTextColor(isRun ? TFT_BLACK : TFT_CYAN, color);
          tft.print(label);
        }

        if (!isRun) {
          // left of pair: last set (ACK-backed) value
          tft.fillRect(PARAM_BTN_X + PARAM_BTN_W + 5, y + 8, 70, 15, TFT_BLACK);
          tft.setCursor(PARAM_BTN_X + PARAM_BTN_W + 5, y + 8);
          if (strcmp(label, "direction") == 0) {
            tft.setTextColor(TFT_BLUE, TFT_BLACK);
            tft.print(paramValues[i] == 1 ? "CW" : "AntiCW");
          } else {
            if (!keypadVisible) {
              tft.setTextColor(TFT_GOLD, TFT_BLACK);
            } else {
              tft.setTextColor(TFT_GOLD);
            }
            tft.printf("%d", paramValues[i]);
          }

          // right of pair (for speed/distance): live feed
          if (strcmp(label, "speed") == 0) {
            tft.fillRect(PARAM_BTN_X + PARAM_BTN_W + 90, y + 8, 100, 15, TFT_BLACK);
            tft.setCursor(PARAM_BTN_X + PARAM_BTN_W + 90, y + 8);
            tft.setTextColor(TFT_BLUE, TFT_BLACK);
            tft.printf("%d", safeReadLiveValue(0) );  // live speed @ index 0
          } else if (strcmp(label, "distance") == 0) {
            tft.fillRect(PARAM_BTN_X + PARAM_BTN_W + 100, y + 20, 100, 15, TFT_BLACK);
            tft.setCursor(PARAM_BTN_X + PARAM_BTN_W + 100, y + 20);
            tft.setTextColor(TFT_BLUE, TFT_BLACK);
            tft.printf("%d", safeReadLiveValue(3));  // live distance @ index 3
          }
        }

        y += PARAM_BTN_H;
      }
    });
  };

  auto checkParamButtonPress = [&](int x, int y) {
    if (!handshakeDone) return;
    if (x >= PARAM_BTN_X && x < PARAM_BTN_X + PARAM_BTN_W) {
      int index = (y - PARAM_BTN_Y_START) / PARAM_BTN_H;
      if (index >= 0 && index < NUM_PARAMS) {
        const char* label = paramLabels[index];
        if (strcmp(label, "run") == 0) {
          if (liveTempC > 40) {
            Serial.println("[MASTER] Safety lock: Cannot run, temp too high!");
            // Optional: show warning on TFT
            withLock([&] {
              tft.fillRect(0, STATUS_BAR_Y, 80, 20, TFT_BLACK);
              tft.setCursor(0, STATUS_BAR_Y);
              tft.setTextColor(TFT_RED, TFT_BLACK);
              tft.print("Over HOT!");
            });
            return;  // block run toggle
          } else {
            tft.fillRect(0, STATUS_BAR_Y, 80, 20, TFT_BLACK);
          }
          // normal toggle otherwise
          isRunning = !isRunning;
          safeWriteParamValue(index, isRunning ? 1 : 0);
          //sendPacket("run", isRunning ? 1 : 0);
          sendIfChanged("run", isRunning ? 1 : 0);
        } else if (strcmp(label, "direction") == 0) {
          directionCW = !directionCW;
          safeWriteParamValue(index, directionCW ? 1 : 0);
          lastDirChangeTime = millis();
          //sendPacket("direction", directionCW ? 1 : 0);
          sendIfChanged("direction", directionCW ? 1 : 0);
        } else {
          currentLabel = label;
          currentInput = "";
          drawKeypad();
          refreshTopBar();
        }
        // reflect immediate UI change; ACK will correct if needed
        drawParamButtons();
      }
    }
  };


  drawParamButtons();

  // --- UI loop (touch + redraw) ---
  while (true) {
    static bool wasTouching = false;
    uint16_t x, y;
    if (tft.getTouch(&x, &y)) {
      if (!wasTouching) {
        checkKeypadPress(x, y);
        checkParamButtonPress(x, y);
        wasTouching = true;
      }
    } else {
      wasTouching = false;

      // Periodic redraws to reflect live updates and DHT/status
      static uint32_t lastDraw = 0;
      uint32_t now = millis();
      if (now - lastDraw > 400) {
        drawParamButtons();
        drawEnvBar();
        drawIP();  // ✅ non-blocking call
        lastDraw = now;
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

// ---------------- Spinner Task ----------------
void SpinnerTask(void* parameter) {
  while (true) {
    if (!keypadVisible && handshakeDone) {  // <-- block spinner if keypad showing
      unsigned long now = millis();
      bool liveActive = (now - lastLiveUpdateTime < liveTimeout);

      if (isRunning && liveActive) {
        drawSpeedSpinner(false);  // spins normally
      } else {
        drawSpeedSpinner(true);  // freeze (no angle update)
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));  // ~20 FPS
  }
}




// ---------------- Handshake Task ----------------
void HandshakeTask(void* parameter) {
  while (true) {
    sendHandshake();
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ---------------- Query Task ----------------
void queries_throttledTask(void* parameter) {
  while (true) {
    if (handshakeDone) {
      // Basic health / env (DHT)
      sendPacket("status", 0);
      sendPacket("temp?", 0);
      sendPacket("humid?", 0);

      // Query each param (except run/direction)
      for (int i = 0; i < NUM_PARAMS; i++) {
        if (strcmp(paramLabels[i], "run") != 0 && strcmp(paramLabels[i], "direction") != 0) {
          String query = String(paramLabels[i]) + "?";
          sendPacket(query.c_str(), 0);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// ---------------- ESP-NOW maintenance ----------------
void ESPNow_setTask(void* parameter) {
  for (;;) {
    // Ensure STA (no re-init)
    if (WiFi.getMode() != WIFI_STA) {
      WiFi.mode(WIFI_STA);
    }

    // Ensure peer exists
    if (!esp_now_is_peer_exist(slaveAddress)) {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, slaveAddress, 6);
      peerInfo.channel = espChannel;
      peerInfo.encrypt = false;
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ERROR] Failed to add peer, will retry...");
      } else {
        Serial.println("[MASTER] Peer (re)added");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
