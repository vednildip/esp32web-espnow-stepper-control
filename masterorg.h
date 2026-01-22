// Replace previous rxQueue declaration to use RxItem
// in master.h: extern QueueHandle_t rxQueue;  (keep it)
//Master  for  "Now_webseerver_Slave_flexistepper_task_smooth01 slave" 

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
 #include <LovyanGFX.hpp>
#include <math.h>
#include "esp_wifi.h"

// ---------------- Hardware / Display ----------------
#define DEG_TO_RAD (3.14159265f / 180.0f)
#define TFTPWPin 10
#define LIGHTPINK 0xFDB8  // approx. RGB(255,182,193)

// Global display object (only declared here)
extern LGFX tft;

// ---------------- ESP-NOW / Protocol ----------------
struct Command {
  char label[10];  // e.g. "speed", "accel", "run"
  int value;       // integer payload
};

// Peer MAC
extern uint8_t slaveAddress[6];

// ---------------- App State ----------------
extern bool handshakeDone;
extern bool isRunning;
extern bool directionCW;
extern unsigned long lastHandshakeTime;
extern const unsigned long handshakeInterval;
extern unsigned long lastQueryTime;
extern const unsigned long queryInterval;
extern unsigned long lastDirChangeTime;
extern const unsigned long dirEchoIgnoreMs;

extern QueueHandle_t rxQueue;
extern bool inChannelScreen;

// Change-only send state
extern int lastSentSpeed;
extern int lastSentAccel;
extern int lastSentDecel;
extern int lastSentDistance;
extern int lastSentDelay;
extern int lastSentRun;
extern int lastSentDirection;
extern unsigned long lastLiveUpdateTime;
extern const unsigned long liveTimeout;

// ---------------- UI Layout ----------------
#define PARAM_BTN_W 120
#define PARAM_BTN_H 30
#define PARAM_BTN_X 10
#define PARAM_BTN_Y_START 10

extern int espChannel;
extern char channelBuffer[3];
extern int bufIndex;

#define KEYPAD_X 200
#define KEYPAD_Y 50
#define KEYPAD_BTN_W 45
#define KEYPAD_BTN_H 30
#define STATUS_BAR_Y 225

extern SemaphoreHandle_t sendMutex;
// ---------------- UI elements ----------------
extern String currentInput;
extern String currentLabel;
extern bool keypadVisible;
extern int touchX, touchY;
extern const char* keys[4][3];
extern float spinnerAngle;
extern float lastSpinnerAngle;
extern float lastDrawnAngle;
extern const int spinnerRadius;
extern const int spinnerX;
extern const int spinnerY;
extern const int spokeCount;
extern const int colors[];
//extern const char* paramLabels[];
//extern const int NUM_PARAMS;
extern volatile int paramValues[];
extern volatile int liveValues[];
extern volatile int liveTempC;
extern volatile int liveHumid;
extern volatile int liveStatus;
extern int lastDrawnTemp;
extern int lastDrawnHumid;
extern int lastDrawnStatus;
// ---------------- RTOS / Sync ----------------
extern portMUX_TYPE mux;
extern TaskHandle_t spinnerTaskHandle;
extern TaskHandle_t HandshakeTaskHandle;
extern TaskHandle_t queries_throttledTaskHandle;
extern TaskHandle_t ESPNow_setTaskHandle;
extern SemaphoreHandle_t tftMutex;
extern IPAddress lastReceivedIP;

// ---------------- Utilities ----------------
extern Command outgoingMessage;
//#define NUM_PARAMS (sizeof(paramLabels) / sizeof(paramLabels[0]))
static const char* paramLabels[] = { "speed","accel","decel","distance","delay","direction","run" };
constexpr int NUM_PARAMS = sizeof(paramLabels) / sizeof(paramLabels[0]);
// ---------------- Function Decls ----------------
int safeReadLiveValue(int index);
void drawKeypad();
void refreshTopBar();
void drawSpeedSpinner(bool freeze);
void tftTask(void* pvParameters);
void SpinnerTask(void* parameter);
void HandshakeTask(void* parameter);
void queries_throttledTask(void* parameter);
void ESPNow_setTask(void* parameter);
int readKeypad();
void hideKeypad();
void checkKeypadPress(int x, int y);
void sendValueAndEcho(const String& val, const String& label);
void showChannelText(const char* buf);
void sendHandshake();
void sendIfChanged(const char* label, int value);
void liveSendTask(void* parameter);
void sendPacket(const char* label, int value);
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
void onDataRecv(const esp_now_recv_info_t* recv_info, const uint8_t* incomingData, int len);
void rxProcessorTask(void* pv);
void safeWriteParamValue(int index, int v);
void safeWriteLiveValue(int index, int v);
void enterChannelScreen();

void queries_throttledTask(void* parameter);
void HandshakeTask(void* parameter);
void SpinnerTask(void* parameter);
void tftTask(void* pvParameters);
