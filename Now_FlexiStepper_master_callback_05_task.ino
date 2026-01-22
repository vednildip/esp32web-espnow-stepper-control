

#include "masterorg.h"
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

// --------------------------------------------------
// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  pinMode(TFTPWPin, OUTPUT);
  digitalWrite(TFTPWPin, HIGH);
  tft.init();
  tft.setRotation(1);
  uint16_t calData[] = { 366, 201, 322, 3694, 3783, 225, 3780, 3620 };
  tft.setTouchCalibrate(calData);
  sendMutex = xSemaphoreCreateMutex();
  if (!sendMutex) {
    Serial.println("sendMutex fail");
    while (1) delay(1000);
  }
  // Mutex for TFT
  tftMutex = xSemaphoreCreateMutex();
  if (!tftMutex) {
    Serial.println("[ERROR] TFT mutex creation failed!");
    while (1) { delay(1000); }
  }

  enterChannelScreen();  // <-- waits until user presses OK
  //hideKeypad();
  Serial.print("Using channel: ");
  Serial.println(espChannel);
  // --- Start Wi-Fi before setting channel ---
  WiFi.mode(WIFI_STA);  // station mode
  WiFi.disconnect();    // ensure disconnected
  esp_wifi_start();     // start Wi-Fi
  // now it is safe to set channel
  esp_wifi_set_channel(espChannel, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed!");
    while (1) { delay(1000); }
  }
  rxQueue = xQueueCreate(8, sizeof(Command));  // allow small backlog
  //rxQueue = xQueueCreate(12, sizeof(RxItem));
// RX processor task (handles queue messages from onDataRecv)
xTaskCreatePinnedToCore(
  rxProcessorTask, "rxProcessorTask",
  4096, NULL, 3, NULL, 1);


  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // Add peer if missing
  if (!esp_now_is_peer_exist(slaveAddress)) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, slaveAddress, 6);
    peerInfo.channel = espChannel;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("[ERROR] Failed to add peer!");
      while (1) { delay(1000); }
    }
  }
  Serial.println("[MASTER] ESP-NOW active");

  // Start UI task (handles all TFT + touch)
  xTaskCreatePinnedToCore(
    tftTask, "tftTask",
    32768, NULL, 2, NULL, 1);

  // Spinner redraw task
  xTaskCreatePinnedToCore(
    SpinnerTask, "SpinnerTask",
    8192, NULL, 2, &spinnerTaskHandle, 1);

  // Handshake task (2s interval)
  xTaskCreatePinnedToCore(
    HandshakeTask, "HandshakeTask",
    4096, NULL, 4, &HandshakeTaskHandle, 1);

  // Query polling task (3s interval)
  xTaskCreatePinnedToCore(
    queries_throttledTask, "queries_throttledTask",
    4096, NULL, 3, &queries_throttledTaskHandle, 1);

  // ESP-NOW maintenance (no re-init)
  xTaskCreatePinnedToCore(
    ESPNow_setTask, "ESPNow_setTask",
    4096, NULL, 1, &ESPNow_setTaskHandle, 1);

  // Live motor updates task (20 Hz)
  xTaskCreatePinnedToCore(
    liveSendTask, "liveSendTask",
    4096, NULL, 3, NULL, 1);

  delay(300);
}

// ---------------- Main Loop ----------------
void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));  // keep WDT happy; all work is in tasks
}


