master/README.md
# esp32web-espnow-stepper-control
FreeRTOS, TFT mutex, ESP-NOW sequencing, handshake, live updates of  stepper control 
# ESP32 ESP-NOW Master (Stepper Control)

This folder contains the **MASTER firmware** for an ESP32-based
**ESP-NOW Master–Slave stepper motor control system**.

The master node provides:
- TFT touch UI
- Channel selection before ESP-NOW init
- Peer management
- Reliable command dispatch
- Periodic handshake & polling
- Live motor update streaming

All real-time work is handled using **FreeRTOS tasks pinned to Core 1**.

---

## 🧠 Design Philosophy

- **No logic in `loop()`**  
  All processing runs in FreeRTOS tasks.

- **ESP-NOW safety first**  
  Wi-Fi channel is selected **before** ESP-NOW initialization.

- **Thread-safe UI**  
  TFT & touch access is protected using a mutex.

- **Non-blocking RX path**  
  ESP-NOW receive callback only enqueues data.

---

## 📂 Folder Contents
master/
├── src/
│ ├── master.ino → Entry point (setup + loop)
│ ├── masterorg.h → Global definitions & objects
│ ├── masterorg.cpp/h → FreeRTOS task implementations
│     ├── espnow_master.* → ESP-NOW init & callbacks
│     ├── tft_ui.* → TFT & touch UI logic
│     └── config.h → Pins, MACs, timing constants
│
└── README.md




