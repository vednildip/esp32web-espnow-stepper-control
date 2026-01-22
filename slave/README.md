# ESP32 ESP-NOW Slave (FlexyStepper + Telemetry)

This folder contains the **SLAVE firmware** for the ESP32-based
**ESP-NOW Master–Slave stepper control system**.

The slave node is responsible for:
- Stepper motor motion control (ESP_FlexyStepper)
- Reliable command execution via ESP-NOW
- Live telemetry feedback (speed & distance)
- Temperature & humidity safety monitoring (DHT11)
- Optional Web UI for local monitoring & control
- Robust communication watchdog & recovery

The firmware is fully **FreeRTOS-based** and designed for **long-running,
fault-tolerant operation**.

---

## 🧠 Design Philosophy

- **Command-driven architecture**
  - All motion changes are driven by ESP-NOW commands from the master
  - Each command is acknowledged explicitly

- **Safe concurrency**
  - ESP-NOW RX callback only enqueues data
  - RX/TX handled in dedicated FreeRTOS tasks

- **Fail-safe motion**
  - Emergency temperature stop
  - Controlled restart after communication recovery
  - Stepper disabled when idle

- **Dual interface**
  - ESP-NOW for master control
  - HTTP WebServer for diagnostics & fallback control

---

## 📂 Folder Contents

