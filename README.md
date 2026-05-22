# 🚗 Real-Time Vehicle Telemetry & Data Logging System
**Arys Garage Pvt. Ltd. — Technical Assignment Q2**
**Candidate:** Harsh Kumar Srivastava | harshsrivastava361@gmail.com

---

## 📌 Project Overview

A real-time embedded telemetry system for vehicle performance monitoring, built on **FreeRTOS** (STM32F4) with a **Python simulation dashboard** demonstrating all system behaviours including live fault injection and recovery.

---

## 🏗️ System Architecture
---

## 📁 Repository Structure
---

## 🚀 Running the Simulation

```bash
# Install dependencies
pip3 install matplotlib numpy

# Clone and run
git clone https://github.com/harshsrivastava361-wq/arys-vehicle-telemetry.git
cd arys-vehicle-telemetry
python3 simulation/telemetry_simulator.py
```

A live dashboard opens showing **Speed, RPM, G-Force, Roll/Pitch, GPS Track, and Fault Timeline** in real-time.

---

## ⚡ RTOS Task Priority Design

| Priority | Task | Rate | Reason |
|----------|------|------|--------|
| 6 | Fault Monitor | 50ms | Safety-critical, preempts all |
| 5 | Sensor Acquisition | 10ms (100Hz) | Hard real-time sampling |
| 4 | GPS Parsing | Per sentence | UART buffer timing |
| 3 | CAN Handler | Per frame | Bus timing constraints |
| 2 | Data Logger | Per queue item | I/O bound, tolerates jitter |
| 1 | Wireless Telemetry | Best-effort | Non-critical |

---

## 🔧 Sensor Fusion (Complementary Filter)
Fuses gyroscope (accurate short-term) with accelerometer (accurate long-term) to eliminate drift without matrix operations.

---

## ⚠️ Fault Detection & Recovery

| Fault | Timeout | Recovery |
|-------|---------|----------|
| GPS Loss | 2000ms | Dead reckoning via IMU + wheel speed |
| IMU Disconnect | 500ms | Disable fusion, GPS-only mode |
| CAN Timeout | 1000ms | Bus-off recovery, log error frame |
| SD Card Failure | On write error | RAM ring buffer + wireless priority boost |

---

## 🛠️ Tech Stack

- **Firmware:** C, FreeRTOS, STM32 HAL, FatFS
- **Protocols:** UART, I2C, SPI, CAN, ADC
- **Simulation:** Python 3.11, Matplotlib, NumPy
- **AI Tools:** Claude (Anthropic) — code generation, architecture, documentation

---

## 📊 Simulation Results

- ✅ 120 second lap simulation with 2 complete laps
- ✅ 12,000+ telemetry data points logged to CSV
- ✅ All 4 fault scenarios triggered and recovered
- ✅ Live GPS track, G-force, orientation displayed
