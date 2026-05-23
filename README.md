# Real-Time Vehicle Telemetry & Data Logging System

**Arys Garage Pvt. Ltd. — Technical Assignment Q2**  
**Candidate:** Harsh Kumar Srivastava | harshsrivastava361@gmail.com

---

## Project Overview

A real-time embedded telemetry system built on **FreeRTOS** (STM32F4) with a **Python simulation dashboard** demonstrating live fault injection and recovery.

---

## System Architecture

![Architecture](https://img.shields.io/badge/RTOS-FreeRTOS-blue) ![Language](https://img.shields.io/badge/Language-C%20%2B%20Python-green)

| Layer | Components |
|-------|-----------|
| Priority 6 | Fault Monitor — safety critical, preempts all |
| Priority 5 | Sensor Acquisition — 100Hz hard real-time |
| Priority 4 | GPS Parsing — NMEA UART timing |
| Priority 3 | CAN Handler — bus timing constraints |
| Priority 2 | Data Logger — I/O bound, SD card via SPI |
| Priority 1 | Wireless Telemetry — best-effort |

**Communication:** xIMUQueue, xGPSQueue, xCANQueue, xLogQueue  
**Interfaces:** UART (GPS) · I2C (IMU) · SPI (SD Card) · CAN Bus · ADC (Wheel Speed)

---

## Repository Structure

| Path | Description |
|------|-------------|
| firmware/main.c | FreeRTOS entry, task creation, queues |
| firmware/tasks/sensor_task.c | 100Hz IMU + complementary filter |
| firmware/tasks/gps_task.c | NMEA 0183 parser, GPS fault detection |
| firmware/tasks/fault_task.c | Safety monitor (Priority 6) |
| firmware/tasks/logger_task.c | SD card CSV logger + RAM fallback |
| simulation/telemetry_simulator.py | Live Python dashboard |
| data/telemetry_*.csv | Auto-generated telemetry logs |
| docs/report.docx | Full assignment report |

---

## Running the Simulation

Install dependencies and run:

    pip3 install matplotlib numpy
    python3 simulation/telemetry_simulator.py

---

## Sensor Fusion (Complementary Filter)

    roll  = 0.98 x (roll  + wx x dt) + 0.02 x arctan2(ay, az)
    pitch = 0.98 x (pitch + wy x dt) + 0.02 x arctan2(-ax, az)

---

## Fault Detection and Recovery

| Fault | Timeout | Recovery |
|-------|---------|----------|
| GPS Loss | 2000ms | Dead reckoning via IMU + wheel speed |
| IMU Disconnect | 500ms | Disable fusion, GPS-only mode |
| CAN Timeout | 1000ms | Bus-off recovery, log error frame |
| SD Card Failure | On write error | RAM ring buffer + wireless priority boost |

---

## Tech Stack

- **Firmware:** C, FreeRTOS, STM32 HAL, FatFS
- **Protocols:** UART, I2C, SPI, CAN, ADC
- **Simulation:** Python 3.11, Matplotlib, NumPy
- **AI Tools:** Claude (Anthropic) — code generation, architecture, documentation

---

## Simulation Results

- 120 second lap simulation with 2 complete laps
- 12,000+ telemetry data points logged to CSV
- All 4 fault scenarios triggered and recovered
- Live GPS track, G-force, orientation displayed
