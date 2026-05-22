# Real-Time Vehicle Telemetry & Data Logging System

**Arys Garage Pvt. Ltd. — Technical Assignment Q2**
**Candidate:** Harsh Kumar Srivastava | harshsrivastava361@gmail.com

---

## Project Overview

A real-time embedded telemetry system for vehicle performance monitoring, built on **FreeRTOS** (STM32F4) with a **Python simulation dashboard** demonstrating all system behaviours including live fault injection and recovery.

---

## System Architecture+-----------------------------------------------------+
|                  FreeRTOS Scheduler                  |
+----------+----------+---------+----------+----------+
|  Fault   |  Sensor  |   GPS   |   CAN    |  Data    |
|  Monitor |  Acq.    |  Parser | Handler  |  Logger  |
|  Prio:6  |  Prio:5  |  Prio:4 |  Prio:3  |  Prio:2  |
+----------+----------+---------+----------+----------+
|      Queues: xIMUQueue, xGPSQueue, xLogQueue         |
+-----------------------------------------------------+
|  UART(GPS) | I2C(IMU) | SPI(SD) | CAN | ADC(Wheel) |
+-----------------------------------------------------+
---

## Repository Structurearys-vehicle-telemetry/
├── firmware/
│   ├── main.c                    # FreeRTOS entry, task creation, queues
│   └── tasks/
│       ├── sensor_task.c         # 100Hz IMU + complementary filter
│       ├── gps_task.c            # NMEA 0183 parser, GPS fault detection
│       ├── fault_task.c          # Safety monitor (Priority 6)
│       └── logger_task.c         # SD card CSV logger + RAM fallback
├── simulation/
│   └── telemetry_simulator.py    # Live Python dashboard
├── data/
│   └── telemetry_*.csv           # Auto-generated telemetry logs
└── docs/
└── report.docx               # Full assignment report
---

## Running the Simulation

```bash
pip3 install matplotlib numpy
git clone https://github.com/harshsrivastava361-wq/arys-vehicle-telemetry.git
cd arys-vehicle-telemetry
python3 simulation/telemetry_simulator.py
```

---

## RTOS Task Priority Design

| Priority | Task | Rate | Reason |
|----------|------|------|--------|
| 6 | Fault Monitor | 50ms | Safety-critical, preempts all |
| 5 | Sensor Acquisition | 10ms (100Hz) | Hard real-time sampling |
| 4 | GPS Parsing | Per sentence | UART buffer timing |
| 3 | CAN Handler | Per frame | Bus timing constraints |
| 2 | Data Logger | Per queue item | I/O bound, tolerates jitter |
| 1 | Wireless Telemetry | Best-effort | Non-critical |

---

## Sensor Fusion
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
- **AI Tools:** Claude (Anthropic)

---

## Simulation Results

- 120 second lap simulation with 2 complete laps
- 12,000+ telemetry data points logged to CSV
- All 4 fault scenarios triggered and recovered
- Live GPS track, G-force, orientation displayed
