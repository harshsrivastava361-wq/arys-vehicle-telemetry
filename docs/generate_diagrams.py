"""
Generates all required diagrams for Arys Garage submission:
1. RTOS Task Architecture Diagram
2. System Block Diagram
3. Sensor Interface Schematic
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np
import os

os.makedirs("docs", exist_ok=True)

# ─────────────────────────────────────────────
# DIAGRAM 1: RTOS Task Architecture
# ─────────────────────────────────────────────
fig, ax = plt.subplots(1, 1, figsize=(16, 10))
ax.set_xlim(0, 16)
ax.set_ylim(0, 10)
ax.axis('off')
fig.patch.set_facecolor('#0d1117')
ax.set_facecolor('#0d1117')

def box(ax, x, y, w, h, color, text, subtext="", fontsize=11):
    rect = FancyBboxPatch((x, y), w, h,
                          boxstyle="round,pad=0.1",
                          facecolor=color, edgecolor='white',
                          linewidth=1.5, alpha=0.9)
    ax.add_patch(rect)
    ax.text(x + w/2, y + h/2 + (0.2 if subtext else 0),
            text, ha='center', va='center',
            color='white', fontsize=fontsize, fontweight='bold',
            fontfamily='monospace')
    if subtext:
        ax.text(x + w/2, y + h/2 - 0.25, subtext,
                ha='center', va='center',
                color='#aaaaaa', fontsize=8, fontfamily='monospace')

def arrow(ax, x1, y1, x2, y2, color='#58a6ff', label=""):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=color, lw=1.5))
    if label:
        mx, my = (x1+x2)/2, (y1+y2)/2
        ax.text(mx+0.1, my, label, color=color, fontsize=7,
                fontfamily='monospace', ha='left')

# Title
ax.text(8, 9.5, 'RTOS Task Architecture — Arys Garage Telemetry System',
        ha='center', va='center', color='white', fontsize=14,
        fontweight='bold', fontfamily='monospace')

# FreeRTOS Kernel bar
box(ax, 0.5, 8.5, 15, 0.7, '#1f4e79', 'FreeRTOS Kernel — STM32F407 @ 168MHz', fontsize=12)

# Tasks
tasks = [
    (0.5,  6.5, '#7b2d00', 'FAULT\nMONITOR',  'Priority 6\n50ms'),
    (3.5,  6.5, '#1a472a', 'SENSOR\nACQ.',     'Priority 5\n10ms/100Hz'),
    (6.5,  6.5, '#1f3a6e', 'GPS\nPARSER',      'Priority 4\nper sentence'),
    (9.5,  6.5, '#4a1942', 'CAN\nHANDLER',     'Priority 3\nper frame'),
    (12.5, 6.5, '#3d2b00', 'DATA\nLOGGER',     'Priority 2\nper item'),
]
colors_t = ['#f78166','#2ea043','#58a6ff','#d2a8ff','#ffa657']
for i, (x, y, c, t, s) in enumerate(tasks):
    box(ax, x, y, 2.5, 1.8, c, t, s, fontsize=10)
    # Arrow from kernel to task
    arrow(ax, x+1.25, 8.5, x+1.25, 8.3, color=colors_t[i])

# Wireless task (lower priority)
box(ax, 6.5, 4.3, 2.5, 1.8, '#2d3748', 'WIRELESS\nTELEM.', 'Priority 1\nbest-effort', fontsize=10)

# Queues
box(ax, 1.0, 4.3, 4.0, 1.5, '#162032', 'xIMUQueue (depth 20)\nxGPSQueue (depth 10)\nxCANQueue (depth 15)', fontsize=9)
box(ax, 10.0, 4.3, 4.5, 1.5, '#162032', 'xLogQueue (depth 50)\nxSDCardMutex\n(binary semaphore)', fontsize=9)

# Queue arrows
arrow(ax, 4.75, 6.5,  3.0, 5.8, color='#2ea043', label='IMU data')
arrow(ax, 7.75, 6.5,  3.0, 5.5, color='#58a6ff', label='GPS data')
arrow(ax, 10.75, 6.5, 12.0, 5.8, color='#d2a8ff', label='CAN frames')
arrow(ax, 3.0,  4.3, 13.5, 5.8, color='#ffa657', label='log entries')
arrow(ax, 13.75, 6.5, 13.5, 5.8, color='#ffa657')

# Hardware layer
box(ax, 0.5, 2.2, 15, 1.6, '#0f2027',
    'Hardware Abstraction Layer (STM32 HAL)',
    'UART(GPS·9600) │ I2C(IMU·400kHz) │ SPI(SD·25MHz) │ CAN(500kbps) │ ADC(12-bit)',
    fontsize=10)

# Arrows from tasks to HAL
for x in [1.75, 4.75, 7.75, 10.75, 13.75]:
    arrow(ax, x, 6.5, x, 3.8, color='#444444')

# Physical sensors
sensors = [
    (0.5,  0.3, '#1a1a2e', 'GPS MODULE\nUART/NMEA'),
    (3.5,  0.3, '#1a1a2e', 'MPU6050 IMU\nI2C 0x68'),
    (6.5,  0.3, '#1a1a2e', 'SD CARD\nSPI/FatFS'),
    (9.5,  0.3, '#1a1a2e', 'CAN BUS\n500 kbps'),
    (12.5, 0.3, '#1a1a2e', 'WHEEL SPEED\nADC CH0'),
]
for x, y, c, t in sensors:
    box(ax, x, y, 2.5, 1.5, c, t, fontsize=9)
    arrow(ax, x+1.25, 2.2, x+1.25, 1.8, color='#555555')

plt.tight_layout()
plt.savefig('docs/rtos_task_architecture.png', dpi=150,
            bbox_inches='tight', facecolor='#0d1117')
plt.close()
print("✅ Diagram 1 saved: docs/rtos_task_architecture.png")


# ─────────────────────────────────────────────
# DIAGRAM 2: System Block Diagram
# ─────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(16, 10))
ax.set_xlim(0, 16)
ax.set_ylim(0, 10)
ax.axis('off')
fig.patch.set_facecolor('#0d1117')
ax.set_facecolor('#0d1117')

ax.text(8, 9.5, 'System Block Diagram — Vehicle Telemetry System',
        ha='center', va='center', color='white', fontsize=14,
        fontweight='bold', fontfamily='monospace')

# Central MCU
box(ax, 5.5, 3.5, 5, 4, '#1f4e79',
    'STM32F407VGT6\nARM Cortex-M4\n168 MHz\n512KB Flash\n192KB RAM\nFreeRTOS', fontsize=10)

# Input sensors (left)
inputs = [
    (0.3, 8.0, 'GPS MODULE\nNEO-6M\nUART 9600'),
    (0.3, 6.0, 'IMU MPU6050\nAccel+Gyro\nI2C 400kHz'),
    (0.3, 4.0, 'WHEEL SPEED\nHall Sensor\nADC 12-bit'),
    (0.3, 2.0, 'TEMPERATURE\nNTC Thermistor\nADC 12-bit'),
]
for x, y, t in inputs:
    box(ax, x, y, 2.8, 1.2, '#1a472a', t, fontsize=9)
    ax.annotate("", xy=(5.5, y+0.6), xytext=(3.1, y+0.6),
                arrowprops=dict(arrowstyle='->', color='#2ea043', lw=1.5))

# Output/Storage (right)
outputs = [
    (12.9, 8.0, 'SD CARD\nFatFS CSV\nSPI 25MHz'),
    (12.9, 6.0, 'CAN BUS\nVehicle Network\n500 kbps'),
    (12.9, 4.0, 'BLE MODULE\nHC-05/ESP32\nWireless Telem.'),
    (12.9, 2.0, 'DEBUG UART\nST-Link\n115200 baud'),
]
for x, y, t in outputs:
    box(ax, x, y, 2.8, 1.2, '#4a1942', t, fontsize=9)
    ax.annotate("", xy=(12.9, y+0.6), xytext=(10.5, y+0.6),
                arrowprops=dict(arrowstyle='->', color='#d2a8ff', lw=1.5))

# Power supply
box(ax, 6.5, 0.2, 3, 0.9, '#3d2b00', '12V Automotive → 3.3V/5V LDO Regulators', fontsize=9)
arrow(ax, 8, 1.1, 8, 3.5, color='#ffa657')

# Labels on MCU interfaces
iface_labels = [
    (5.0, 8.4, 'UART2'),
    (5.0, 6.4, 'I2C1'),
    (5.0, 4.4, 'ADC1'),
    (5.0, 2.4, 'ADC2'),
    (10.5, 8.4, 'SPI1'),
    (10.5, 6.4, 'CAN1'),
    (10.5, 4.4, 'UART3'),
    (10.5, 2.4, 'UART1'),
]
for x, y, t in iface_labels:
    ax.text(x, y, t, color='#58a6ff', fontsize=8,
            ha='center', fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.2', facecolor='#0d1117',
                      edgecolor='#58a6ff', linewidth=0.8))

plt.tight_layout()
plt.savefig('docs/system_block_diagram.png', dpi=150,
            bbox_inches='tight', facecolor='#0d1117')
plt.close()
print("✅ Diagram 2 saved: docs/system_block_diagram.png")


# ─────────────────────────────────────────────
# DIAGRAM 3: Sensor Interface Schematic
# ─────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(16, 12))
fig.patch.set_facecolor('#0d1117')
fig.suptitle('Sensor Interface Schematics — STM32F407',
             color='white', fontsize=14, fontweight='bold',
             fontfamily='monospace')

schematics = [
    {
        'title': 'GPS Module (NEO-6M) — UART Interface',
        'color': '#1a472a',
        'pins': [
            ('NEO-6M VCC', '→', '3.3V'),
            ('NEO-6M GND', '→', 'GND'),
            ('NEO-6M TX',  '→', 'STM32 PA3 (UART2_RX)'),
            ('NEO-6M RX',  '→', 'STM32 PA2 (UART2_TX)'),
            ('NEO-6M PPS', '→', 'STM32 PA0 (TIM2_CH1)'),
        ],
        'config': 'Baud: 9600 | Protocol: NMEA 0183\nSentences: $GPRMC, $GPGGA\nTimeout: 2000ms → FAULT_GPS_LOSS'
    },
    {
        'title': 'IMU MPU6050 — I2C Interface',
        'color': '#1f3a6e',
        'pins': [
            ('MPU6050 VCC', '→', '3.3V'),
            ('MPU6050 GND', '→', 'GND'),
            ('MPU6050 SCL', '→', 'STM32 PB6 (I2C1_SCL)'),
            ('MPU6050 SDA', '→', 'STM32 PB7 (I2C1_SDA)'),
            ('MPU6050 INT', '→', 'STM32 PB0 (EXTI0)'),
            ('AD0 pin',     '→', 'GND → Address 0x68'),
        ],
        'config': 'Speed: 400kHz Fast Mode\nAccel: ±2g (16384 LSB/g)\nGyro: ±250°/s (131 LSB/°/s)\nTimeout: 500ms → FAULT_IMU_DISC'
    },
    {
        'title': 'SD Card — SPI Interface (FatFS)',
        'color': '#3d2b00',
        'pins': [
            ('SD VCC',  '→', '3.3V (100mA)'),
            ('SD GND',  '→', 'GND'),
            ('SD MOSI', '→', 'STM32 PA7 (SPI1_MOSI)'),
            ('SD MISO', '→', 'STM32 PA6 (SPI1_MISO)'),
            ('SD SCK',  '→', 'STM32 PA5 (SPI1_SCK)'),
            ('SD CS',   '→', 'STM32 PA4 (GPIO_OUT)'),
        ],
        'config': 'Init Speed: 400kHz | Op Speed: 25MHz\nFilesystem: FatFS R0.15\nFormat: CSV timestamped\nFallback: RAM ring buffer (512 entries)'
    },
    {
        'title': 'CAN Bus — CAN1 Interface',
        'color': '#4a1942',
        'pins': [
            ('TJA1050 VCC',  '→', '5V'),
            ('TJA1050 GND',  '→', 'GND'),
            ('TJA1050 TXD',  '→', 'STM32 PD1 (CAN1_TX)'),
            ('TJA1050 RXD',  '→', 'STM32 PD0 (CAN1_RX)'),
            ('CANH',         '→', 'CAN Bus High (120Ω term.)'),
            ('CANL',         '→', 'CAN Bus Low  (120Ω term.)'),
        ],
        'config': 'Bitrate: 500 kbps\nDiagnostic ID: 0x7FF\nTimeout: 1000ms → FAULT_CAN_TIMEOUT\nRecovery: Bus-off reset sequence'
    },
]

for ax, sch in zip(axes.flat, schematics):
    ax.set_facecolor('#0d1117')
    ax.axis('off')
    ax.set_title(sch['title'], color='#58a6ff', fontsize=10,
                 fontweight='bold', fontfamily='monospace', pad=10)

    y = 0.88
    for left, arrow_s, right in sch['pins']:
        ax.text(0.02, y, left, transform=ax.transAxes,
                color='#ffa657', fontsize=9, fontfamily='monospace')
        ax.text(0.42, y, arrow_s, transform=ax.transAxes,
                color='white', fontsize=9, ha='center')
        ax.text(0.48, y, right, transform=ax.transAxes,
                color='#2ea043', fontsize=9, fontfamily='monospace')
        y -= 0.11

    ax.text(0.02, y - 0.02, sch['config'],
            transform=ax.transAxes,
            color='#8b949e', fontsize=8, fontfamily='monospace',
            verticalalignment='top',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#161b22',
                      edgecolor='#30363d', linewidth=1))

plt.tight_layout()
plt.savefig('docs/sensor_interface_schematics.png', dpi=150,
            bbox_inches='tight', facecolor='#0d1117')
plt.close()
print("✅ Diagram 3 saved: docs/sensor_interface_schematics.png")
print("\n✅ All diagrams generated successfully!")
