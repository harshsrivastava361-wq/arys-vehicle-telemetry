/* ================================================================
 * Arys Garage — Real-Time Vehicle Telemetry System
 * File     : telemetry_task.c
 * Priority : 1  (LOWEST — best-effort wireless transmission)
 *
 * Transmits live telemetry over BLE/WiFi (ESP32 or HC-05).
 * When SD card fails, this task boosts its rate and drains the
 * RAM ring buffer so no data is lost.
 *
 * Packet format (16 bytes, sent as binary over UART to BLE chip):
 *   [0..1]  speed_kmh  ×10  uint16
 *   [2..3]  rpm        ×1   uint16
 *   [4..5]  g_force    ×100 int16
 *   [6..7]  roll       ×10  int16
 *   [8..9]  pitch      ×10  int16
 *   [10..11] heading   ×10  uint16
 *   [12]    fault_flags      uint8
 *   [13..15] reserved        uint8 ×3
 * ================================================================ */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "telemetry.h"
#include <string.h>

#define WIRELESS_NORMAL_MS   100   /* 10 Hz normal rate  */
#define WIRELESS_BOOST_MS     20   /* 50 Hz on SD failure */
#define PACKET_LEN            16

static TelemetryMode_t current_mode = TELEM_MODE_NORMAL;
static uint8_t         high_priority = 0;

void Telemetry_SetMode(TelemetryMode_t mode) {
    current_mode = mode;
}

void Wireless_SetHighPriority(void) {
    high_priority = 1;
}

/* ── Build 16-byte wireless packet from log entry ────────────── */
static void build_packet(uint8_t *pkt, const Log_Entry_t *e)
{
    uint16_t spd = (uint16_t)(e->speed_kmh * 10);
    uint16_t rpm = (uint16_t)(e->wheel_rpm);
    int16_t  gf  = (int16_t)(e->g_force   * 100);
    int16_t  rol = (int16_t)(e->roll       * 10);
    int16_t  pit = (int16_t)(e->pitch      * 10);
    uint16_t hdg = (uint16_t)(e->heading   * 10);

    pkt[0]  = (spd >> 8) & 0xFF;  pkt[1]  = spd & 0xFF;
    pkt[2]  = (rpm >> 8) & 0xFF;  pkt[3]  = rpm & 0xFF;
    pkt[4]  = (gf  >> 8) & 0xFF;  pkt[5]  = gf  & 0xFF;
    pkt[6]  = (rol >> 8) & 0xFF;  pkt[7]  = rol & 0xFF;
    pkt[8]  = (pit >> 8) & 0xFF;  pkt[9]  = pit & 0xFF;
    pkt[10] = (hdg >> 8) & 0xFF;  pkt[11] = hdg & 0xFF;
    pkt[12] = e->fault_flags;
    pkt[13] = pkt[14] = pkt[15] = 0;
}

void vTaskWirelessTelemetry(void *pvParameters)
{
    Log_Entry_t entry;
    uint8_t     packet[PACKET_LEN];
    uint32_t    delay_ms = WIRELESS_NORMAL_MS;

    for (;;) {
        /* Boost rate if SD card has failed */
        delay_ms = high_priority ? WIRELESS_BOOST_MS
                                 : WIRELESS_NORMAL_MS;

        /* Read latest entry from log queue (non-blocking) */
        if (xQueueReceive(xLogQueue, &entry, 0) == pdTRUE) {
            build_packet(packet, &entry);
            Wireless_Send(packet, PACKET_LEN);
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
