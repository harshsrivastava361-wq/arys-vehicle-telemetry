/* =============================================================
 * Task: GPS Parsing
 * Protocol: NMEA 0183 via UART at 9600 baud
 * Fix 1: strtok replaced with strtok_r (thread-safe)
 * Fix 2: Fault logic removed — fault_task.c owns all timeouts.
 *        GPS task has ONE job: parse sentence, pet watchdog.
 *        If GPS goes silent, fault_task detects the timeout
 *        automatically because FAULT_UpdateGPSTick() stops
 *        being called. Clean separation of responsibilities.
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>

#define NMEA_BUF_SIZE  128
#define GPS_READ_MS    1000   /* How long to wait for a sentence */

static int parse_GPRMC(const char *sentence, GPS_Data_t *out) {
    char  buf[NMEA_BUF_SIZE];
    char *saveptr = NULL;   /* strtok_r state — lives on stack, thread-safe */

    strncpy(buf, sentence, NMEA_BUF_SIZE - 1);
    buf[NMEA_BUF_SIZE - 1] = '\0';

    char *token = strtok_r(buf, ",", &saveptr);
    if (!token || strcmp(token, "$GPRMC") != 0) return -1;

    /* Time: HHMMSS.ss */
    token = strtok_r(NULL, ",", &saveptr);
    if (!token) return -1;
    out->utc_time = atof(token);

    /* Status: A=valid V=void */
    token = strtok_r(NULL, ",", &saveptr);
    if (!token || *token != 'A') {
        out->fix_valid = 0;
        return 0;
    }
    out->fix_valid = 1;

    /* Latitude: DDMM.MMMM */
    token = strtok_r(NULL, ",", &saveptr);
    if (!token) return -1;
    double raw_lat = atof(token);
    int lat_deg    = (int)(raw_lat / 100);
    out->latitude  = lat_deg + (raw_lat - lat_deg * 100) / 60.0;

    token = strtok_r(NULL, ",", &saveptr);
    if (token && *token == 'S') out->latitude *= -1;

    /* Longitude: DDDMM.MMMM */
    token = strtok_r(NULL, ",", &saveptr);
    if (!token) return -1;
    double raw_lon = atof(token);
    int lon_deg    = (int)(raw_lon / 100);
    out->longitude = lon_deg + (raw_lon - lon_deg * 100) / 60.0;

    token = strtok_r(NULL, ",", &saveptr);
    if (token && *token == 'W') out->longitude *= -1;

    /* Speed knots → km/h */
    token = strtok_r(NULL, ",", &saveptr);
    if (token) out->speed_kmh = atof(token) * 1.852f;

    /* Heading */
    token = strtok_r(NULL, ",", &saveptr);
    if (token) out->heading = atof(token);

    return 1;
}

void vTaskGPSParsing(void *pvParameters) {
    char       nmea_buf[NMEA_BUF_SIZE];
    GPS_Data_t gps_data;

    for (;;) {
        /* Block waiting for one NMEA sentence from UART */
        int len = UART_ReadLine(GPS_UART_PORT, nmea_buf,
                                NMEA_BUF_SIZE,
                                pdMS_TO_TICKS(GPS_READ_MS));

        /* ── No sentence received ─────────────────────────────
         * We do NOT call FAULT_Set() here anymore.
         * We simply do nothing — FAULT_UpdateGPSTick() won't
         * be called, so fault_task.c will detect the silence
         * after GPS_TIMEOUT_MS and trigger the fault itself.
         * One place owns fault logic. No duplication.         */
        if (len <= 0) continue;

        /* ── Parse only $GPRMC sentences ── */
        if (strncmp(nmea_buf, "$GPRMC", 6) != 0) continue;

        int result = parse_GPRMC(nmea_buf, &gps_data);

        /* ── Valid fix received ───────────────────────────────
         * Pet the watchdog — this is the GPS task's ONLY job.
         * fault_task.c sees the tick update and knows GPS is
         * alive. If we stop calling this, fault triggers.     */
        if (result == 1 && gps_data.fix_valid) {
            gps_data.timestamp_ms = xTaskGetTickCount();
            FAULT_UpdateGPSTick();
            xQueueSend(xGPSQueue, &gps_data, 0);
        }
    }
}
