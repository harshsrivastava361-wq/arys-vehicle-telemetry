/* =============================================================
 * Task: GPS Parsing
 * Protocol: NMEA 0183 via UART at 9600 baud
 * Fix: Replaced strtok() with strtok_r() for thread safety
 *      strtok() uses internal static state — if RTOS preempts
 *      mid-parse, the pointer corrupts and causes HardFault.
 *      strtok_r() takes a caller-supplied saveptr, making it
 *      fully reentrant and safe in an RTOS environment.
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define GPS_TIMEOUT_MS  2000
#define NMEA_BUF_SIZE   128

static int parse_GPRMC(const char *sentence, GPS_Data_t *out) {
    char buf[NMEA_BUF_SIZE];
    strncpy(buf, sentence, NMEA_BUF_SIZE - 1);
    buf[NMEA_BUF_SIZE - 1] = '\0';

    /* ── saveptr is LOCAL to this function call ───────────────
     * Each call to parse_GPRMC gets its own saveptr on the
     * stack. No shared static state = no concurrency crash.   */
    char *saveptr = NULL;

    char *token = strtok_r(buf, ",", &saveptr);
    if (!token || strcmp(token, "$GPRMC") != 0) return -1;

    /* Time field: HHMMSS.ss */
    token = strtok_r(NULL, ",", &saveptr);
    if (!token) return -1;
    out->utc_time = atof(token);

    /* Status: A=valid, V=invalid */
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
    int lat_deg = (int)(raw_lat / 100);
    out->latitude = lat_deg + (raw_lat - lat_deg * 100) / 60.0;

    /* N/S hemisphere */
    token = strtok_r(NULL, ",", &saveptr);
    if (token && *token == 'S') out->latitude *= -1;

    /* Longitude: DDDMM.MMMM */
    token = strtok_r(NULL, ",", &saveptr);
    if (!token) return -1;
    double raw_lon = atof(token);
    int lon_deg = (int)(raw_lon / 100);
    out->longitude = lon_deg + (raw_lon - lon_deg * 100) / 60.0;

    /* E/W hemisphere */
    token = strtok_r(NULL, ",", &saveptr);
    if (token && *token == 'W') out->longitude *= -1;

    /* Speed in knots → km/h */
    token = strtok_r(NULL, ",", &saveptr);
    if (token) out->speed_kmh = atof(token) * 1.852f;

    /* Course over ground (heading) */
    token = strtok_r(NULL, ",", &saveptr);
    if (token) out->heading = atof(token);

    return 1;
}

void vTaskGPSParsing(void *pvParameters) {
    char       nmea_buf[NMEA_BUF_SIZE];
    GPS_Data_t gps_data;

    for (;;) {
        int len = UART_ReadLine(GPS_UART_PORT, nmea_buf,
                                NMEA_BUF_SIZE,
                                pdMS_TO_TICKS(GPS_TIMEOUT_MS));

        if (len <= 0) {
            /* Timeout — no valid sentence received */
            gps_data.fix_valid = 0;
            FAULT_Set(FAULT_GPS_TIMEOUT);
            xQueueSend(xGPSQueue, &gps_data, 0);
            continue;
        }

        if (strncmp(nmea_buf, "$GPRMC", 6) == 0) {
            if (parse_GPRMC(nmea_buf, &gps_data) == 1
                    && gps_data.fix_valid) {
                gps_data.timestamp_ms = xTaskGetTickCount();
                FAULT_Clear(FAULT_GPS_TIMEOUT);
                /* Pet the watchdog so fault monitor knows GPS is alive */
                FAULT_UpdateGPSTick();
                xQueueSend(xGPSQueue, &gps_data, 0);
            }
        }
    }
}
