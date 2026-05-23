/* =============================================================
 * Task: GPS Parsing
 * Protocol: NMEA 0183 via UART at 9600 baud
 * Fix 1: strtok replaced with strtok_r (thread-safe)
 * Fix 2: Fault logic removed — fault_task.c owns all timeouts
 * Fix 3: Explicit null termination after strncpy — if source
 *        fills entire buffer, strncpy won't add \0, causing
 *        strtok_r to march past the array into random RAM.
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>

#define NMEA_BUF_SIZE  128
#define GPS_READ_MS    1000

static int parse_GPRMC(const char *sentence, GPS_Data_t *out) {
    char  buf[NMEA_BUF_SIZE];
    char *saveptr = NULL;

    /* ── Safe copy with guaranteed null termination ───────────
     * strncpy copies up to NMEA_BUF_SIZE-1 characters.
     * The final byte is then EXPLICITLY set to '\0'.
     * This is safe even if sentence fills the entire buffer.
     * Without this, a 128-byte malformed frame has NO null
     * terminator and strtok_r walks into undefined memory.    */
    strncpy(buf, sentence, NMEA_BUF_SIZE - 1);
    buf[NMEA_BUF_SIZE - 1] = '\0';  /* always force null terminator */

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

    /* ── Also null-terminate the receive buffer at init ────────
     * Prevents reading garbage if UART_ReadLine returns partial
     * data on first call before the buffer is ever written     */
    memset(nmea_buf, 0, sizeof(nmea_buf));

    for (;;) {
        int len = UART_ReadLine(GPS_UART_PORT, nmea_buf,
                                NMEA_BUF_SIZE - 1,
                                pdMS_TO_TICKS(GPS_READ_MS));

        if (len <= 0) continue;

        /* Force null termination on received data too */
        nmea_buf[len] = '\0';

        if (strncmp(nmea_buf, "$GPRMC", 6) != 0) continue;

        int result = parse_GPRMC(nmea_buf, &gps_data);

        if (result == 1 && gps_data.fix_valid) {
            gps_data.timestamp_ms = xTaskGetTickCount();
            FAULT_UpdateGPSTick();
            xQueueSend(xGPSQueue, &gps_data, 0);
        }
    }
}
