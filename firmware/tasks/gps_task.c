/* =============================================================
 * Task: GPS Parsing
 * Protocol: NMEA 0183 over UART at 9600 baud
 * Sentences: $GPRMC, $GPGGA
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "gps_task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define GPS_TIMEOUT_MS   2000   /* Fault if no fix for 2 seconds */
#define NMEA_BUF_SIZE    128

static uint32_t last_fix_tick = 0;
static uint8_t  gps_lost      = 0;

/* Simple NMEA $GPRMC parser */
static int parse_GPRMC(const char *sentence, GPS_Data_t *out) {
    /* Format: $GPRMC,HHMMSS.ss,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,... */
    char buf[NMEA_BUF_SIZE];
    strncpy(buf, sentence, NMEA_BUF_SIZE - 1);

    char *token = strtok(buf, ",");
    if (!token || strcmp(token, "$GPRMC") != 0) return -1;

    token = strtok(NULL, ",");
    if (!token) return -1;
    /* Parse time: HHMMSS.ss */
    out->utc_time = atof(token);

    token = strtok(NULL, ",");
    if (!token || *token != 'A') {  /* 'A' = valid fix */
        out->fix_valid = 0;
        return 0;
    }
    out->fix_valid = 1;

    /* Latitude: DDMM.MMMM */
    token = strtok(NULL, ",");
    if (!token) return -1;
    double raw_lat = atof(token);
    int lat_deg = (int)(raw_lat / 100);
    out->latitude = lat_deg + (raw_lat - lat_deg * 100) / 60.0;

    token = strtok(NULL, ",");
    if (token && *token == 'S') out->latitude *= -1;

    /* Longitude: DDDMM.MMMM */
    token = strtok(NULL, ",");
    if (!token) return -1;
    double raw_lon = atof(token);
    int lon_deg = (int)(raw_lon / 100);
    out->longitude = lon_deg + (raw_lon - lon_deg * 100) / 60.0;

    token = strtok(NULL, ",");
    if (token && *token == 'W') out->longitude *= -1;

    /* Speed in knots → km/h */
    token = strtok(NULL, ",");
    if (token) out->speed_kmh = atof(token) * 1.852f;

    /* Heading (course over ground) */
    token = strtok(NULL, ",");
    if (token) out->heading = atof(token);

    return 1;
}

void vTaskGPSParsing(void *pvParameters) {
    char     nmea_buf[NMEA_BUF_SIZE];
    GPS_Data_t gps_data;

    for (;;) {
        /* Read one NMEA sentence from UART (blocking with timeout) */
        int len = UART_ReadLine(GPS_UART_PORT, nmea_buf, NMEA_BUF_SIZE,
                                pdMS_TO_TICKS(GPS_TIMEOUT_MS));

        if (len <= 0) {
            /* ---- FAULT: GPS Timeout ---- */
            gps_lost = 1;
            gps_data.fix_valid = 0;
            FAULT_Set(FAULT_GPS_TIMEOUT);
            xQueueSend(xGPSQueue, &gps_data, 0);
            continue;
        }

        if (strncmp(nmea_buf, "$GPRMC", 6) == 0) {
            int result = parse_GPRMC(nmea_buf, &gps_data);
            if (result == 1 && gps_data.fix_valid) {
                last_fix_tick = xTaskGetTickCount();
                gps_lost = 0;
                FAULT_Clear(FAULT_GPS_TIMEOUT);
                gps_data.timestamp_ms = last_fix_tick;
                xQueueSend(xGPSQueue, &gps_data, 0);
            }
        }
    }
}
