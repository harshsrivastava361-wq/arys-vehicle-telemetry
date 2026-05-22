/* =============================================================
 * Task: Data Logger
 * Storage: SD card via SPI (FATFS)
 * Format: CSV with timestamp
 * Fallback: RAM ring buffer on SD failure
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "ff.h"   /* FatFS */
#include "logger_task.h"
#include <stdio.h>
#include <string.h>

#define RAM_BUFFER_SIZE     512
#define CSV_HEADER  "timestamp_ms,speed_kmh,rpm,accel_x,accel_y,accel_z," \
                    "roll,pitch,g_force,latitude,longitude,heading,faults\n"

/* RAM fallback ring buffer */
static Log_Entry_t ram_buffer[RAM_BUFFER_SIZE];
static uint16_t    ram_head = 0;
static uint8_t     use_ram  = 0;

static FATFS fs;
static FIL   log_file;

static int open_log_file(void) {
    FRESULT res = f_mount(&fs, "", 1);
    if (res != FR_OK) return -1;

    char filename[32];
    snprintf(filename, sizeof(filename), "telem_%lu.csv",
             (unsigned long)xTaskGetTickCount());

    res = f_open(&log_file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) return -1;

    UINT bw;
    f_write(&log_file, CSV_HEADER, strlen(CSV_HEADER), &bw);
    return 0;
}

void DataLogger_UseRAMBuffer(void) {
    use_ram = 1;
}

void vTaskDataLogger(void *pvParameters) {
    Log_Entry_t entry;
    char line[256];

    /* Try to open SD card */
    if (open_log_file() != 0) {
        FAULT_Set(FAULT_SD_FAIL);
        use_ram = 1;
    }

    for (;;) {
        /* Wait for a log entry (block up to 100ms) */
        if (xQueueReceive(xLogQueue, &entry, pdMS_TO_TICKS(100)) == pdTRUE) {

            snprintf(line, sizeof(line),
                "%lu,%.2f,%.1f,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%.6f,%.6f,%.1f,0x%02X\n",
                entry.timestamp_ms,
                entry.speed_kmh,
                entry.wheel_rpm,
                entry.accel_x, entry.accel_y, entry.accel_z,
                entry.roll, entry.pitch, entry.g_force,
                entry.latitude, entry.longitude, entry.heading,
                entry.fault_flags);

            if (!use_ram) {
                /* Write to SD card with mutex protection */
                if (xSemaphoreTake(xSDCardMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    UINT bw;
                    FRESULT res = f_write(&log_file, line, strlen(line), &bw);
                    if (res != FR_OK || bw != strlen(line)) {
                        FAULT_Set(FAULT_SD_FAIL);
                        use_ram = 1;
                    }
                    /* Flush every 10 entries to avoid data loss */
                    if (entry.timestamp_ms % 10 == 0) f_sync(&log_file);
                    xSemaphoreGive(xSDCardMutex);
                }
            } else {
                /* Store in RAM ring buffer */
                ram_buffer[ram_head % RAM_BUFFER_SIZE] = entry;
                ram_head++;
            }
        }
    }
}
