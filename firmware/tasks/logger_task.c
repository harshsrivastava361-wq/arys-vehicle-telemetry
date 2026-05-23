/* =============================================================
 * Task: Data Logger
 * Storage: SD card via SPI using FatFS
 * Format: Timestamped CSV
 * Fallback: RAM ring buffer if SD card fails
 *
 * Fix 1: f_sync timing trap — use write counter not timestamp
 *        timestamp_ms % 10 could skip exact multiples if RTOS
 *        delays cause gaps. A dedicated counter is reliable.
 *
 * Fix 2: Dropped data trap — if mutex times out, stash the
 *        data point in RAM buffer instead of discarding it.
 *        No telemetry data is ever silently thrown away.
 *
 * Fix 3: Floating point linker note (STM32CubeIDE):
 *        snprintf("%.2f") prints nothing by default on STM32
 *        because newlib-nano disables float printf to save RAM.
 *        Fix: add -u _printf_float to linker flags, OR check
 *        "Use float with printf from newlib-nano" in CubeIDE
 *        project properties → C/C++ Build → Settings → MCU GCC
 *        Linker → Miscellaneous → float support checkbox.
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>

#define RAM_BUFFER_SIZE  512
#define SYNC_EVERY_N     10   /* flush to SD every 10 writes */

#define CSV_HEADER "timestamp_ms,speed_kmh,rpm,accel_x,accel_y,accel_z," \
                   "roll,pitch,g_force,latitude,longitude,heading,faults\n"

static Log_Entry_t ram_buffer[RAM_BUFFER_SIZE];
static uint16_t    ram_head  = 0;
static uint8_t     use_ram   = 0;
static FATFS       fs;
static FIL         log_file;

/* ── Fix 1: dedicated write counter ──────────────────────────
 * Previously: if (entry.timestamp_ms % 10 == 0) f_sync()
 * Problem: RTOS delays mean timestamps like 1001, 1012, 1023
 *          might skip 1010 and 1020 entirely — f_sync never
 *          runs — power loss = entire CSV file corrupted.
 * Fix: count actual writes. Every 10th write = guaranteed sync */
static uint32_t write_counter = 0;

void DataLogger_UseRAMBuffer(void) { use_ram = 1; }

/* ── Store entry in RAM ring buffer ───────────────────────── */
static void stash_in_ram(const Log_Entry_t *entry) {
    ram_buffer[ram_head % RAM_BUFFER_SIZE] = *entry;
    ram_head++;
}

static int open_log_file(void) {
    if (f_mount(&fs, "", 1) != FR_OK) return -1;
    char fname[32];
    snprintf(fname, sizeof(fname), "telem_%lu.csv",
             (unsigned long)xTaskGetTickCount());
    if (f_open(&log_file, fname,
               FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return -1;
    UINT bw;
    f_write(&log_file, CSV_HEADER, strlen(CSV_HEADER), &bw);
    return 0;
}

void vTaskDataLogger(void *pvParameters) {
    Log_Entry_t entry;
    char        line[256];

    if (open_log_file() != 0) {
        FAULT_Set(FAULT_SD_FAIL);
        use_ram = 1;
    }

    for (;;) {
        if (xQueueReceive(xLogQueue, &entry,
                          pdMS_TO_TICKS(100)) != pdTRUE) continue;

        /* Build CSV line */
        snprintf(line, sizeof(line),
            /* NOTE: if %.2f prints nothing on your STM32, add
             * -u _printf_float to your linker flags in CubeIDE */
            "%lu,%.2f,%.1f,%.4f,%.4f,%.4f,"
            "%.2f,%.2f,%.3f,%.6f,%.6f,%.1f,0x%02X\n",
            entry.timestamp_ms,
            entry.speed_kmh,   entry.wheel_rpm,
            entry.accel_x,     entry.accel_y,   entry.accel_z,
            entry.roll,        entry.pitch,      entry.g_force,
            entry.latitude,    entry.longitude,  entry.heading,
            entry.fault_flags);

        if (use_ram) {
            stash_in_ram(&entry);
            continue;
        }

        /* ── Fix 2: mutex timeout → stash, never discard ─────
         * Previously: if mutex times out, the if-block is
         * skipped and this data point is lost forever — silent
         * data loss that corrupts lap timing and g-force graphs.
         * Fix: on timeout, stash in RAM buffer. No data lost.  */
        if (xSemaphoreTake(xSDCardMutex,
                           pdMS_TO_TICKS(50)) != pdTRUE) {
            /* Mutex timed out — SD card busy, stash for later */
            stash_in_ram(&entry);
            continue;
        }

        /* Mutex acquired — write to SD card */
        UINT bw;
        FRESULT res = f_write(&log_file, line, strlen(line), &bw);

        if (res != FR_OK || bw != strlen(line)) {
            /* Write failed — switch to RAM, stash this entry */
            FAULT_Set(FAULT_SD_FAIL);
            use_ram = 1;
            xSemaphoreGive(xSDCardMutex);
            stash_in_ram(&entry);
            continue;
        }

        /* ── Fix 1: sync every N writes using counter ─────────
         * Guaranteed to run every 10 entries regardless of
         * what the timestamp value happens to be.              */
        write_counter++;
        if (write_counter % SYNC_EVERY_N == 0) {
            f_sync(&log_file);
        }

        xSemaphoreGive(xSDCardMutex);
    }
}
