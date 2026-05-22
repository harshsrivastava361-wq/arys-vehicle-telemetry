/* =============================================================
 * Task: Fault Monitor (Highest Priority — Safety Critical)
 * Monitors: GPS loss, IMU disconnect, CAN timeout, SD failure
 * Recovery: Graceful degradation per fault type
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "fault_task.h"
#include <stdio.h>

/* Fault register — bitmask */
static volatile uint32_t fault_register = 0x00000000;

/* Fault codes */
#define FAULT_GPS_LOSS      (1 << 0)
#define FAULT_IMU_DISC      (1 << 1)
#define FAULT_CAN_TIMEOUT   (1 << 2)
#define FAULT_SD_FAIL       (1 << 3)

/* Thresholds */
#define IMU_TIMEOUT_MS      500
#define CAN_TIMEOUT_MS      1000
#define GPS_TIMEOUT_MS      2000

static uint32_t last_imu_tick = 0;
static uint32_t last_can_tick = 0;
static uint32_t last_gps_tick = 0;

void FAULT_Set(uint32_t fault_code) {
    fault_register |= fault_code;
}

void FAULT_Clear(uint32_t fault_code) {
    fault_register &= ~fault_code;
}

/* Returns 1 if fault is active */
int FAULT_IsSet(uint32_t fault_code) {
    return (fault_register & fault_code) != 0;
}

static void handle_gps_loss(void) {
    /* Graceful degradation: switch to dead reckoning using IMU + wheel speed */
    printf("[FAULT] GPS signal lost — activating dead reckoning mode\n");
    Telemetry_SetMode(TELEM_MODE_DEAD_RECKONING);
    LED_SetFaultIndicator(LED_GPS_FAULT);
}

static void handle_imu_disconnect(void) {
    /* Disable orientation estimation, continue GPS-only mode */
    printf("[FAULT] IMU disconnected — orientation data unavailable\n");
    SensorFusion_Disable();
    LED_SetFaultIndicator(LED_IMU_FAULT);
}

static void handle_can_timeout(void) {
    /* Attempt CAN bus reset, log error frame */
    printf("[FAULT] CAN timeout — attempting bus reset\n");
    CAN_BusOff_Recovery();
    FaultLog_Write(FAULT_CAN_TIMEOUT, xTaskGetTickCount());
}

static void handle_sd_failure(void) {
    /* Switch to RAM ring buffer, transmit wirelessly instead */
    printf("[FAULT] SD card write failure — switching to RAM buffer + wireless\n");
    DataLogger_UseRAMBuffer();
    Wireless_SetPriority(WIRELESS_HIGH_PRIORITY);
}

void vTaskFaultMonitor(void *pvParameters) {
    for (;;) {
        uint32_t now = xTaskGetTickCount();

        /* ---- Check GPS timeout ---- */
        if ((now - last_gps_tick) > pdMS_TO_TICKS(GPS_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_GPS_LOSS)) {
                FAULT_Set(FAULT_GPS_LOSS);
                handle_gps_loss();
            }
        }

        /* ---- Check IMU timeout ---- */
        if ((now - last_imu_tick) > pdMS_TO_TICKS(IMU_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_IMU_DISC)) {
                FAULT_Set(FAULT_IMU_DISC);
                handle_imu_disconnect();
            }
        }

        /* ---- Check CAN timeout ---- */
        if ((now - last_can_tick) > pdMS_TO_TICKS(CAN_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_CAN_TIMEOUT)) {
                FAULT_Set(FAULT_CAN_TIMEOUT);
                handle_can_timeout();
            }
        }

        /* ---- Report fault register over CAN (0x7FF = diagnostic ID) ---- */
        if (fault_register != 0) {
            CAN_Frame_t diag;
            diag.id      = 0x7FF;
            diag.dlc     = 4;
            diag.data[0] = (fault_register >> 24) & 0xFF;
            diag.data[1] = (fault_register >> 16) & 0xFF;
            diag.data[2] = (fault_register >>  8) & 0xFF;
            diag.data[3] = (fault_register)       & 0xFF;
            CAN_Transmit(&diag);
        }

        /* Run every 50ms — fault detection latency acceptable */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
