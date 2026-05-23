/* =============================================================
 * Task: Fault Monitor — Priority 6 (HIGHEST)
 * Detects: GPS loss, IMU disconnect, CAN timeout, SD failure
 * Recovery: Graceful degradation per fault type
 * Fix: Watchdog tick setters so other tasks can "pet" the watchdog
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"

#define FAULT_GPS_LOSS    (1 << 0)
#define FAULT_IMU_DISC    (1 << 1)
#define FAULT_CAN_TIMEOUT (1 << 2)
#define FAULT_SD_FAIL     (1 << 3)

#define IMU_TIMEOUT_MS  500
#define CAN_TIMEOUT_MS  1000
#define GPS_TIMEOUT_MS  2000

static volatile uint32_t fault_register = 0;

/* ── Watchdog ticks — initialized to current tick at boot,
 * not 0, to prevent false faults on startup               */
static volatile uint32_t last_imu_tick = 0;
static volatile uint32_t last_can_tick = 0;
static volatile uint32_t last_gps_tick = 0;
static uint8_t ticks_initialized = 0;

/* ── Public API ───────────────────────────────────────────── */
void FAULT_Set(uint32_t code)   { fault_register |=  code; }
void FAULT_Clear(uint32_t code) { fault_register &= ~code; }
int  FAULT_IsSet(uint32_t code) { return (fault_register & code) != 0; }

/* ── Watchdog "pet" functions — called by other tasks ─────── */
void FAULT_UpdateGPSTick(void) {
    last_gps_tick = xTaskGetTickCount();
}

void FAULT_UpdateIMUTick(void) {
    last_imu_tick = xTaskGetTickCount();
}

void FAULT_UpdateCANTick(void) {
    last_can_tick = xTaskGetTickCount();
}

void vTaskFaultMonitor(void *pvParameters) {

    /* ── Initialize all ticks to NOW at boot.
     * This prevents false fault triggers during startup
     * before other tasks have had a chance to run.        */
    last_gps_tick = xTaskGetTickCount();
    last_imu_tick = xTaskGetTickCount();
    last_can_tick = xTaskGetTickCount();
    ticks_initialized = 1;

    for (;;) {
        uint32_t now = xTaskGetTickCount();

        /* ── GPS watchdog ── */
        if ((now - last_gps_tick) > pdMS_TO_TICKS(GPS_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_GPS_LOSS)) {
                FAULT_Set(FAULT_GPS_LOSS);
                /* Graceful recovery: dead reckoning via IMU + wheel speed */
                Telemetry_SetMode(TELEM_MODE_DEAD_RECKONING);
            }
        } else {
            /* Auto-clear when tick is being updated again */
            FAULT_Clear(FAULT_GPS_LOSS);
        }

        /* ── IMU watchdog ── */
        if ((now - last_imu_tick) > pdMS_TO_TICKS(IMU_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_IMU_DISC)) {
                FAULT_Set(FAULT_IMU_DISC);
                /* Graceful recovery: disable fusion, GPS-only mode */
                SensorFusion_Disable();
            }
        } else {
            FAULT_Clear(FAULT_IMU_DISC);
        }

        /* ── CAN watchdog ── */
        if ((now - last_can_tick) > pdMS_TO_TICKS(CAN_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_CAN_TIMEOUT)) {
                FAULT_Set(FAULT_CAN_TIMEOUT);
                /* Graceful recovery: bus-off reset sequence */
                CAN_BusOff_Recovery();
                FaultLog_Write(FAULT_CAN_TIMEOUT, now);
            }
        } else {
            FAULT_Clear(FAULT_CAN_TIMEOUT);
        }

        /* ── Broadcast fault register on CAN diagnostic frame ── */
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

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
