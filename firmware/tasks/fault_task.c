/* =============================================================
 * Task: Fault Monitor — Priority 6 (HIGHEST)
 * Fixes applied:
 *   1. Starving watchdog — ticks initialized at boot
 *   2. Thread-safety — FAULT_Set/Clear use critical sections
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

#define FAULT_GPS_LOSS    (1 << 0)
#define FAULT_IMU_DISC    (1 << 1)
#define FAULT_CAN_TIMEOUT (1 << 2)
#define FAULT_SD_FAIL     (1 << 3)

#define IMU_TIMEOUT_MS  500
#define CAN_TIMEOUT_MS  1000
#define GPS_TIMEOUT_MS  2000

/* ── Shared fault register ────────────────────────────────────
 * Marked volatile so compiler never caches it in a register.
 * Protected by critical sections on every read-modify-write.  */
static volatile uint32_t fault_register = 0;

/* ── Watchdog ticks ───────────────────────────────────────── */
static volatile uint32_t last_imu_tick = 0;
static volatile uint32_t last_can_tick = 0;
static volatile uint32_t last_gps_tick = 0;

/* ── Thread-safe FAULT_Set ────────────────────────────────────
 * taskENTER_CRITICAL disables interrupts for the duration of
 * the read-modify-write so no other task or ISR can corrupt
 * the fault_register between the read and the write.          */
void FAULT_Set(uint32_t code) {
    taskENTER_CRITICAL();
    fault_register |= code;
    taskEXIT_CRITICAL();
}

/* ── Thread-safe FAULT_Clear ──────────────────────────────── */
void FAULT_Clear(uint32_t code) {
    taskENTER_CRITICAL();
    fault_register &= ~code;
    taskEXIT_CRITICAL();
}

/* ── Thread-safe FAULT_IsSet ──────────────────────────────── */
int FAULT_IsSet(uint32_t code) {
    taskENTER_CRITICAL();
    int result = (fault_register & code) != 0;
    taskEXIT_CRITICAL();
    return result;
}

/* ── Thread-safe snapshot for CAN broadcast ──────────────────
 * Reading a 32-bit value on Cortex-M4 is atomic, but we
 * snapshot it inside a critical section anyway to guarantee
 * consistency if fault_register ever becomes wider.           */
static uint32_t FAULT_Snapshot(void) {
    taskENTER_CRITICAL();
    uint32_t snapshot = fault_register;
    taskEXIT_CRITICAL();
    return snapshot;
}

/* ── Watchdog pet functions — called by other tasks ──────── */
void FAULT_UpdateGPSTick(void) {
    taskENTER_CRITICAL();
    last_gps_tick = xTaskGetTickCount();
    taskEXIT_CRITICAL();
}

void FAULT_UpdateIMUTick(void) {
    taskENTER_CRITICAL();
    last_imu_tick = xTaskGetTickCount();
    taskEXIT_CRITICAL();
}

void FAULT_UpdateCANTick(void) {
    taskENTER_CRITICAL();
    last_can_tick = xTaskGetTickCount();
    taskEXIT_CRITICAL();
}

void vTaskFaultMonitor(void *pvParameters) {

    /* Initialize watchdog ticks to NOW so we don't get
     * false faults before other tasks have started       */
    last_gps_tick = xTaskGetTickCount();
    last_imu_tick = xTaskGetTickCount();
    last_can_tick = xTaskGetTickCount();

    for (;;) {
        uint32_t now = xTaskGetTickCount();

        /* ── Snapshot ticks inside critical section ──────────
         * Prevents torn reads if an ISR updates a tick value
         * halfway through our comparison                      */
        uint32_t gps_tick, imu_tick, can_tick;
        taskENTER_CRITICAL();
        gps_tick = last_gps_tick;
        imu_tick = last_imu_tick;
        can_tick = last_can_tick;
        taskEXIT_CRITICAL();

        /* ── GPS watchdog ── */
        if ((now - gps_tick) > pdMS_TO_TICKS(GPS_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_GPS_LOSS)) {
                FAULT_Set(FAULT_GPS_LOSS);
                Telemetry_SetMode(TELEM_MODE_DEAD_RECKONING);
            }
        } else {
            FAULT_Clear(FAULT_GPS_LOSS);
        }

        /* ── IMU watchdog ── */
        if ((now - imu_tick) > pdMS_TO_TICKS(IMU_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_IMU_DISC)) {
                FAULT_Set(FAULT_IMU_DISC);
                SensorFusion_Disable();
            }
        } else {
            FAULT_Clear(FAULT_IMU_DISC);
        }

        /* ── CAN watchdog ── */
        if ((now - can_tick) > pdMS_TO_TICKS(CAN_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_CAN_TIMEOUT)) {
                FAULT_Set(FAULT_CAN_TIMEOUT);
                CAN_BusOff_Recovery();
                FaultLog_Write(FAULT_CAN_TIMEOUT, now);
            }
        } else {
            FAULT_Clear(FAULT_CAN_TIMEOUT);
        }

        /* ── Broadcast fault register on CAN 0x7FF ── */
        uint32_t fr = FAULT_Snapshot();
        if (fr != 0) {
            CAN_Frame_t diag;
            diag.id      = 0x7FF;
            diag.dlc     = 4;
            diag.data[0] = (fr >> 24) & 0xFF;
            diag.data[1] = (fr >> 16) & 0xFF;
            diag.data[2] = (fr >>  8) & 0xFF;
            diag.data[3] = (fr)       & 0xFF;
            CAN_Transmit(&diag);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
