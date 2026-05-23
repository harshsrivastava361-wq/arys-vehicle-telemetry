/* =============================================================
 * Task: Fault Monitor — Priority 6 (HIGHEST)
 * Fix 1: Starving watchdog  — ticks initialized at boot
 * Fix 2: Thread-safety      — critical sections on fault_register
 * Fix 3: CAN bus flooding   — rate-limited to 1Hz + state change
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

#define FAULT_GPS_LOSS       (1 << 0)
#define FAULT_IMU_DISC       (1 << 1)
#define FAULT_CAN_TIMEOUT    (1 << 2)
#define FAULT_SD_FAIL        (1 << 3)

#define IMU_TIMEOUT_MS       500
#define CAN_TIMEOUT_MS       1000
#define GPS_TIMEOUT_MS       2000
#define CAN_DIAG_INTERVAL_MS 1000

static volatile uint32_t fault_register    = 0;
static volatile uint32_t last_imu_tick     = 0;
static volatile uint32_t last_can_tick     = 0;
static volatile uint32_t last_gps_tick     = 0;
static uint32_t prev_fault_register        = 0;
static uint32_t last_can_diag_tick         = 0;

void FAULT_Set(uint32_t code) {
    taskENTER_CRITICAL();
    fault_register |= code;
    taskEXIT_CRITICAL();
}

void FAULT_Clear(uint32_t code) {
    taskENTER_CRITICAL();
    fault_register &= ~code;
    taskEXIT_CRITICAL();
}

int FAULT_IsSet(uint32_t code) {
    taskENTER_CRITICAL();
    int result = (fault_register & code) != 0;
    taskEXIT_CRITICAL();
    return result;
}

static uint32_t FAULT_Snapshot(void) {
    taskENTER_CRITICAL();
    uint32_t snapshot = fault_register;
    taskEXIT_CRITICAL();
    return snapshot;
}

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

static void send_diag_frame(uint32_t fr) {
    CAN_Frame_t diag;
    diag.id      = 0x7FF;
    diag.dlc     = 4;
    diag.data[0] = (fr >> 24) & 0xFF;
    diag.data[1] = (fr >> 16) & 0xFF;
    diag.data[2] = (fr >>  8) & 0xFF;
    diag.data[3] = (fr)       & 0xFF;
    CAN_Transmit(&diag);
    last_can_diag_tick  = xTaskGetTickCount();
    prev_fault_register = fr;
}

void vTaskFaultMonitor(void *pvParameters) {

    last_gps_tick      = xTaskGetTickCount();
    last_imu_tick      = xTaskGetTickCount();
    last_can_tick      = xTaskGetTickCount();
    last_can_diag_tick = xTaskGetTickCount();

    for (;;) {
        uint32_t now = xTaskGetTickCount();

        uint32_t gps_tick, imu_tick, can_tick;
        taskENTER_CRITICAL();
        gps_tick = last_gps_tick;
        imu_tick = last_imu_tick;
        can_tick = last_can_tick;
        taskEXIT_CRITICAL();

        /* GPS watchdog */
        if ((now - gps_tick) > pdMS_TO_TICKS(GPS_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_GPS_LOSS)) {
                FAULT_Set(FAULT_GPS_LOSS);
                Telemetry_SetMode(TELEM_MODE_DEAD_RECKONING);
            }
        } else {
            FAULT_Clear(FAULT_GPS_LOSS);
        }

        /* IMU watchdog */
        if ((now - imu_tick) > pdMS_TO_TICKS(IMU_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_IMU_DISC)) {
                FAULT_Set(FAULT_IMU_DISC);
                SensorFusion_Disable();
            }
        } else {
            FAULT_Clear(FAULT_IMU_DISC);
        }

        /* CAN watchdog */
        if ((now - can_tick) > pdMS_TO_TICKS(CAN_TIMEOUT_MS)) {
            if (!FAULT_IsSet(FAULT_CAN_TIMEOUT)) {
                FAULT_Set(FAULT_CAN_TIMEOUT);
                CAN_BusOff_Recovery();
                FaultLog_Write(FAULT_CAN_TIMEOUT, now);
            }
        } else {
            FAULT_Clear(FAULT_CAN_TIMEOUT);
        }

        /* CAN diagnostic frame
         * Send ONLY if fault state changed OR 1 second has passed
         * Prevents flooding bus with 20 frames/sec on persistent fault */
        uint32_t fr            = FAULT_Snapshot();
        uint8_t state_changed  = (fr != prev_fault_register);
        uint8_t interval_elapsed = ((now - last_can_diag_tick) >=
                                     pdMS_TO_TICKS(CAN_DIAG_INTERVAL_MS));

        if (state_changed || interval_elapsed) {
            send_diag_frame(fr);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
