/* ================================================================
 * Arys Garage — Real-Time Vehicle Telemetry System
 * File     : can_task.c
 * Priority : 3  (above logger, below GPS and sensor)
 * Rate     : Per CAN frame (interrupt-driven on hardware)
 *
 * Responsibilities:
 *   TX — broadcasts telemetry frames onto the vehicle CAN bus
 *   RX — receives ECU messages (BMS, VCU, MCU status)
 *   Watchdog — pets fault_task watchdog on every valid RX frame
 *
 * CAN Message IDs used:
 *   0x100  TELEMETRY_GPS    tx  lat/lon/speed/heading
 *   0x101  TELEMETRY_IMU    tx  ax/ay/az packed int16
 *   0x110  BMS_STATUS       rx  SOC, voltage, temperature
 *   0x120  VCU_COMMAND      rx  throttle → torque request
 *   0x130  MCU_STATUS       rx  actual speed, motor temp
 *   0x7FF  FAULT_DIAG       tx  fault register bitmask
 * ================================================================ */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "telemetry.h"
#include <string.h>

/* TX rate limits */
#define CAN_TX_GPS_MS    1000   /* 1 Hz  */
#define CAN_TX_IMU_MS     100   /* 10 Hz */
#define CAN_RX_TIMEOUT_MS 500   /* watchdog: 500ms no frame = fault */

/* ── Pack GPS data into 8-byte CAN frame ─────────────────────── */
static void build_gps_frame(CAN_Frame_t *f,
                             double lat, double lon,
                             float speed, float heading)
{
    /* lat/lon: delta from base ×10000 → int16 (±3.2768°)
     * speed ×10 → uint16  heading ×10 → uint16           */
    int16_t  dlat = (int16_t)((lat  - 12.9716) * 10000);
    int16_t  dlon = (int16_t)((lon  - 77.5946) * 10000);
    uint16_t spd  = (uint16_t)(speed   * 10);
    uint16_t hdg  = (uint16_t)(heading * 10);

    f->id      = 0x100;
    f->dlc     = 8;
    f->data[0] = (dlat >> 8) & 0xFF;
    f->data[1] =  dlat       & 0xFF;
    f->data[2] = (dlon >> 8) & 0xFF;
    f->data[3] =  dlon       & 0xFF;
    f->data[4] = (spd  >> 8) & 0xFF;
    f->data[5] =  spd        & 0xFF;
    f->data[6] = (hdg  >> 8) & 0xFF;
    f->data[7] =  hdg        & 0xFF;
}

/* ── Pack IMU accelerometer into 6-byte CAN frame ────────────── */
static void build_imu_frame(CAN_Frame_t *f,
                             float ax, float ay, float az)
{
    /* Each axis: int16 ×100 (range ±327.67 m/s²) */
    int16_t ix = (int16_t)(ax * 100);
    int16_t iy = (int16_t)(ay * 100);
    int16_t iz = (int16_t)(az * 100);

    f->id      = 0x101;
    f->dlc     = 6;
    f->data[0] = (ix >> 8) & 0xFF;
    f->data[1] =  ix       & 0xFF;
    f->data[2] = (iy >> 8) & 0xFF;
    f->data[3] =  iy       & 0xFF;
    f->data[4] = (iz >> 8) & 0xFF;
    f->data[5] =  iz       & 0xFF;
}

void vTaskCANHandler(void *pvParameters)
{
    CAN_Frame_t  tx_frame;
    CAN_Frame_t  rx_frame;
    IMU_Data_t   imu;
    GPS_Data_t   gps;

    TickType_t last_gps_tx = 0;
    TickType_t last_imu_tx = 0;

    /* Snapshot of latest sensor data for TX */
    float  cur_lat = 12.9716f, cur_lon = 77.5946f;
    float  cur_speed = 0, cur_heading = 0;
    float  cur_ax = 0, cur_ay = 0, cur_az = 9.81f;

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        /* ── Drain IMU queue for latest values ───────────────── */
        while (xQueueReceive(xIMUQueue, &imu, 0) == pdTRUE) {
            cur_ax    = imu.accel_x;
            cur_ay    = imu.accel_y;
            cur_az    = imu.accel_z;
            cur_speed = imu.speed_kmh;
            FAULT_UpdateIMUTick();
        }

        /* ── Drain GPS queue for latest position ─────────────── */
        while (xQueueReceive(xGPSQueue, &gps, 0) == pdTRUE) {
            if (gps.fix_valid) {
                cur_lat     = (float)gps.latitude;
                cur_lon     = (float)gps.longitude;
                cur_heading = gps.heading;
            }
        }

        /* ── TX: GPS frame @ 1 Hz ─────────────────────────────── */
        if ((now - last_gps_tx) >= pdMS_TO_TICKS(CAN_TX_GPS_MS)) {
            build_gps_frame(&tx_frame,
                            cur_lat, cur_lon,
                            cur_speed, cur_heading);
            CAN_Transmit(&tx_frame);
            last_gps_tx = now;
        }

        /* ── TX: IMU frame @ 10 Hz ───────────────────────────── */
        if ((now - last_imu_tx) >= pdMS_TO_TICKS(CAN_TX_IMU_MS)) {
            build_imu_frame(&tx_frame, cur_ax, cur_ay, cur_az);
            CAN_Transmit(&tx_frame);
            last_imu_tx = now;
        }

        /* ── RX: receive one frame (non-blocking) ────────────── */
        if (CAN_Receive(&rx_frame, 0) == 0) {
            /* Got a frame — pet the CAN watchdog */
            FAULT_UpdateCANTick();

            switch (rx_frame.id) {
                case 0x110:  /* BMS_STATUS  */
                    /* SOC = data[0], voltage = data[1..2] ×0.1V */
                    break;
                case 0x120:  /* VCU_COMMAND */
                    /* Throttle = data[0] (0–255 = 0–100%) */
                    break;
                case 0x130:  /* MCU_STATUS  */
                    /* Motor speed = data[0..1] RPM */
                    break;
                default:
                    break;
            }
        }

        /* 10ms sleep — CAN watchdog in fault_task handles timeout */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
