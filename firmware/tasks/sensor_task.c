/* =============================================================
 * Task: Sensor Acquisition
 * Rate: 100 Hz (10ms period)
 * Sensors: IMU (MPU6050 via I2C), Wheel Speed (ADC), Temp (ADC)
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "sensor_task.h"
#include "imu_driver.h"
#include <math.h>

/* Complementary Filter coefficient (alpha) */
#define ALPHA           0.98f
#define TASK_PERIOD_MS  10

/* Persistent filter state */
static float roll_angle  = 0.0f;
static float pitch_angle = 0.0f;

void vTaskSensorAcquisition(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    IMU_Raw_t  raw;
    IMU_Data_t fused;

    for (;;) {
        /* --- Read raw IMU via I2C --- */
        IMU_ReadRaw(&raw);  /* MPU6050 driver call */

        /* --- Convert ADC counts to physical units --- */
        float ax = raw.accel_x * 9.81f / 16384.0f;  /* ±2g range */
        float ay = raw.accel_y * 9.81f / 16384.0f;
        float az = raw.accel_z * 9.81f / 16384.0f;
        float gx = raw.gyro_x * (M_PI / 180.0f) / 131.0f;  /* ±250°/s */
        float gy = raw.gyro_y * (M_PI / 180.0f) / 131.0f;
        float gz = raw.gyro_z * (M_PI / 180.0f) / 131.0f;

        /* --- Complementary Filter for Roll & Pitch ---
         * Fuses gyroscope (good short-term) with accelerometer (good long-term)
         * roll  = α*(roll  + gx*dt) + (1-α)*atan2(ay, az)
         * pitch = α*(pitch + gy*dt) + (1-α)*atan2(-ax, az)
         */
        float dt = TASK_PERIOD_MS / 1000.0f;
        float accel_roll  = atan2f(ay, az) * (180.0f / M_PI);
        float accel_pitch = atan2f(-ax, az) * (180.0f / M_PI);

        roll_angle  = ALPHA * (roll_angle  + gx * dt * (180.0f / M_PI))
                    + (1.0f - ALPHA) * accel_roll;
        pitch_angle = ALPHA * (pitch_angle + gy * dt * (180.0f / M_PI))
                    + (1.0f - ALPHA) * accel_pitch;

        /* --- G-Force magnitude --- */
        float g_force = sqrtf(ax*ax + ay*ay + az*az) / 9.81f;

        /* --- Wheel Speed via ADC --- */
        uint16_t adc_val = ADC_ReadChannel(WHEEL_SPEED_ADC_CH);
        float wheel_rpm  = (adc_val / 4095.0f) * MAX_RPM;
        /* Wheel circumference = π * diameter (0.56m for typical tyre) */
        float vehicle_speed_kmh = (wheel_rpm * M_PI * 0.56f * 60.0f) / 1000.0f;

        /* --- Pack fused data --- */
        fused.timestamp_ms  = xTaskGetTickCount();
        fused.accel_x       = ax;
        fused.accel_y       = ay;
        fused.accel_z       = az;
        fused.gyro_x        = gx;
        fused.gyro_y        = gy;
        fused.gyro_z        = gz;
        fused.roll          = roll_angle;
        fused.pitch         = pitch_angle;
        fused.g_force       = g_force;
        fused.wheel_rpm     = wheel_rpm;
        fused.speed_kmh     = vehicle_speed_kmh;
        fused.sensor_ok     = 1;

        /* --- Send to logging queue (non-blocking) --- */
        xQueueSend(xIMUQueue, &fused, 0);

        /* --- Precise 100Hz timing --- */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}
