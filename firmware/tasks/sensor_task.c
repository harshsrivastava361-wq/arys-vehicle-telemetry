/* =============================================================
 * Task: Sensor Acquisition
 * Rate: 100 Hz (10ms period)
 * Sensors: IMU (MPU6050 via I2C), Wheel Speed (ADC)
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "sensor_task.h"
#include "imu_driver.h"
#include "fault_task.h" /* Fix 1: Required to pet the watchdog */
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
        /* --- Fix 2: I2C Noise Protection ---
         * Only process data and pet watchdog if the physical read succeeds.
         * (Assuming IMU_ReadRaw returns 1 or HAL_OK on success) */
        if (IMU_ReadRaw(&raw) == 1) { 
            
            /* --- Fix 1: Pet the Watchdog --- */
            FAULT_UpdateIMUTick();

            /* --- Convert ADC counts to physical units --- */
            float ax = raw.accel_x * 9.81f / 16384.0f;  /* m/s² */
            float ay = raw.accel_y * 9.81f / 16384.0f;
            float az = raw.accel_z * 9.81f / 16384.0f;
            
            /* Fix 3: Float Optimization. Calculate directly in deg/s for the filter */
            float gx_deg = raw.gyro_x / 131.0f;  
            float gy_deg = raw.gyro_y / 131.0f;  
            float gz_deg = raw.gyro_z / 131.0f;  

            /* --- Complementary Filter for Roll & Pitch --- */
            float dt = TASK_PERIOD_MS / 1000.0f;
            float accel_roll  = atan2f(ay, az) * (180.0f / M_PI);
            float accel_pitch = atan2f(-ax, az) * (180.0f / M_PI);

            /* No redundant rad-to-deg conversions needed here anymore */
            roll_angle  = ALPHA * (roll_angle  + gx_deg * dt) + (1.0f - ALPHA) * accel_roll;
            pitch_angle = ALPHA * (pitch_angle + gy_deg * dt) + (1.0f - ALPHA) * accel_pitch;

            /* --- G-Force magnitude --- */
            float g_force = sqrtf(ax*ax + ay*ay + az*az) / 9.81f;

            /* --- Wheel Speed via ADC --- */
            uint16_t adc_val = ADC_ReadChannel(WHEEL_SPEED_ADC_CH);
            float wheel_rpm  = (adc_val / 4095.0f) * MAX_RPM;
            float vehicle_speed_kmh = (wheel_rpm * M_PI * 0.56f * 60.0f) / 1000.0f;

            /* --- Pack fused data --- */
            fused.timestamp_ms  = xTaskGetTickCount();
            fused.accel_x       = ax;
            fused.accel_y       = ay;
            fused.accel_z       = az;
            
            /* Calculate radians just once for the final struct export */
            fused.gyro_x        = gx_deg * (M_PI / 180.0f);
            fused.gyro_y        = gy_deg * (M_PI / 180.0f);
            fused.gyro_z        = gz_deg * (M_PI / 180.0f);
            
            fused.roll          = roll_angle;
            fused.pitch         = pitch_angle;
            fused.g_force       = g_force;
            fused.wheel_rpm     = wheel_rpm;
            fused.speed_kmh     = vehicle_speed_kmh;
            fused.sensor_ok     = 1;

            /* --- Send to logging queue --- */
            xQueueSend(xIMUQueue, &fused, 0);
        }
        /* If IMU_ReadRaw fails, we do NOT pet the watchdog and we skip the queue.
         * The Priority 6 Fault task will automatically detect the silence and handle it. */

        /* --- Precise 100Hz timing --- */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}
