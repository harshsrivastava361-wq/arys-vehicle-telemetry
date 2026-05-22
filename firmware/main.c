/* =============================================================
 * Arys Garage — Real-Time Vehicle Telemetry System
 * File: main.c
 * Target: STM32F4 (simulated with FreeRTOS)
 * Author: Harsh
 * ============================================================= */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <stdint.h>

/* ---- Shared Queues ---- */
QueueHandle_t xGPSQueue;
QueueHandle_t xIMUQueue;
QueueHandle_t xCANQueue;
QueueHandle_t xLogQueue;

/* ---- Shared Mutex ---- */
SemaphoreHandle_t xSDCardMutex;

/* ---- Task Prototypes ---- */
void vTaskSensorAcquisition(void *pvParameters);
void vTaskGPSParsing(void *pvParameters);
void vTaskCANHandler(void *pvParameters);
void vTaskDataLogger(void *pvParameters);
void vTaskWirelessTelemetry(void *pvParameters);
void vTaskFaultMonitor(void *pvParameters);

/* ---- Task Priority Table (higher number = higher priority) ----
 *
 * PRIORITY | TASK                  | RATIONALE
 * ---------|----------------------|----------------------------------
 *    6     | FaultMonitor          | Safety-critical, must preempt all
 *    5     | SensorAcquisition     | Hard real-time, 100Hz sampling
 *    4     | GPSParsing            | Time-sensitive NMEA parsing
 *    3     | CANHandler            | Bus timing constraints
 *    2     | DataLogger            | I/O bound, can tolerate jitter
 *    1     | WirelessTelemetry     | Best-effort, lowest priority
 */

int main(void) {
    /* Hardware Init (abstracted) */
    HAL_Init();
    SystemClock_Config();
    UART_Init();
    SPI_Init();
    I2C_Init();
    CAN_Init();
    ADC_Init();

    /* Create Queues */
    xGPSQueue = xQueueCreate(10, sizeof(GPS_Data_t));
    xIMUQueue = xQueueCreate(20, sizeof(IMU_Data_t));
    xCANQueue = xQueueCreate(15, sizeof(CAN_Frame_t));
    xLogQueue = xQueueCreate(50, sizeof(Log_Entry_t));

    /* Create Mutex */
    xSDCardMutex = xSemaphoreCreateMutex();

    /* Create Tasks */
    xTaskCreate(vTaskSensorAcquisition, "SensorAcq",  512, NULL, 5, NULL);
    xTaskCreate(vTaskGPSParsing,        "GPSParse",   512, NULL, 4, NULL);
    xTaskCreate(vTaskCANHandler,        "CANHandle",  512, NULL, 3, NULL);
    xTaskCreate(vTaskDataLogger,        "DataLog",    1024, NULL, 2, NULL);
    xTaskCreate(vTaskWirelessTelemetry, "Wireless",   512, NULL, 1, NULL);
    xTaskCreate(vTaskFaultMonitor,      "FaultMon",   256, NULL, 6, NULL);

    /* Start Scheduler */
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;);
    return 0;
}
