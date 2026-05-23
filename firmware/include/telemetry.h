/* ================================================================
 * Arys Garage — Real-Time Vehicle Telemetry System
 * File   : telemetry.h
 * About  : Shared type definitions, fault codes, queue handles.
 *          Every firmware .c file includes this header.
 * ================================================================ */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>

/* ── Fault bitmask codes ──────────────────────────────────────── */
#define FAULT_GPS_LOSS      (1 << 0)
#define FAULT_IMU_DISC      (1 << 1)
#define FAULT_CAN_TIMEOUT   (1 << 2)
#define FAULT_SD_FAIL       (1 << 3)

/* ── Telemetry modes ──────────────────────────────────────────── */
typedef enum {
    TELEM_MODE_NORMAL         = 0,
    TELEM_MODE_DEAD_RECKONING = 1,
    TELEM_MODE_GPS_ONLY       = 2,
} TelemetryMode_t;

/* ── Raw IMU register values (direct from MPU-6050) ──────────── */
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x,  gyro_y,  gyro_z;
} IMU_Raw_t;

/* ── Fused IMU data (after complementary filter) ─────────────── */
typedef struct {
    uint32_t timestamp_ms;
    float    accel_x, accel_y, accel_z;   /* m/s²  */
    float    gyro_x,  gyro_y,  gyro_z;    /* rad/s */
    float    roll, pitch;                  /* deg   */
    float    g_force;                      /* g     */
    float    wheel_rpm;
    float    speed_kmh;
    uint8_t  sensor_ok;
} IMU_Data_t;

/* ── GPS data (parsed from NMEA $GPRMC) ──────────────────────── */
typedef struct {
    uint32_t timestamp_ms;
    double   latitude;       /* decimal degrees */
    double   longitude;      /* decimal degrees */
    float    speed_kmh;
    float    heading;        /* 0–360°          */
    float    utc_time;
    uint8_t  fix_valid;
} GPS_Data_t;

/* ── CAN frame (CAN 2.0A, 11-bit ID) ─────────────────────────── */
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_Frame_t;

/* ── Log entry written to SD card CSV ────────────────────────── */
typedef struct {
    uint32_t timestamp_ms;
    float    speed_kmh;
    float    wheel_rpm;
    float    accel_x, accel_y, accel_z;
    float    roll, pitch, g_force;
    double   latitude, longitude;
    float    heading;
    uint8_t  fault_flags;
} Log_Entry_t;

/* ── Shared FreeRTOS queue handles (defined in main.c) ───────── */
extern QueueHandle_t   xGPSQueue;
extern QueueHandle_t   xIMUQueue;
extern QueueHandle_t   xCANQueue;
extern QueueHandle_t   xLogQueue;
extern SemaphoreHandle_t xSDCardMutex;

/* ── Peripheral port IDs ──────────────────────────────────────── */
#define GPS_UART_PORT       2      /* UART2 — PA2/PA3              */
#define WHEEL_SPEED_ADC_CH  0      /* ADC1 channel 0               */
#define MAX_RPM             8000.0f

/* ── Fault monitor API (defined in fault_task.c) ─────────────── */
void FAULT_Set(uint32_t code);
void FAULT_Clear(uint32_t code);
int  FAULT_IsSet(uint32_t code);
void FAULT_UpdateGPSTick(void);
void FAULT_UpdateIMUTick(void);
void FAULT_UpdateCANTick(void);

/* ── System mode API ──────────────────────────────────────────── */
void Telemetry_SetMode(TelemetryMode_t mode);
void SensorFusion_Disable(void);
void DataLogger_UseRAMBuffer(void);

/* ── Peripheral driver stubs (defined in drivers/) ───────────── */
int     UART_ReadLine(int port, char *buf, int maxlen, uint32_t timeout);
int     UART_Write(int port, const uint8_t *buf, int len);
int     I2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *buf, int len);
int     I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t val);
void    IMU_ReadRaw(IMU_Raw_t *out);
uint16_t ADC_ReadChannel(uint8_t channel);
int     CAN_Transmit(const CAN_Frame_t *frame);
int     CAN_Receive(CAN_Frame_t *frame, uint32_t timeout_ms);
void    CAN_BusOff_Recovery(void);
void    FaultLog_Write(uint32_t fault_code, uint32_t tick);
void    Wireless_Send(const void *data, uint16_t len);
void    Wireless_SetHighPriority(void);
void    LED_SetFaultIndicator(uint8_t indicator);
#define LED_GPS_FAULT  0x01
#define LED_IMU_FAULT  0x02

#endif /* TELEMETRY_H */
