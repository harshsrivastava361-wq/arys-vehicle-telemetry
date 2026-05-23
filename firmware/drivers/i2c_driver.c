/* I2C Driver — MPU-6050 IMU (I2C1, PB6/PB7, 400kHz Fast Mode) */
#include "telemetry.h"

int I2C_ReadReg(uint8_t addr, uint8_t reg, uint8_t *buf, int len) {
    /* HAL_I2C_Mem_Read(&hi2c1, addr<<1, reg, 1, buf, len, 10) */
    (void)addr; (void)reg; (void)buf; (void)len;
    return 0;
}

int I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
    /* HAL_I2C_Mem_Write(&hi2c1, addr<<1, reg, 1, &val, 1, 10) */
    (void)addr; (void)reg; (void)val;
    return 0;
}

void IMU_ReadRaw(IMU_Raw_t *out) {
    /* MPU-6050 burst read: registers 0x3B–0x48 (14 bytes)
     * Accel: 0x3B–0x40   Temp: 0x41–0x42   Gyro: 0x43–0x48 */
    uint8_t buf[14] = {0};
    I2C_ReadReg(0x68, 0x3B, buf, 14);
    out->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    out->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    out->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    out->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);
}
