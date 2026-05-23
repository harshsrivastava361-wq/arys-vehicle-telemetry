/* SPI Driver — SD Card (SPI1, PA4–PA7, up to 25MHz) */
#include "telemetry.h"

/* SD card uses FatFS — spi_driver provides the low-level
 * disk_read / disk_write callbacks required by ff.c.
 * Init: 400kHz for card identification, then 25MHz for data. */

void SPI_SD_Init(void) {
    /* HAL_SPI_Init(&hspi1) — configured in CubeMX */
}

uint8_t SPI_SD_ReadByte(void) {
    uint8_t rx = 0xFF;
    /* HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 10) */
    return rx;
}

void SPI_SD_WriteByte(uint8_t byte) {
    /* HAL_SPI_Transmit(&hspi1, &byte, 1, 10) */
    (void)byte;
}

void SPI_SD_CS_Low(void)  { /* HAL_GPIO_WritePin(SD_CS_GPIO, SD_CS_PIN, 0) */ }
void SPI_SD_CS_High(void) { /* HAL_GPIO_WritePin(SD_CS_GPIO, SD_CS_PIN, 1) */ }
