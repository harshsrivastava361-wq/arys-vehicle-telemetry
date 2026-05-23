/* UART Driver — GPS (UART2 PA2/PA3) and Debug (UART1) */
#include "telemetry.h"
#include <string.h>

int UART_ReadLine(int port, char *buf, int maxlen, uint32_t timeout_ticks) {
    /* On real hardware: HAL_UART_Receive with timeout.
     * Reads until \n or maxlen-1 chars. Returns bytes read or -1 on timeout. */
    (void)port; (void)buf; (void)maxlen; (void)timeout_ticks;
    return -1; /* stub — replace with HAL_UART_Receive */
}

int UART_Write(int port, const uint8_t *buf, int len) {
    /* On real hardware: HAL_UART_Transmit */
    (void)port; (void)buf; (void)len;
    return 0;
}
