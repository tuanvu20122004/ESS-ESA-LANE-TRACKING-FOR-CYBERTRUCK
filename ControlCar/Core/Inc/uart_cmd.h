#ifndef UART_CMD_H
#define UART_CMD_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void UART_CommandHandler(uint8_t *rx_buffer);

#endif
