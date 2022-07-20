#ifndef __UART_H
#define __UART_H

#include "stm32f4xx_hal.h"

static void MX_USART2_UART_Init(void);
void printWelcomeMessage(void);
uint8_t readUserInput(void);
uint8_t processUserInput(uint8_t);

#endif //__UART_H