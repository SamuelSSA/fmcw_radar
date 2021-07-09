/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_WRAPPER_H
#define __UART_WRAPPER_H

#include "usart.h"

void uart_send(char *data, uint32_t size)
{
	HAL_UART_Transmit(&huart3, "ab", 3, 100);
}

#endif /* __UART_WRAPPER */
