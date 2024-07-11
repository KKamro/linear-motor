/*
 * serial.cpp
 *
 *  Created on: Jul 2, 2024
 *      Author: 48690
 */

#include "serial.hpp"

void UART::print(const char* txt)
{
	HAL_UART_Transmit_DMA(UART_ID, (uint8_t*)txt, strlen(txt));
}
void UART::println(const char *txt)
{
	uint8_t len = strlen(txt);
	static uint8_t buff[20];
	memcpy(buff, txt, len);
	buff[len] = '\r';
	buff[len + 1] = '\n';
	HAL_UART_Transmit_DMA(UART_ID, buff, (len+2));
}
