/*
 * serial.hpp
 *
 *  Created on: Jul 2, 2024
 *      Author: 48690
 */

#ifndef INC_SERIAL_HPP_
#define INC_SERIAL_HPP_

#include "main.h"
#include "usart.h"
#include "string.h"

class UART
{
private:
	UART_HandleTypeDef* UART_ID;
	uint32_t TIMEOUT;
public:
	UART() = delete;
	UART(UART_HandleTypeDef* huart, uint32_t timeout = HAL_MAX_DELAY) : UART_ID(huart), TIMEOUT(timeout) {}
	void print(const char *txt);
	void println(const char *txt);
};

#endif /* INC_SERIAL_HPP_ */
