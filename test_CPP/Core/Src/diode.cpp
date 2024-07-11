/*
 * diode.cpp
 *
 *  Created on: Jul 2, 2024
 *      Author: 48690
 */
#include "diode.hpp"

void Diode::Toggle()
{
	HAL_GPIO_TogglePin(GPIO_PORT, GPIO_PIN);
}
void Diode::On()
{
	HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN, GPIO_PIN_SET);
}
void Diode::Off()
{
	HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN, GPIO_PIN_RESET);
}

GPIO_TypeDef* Diode::GetDiodePort()
{
	return GPIO_PORT;
}
uint16_t Diode::GetDiodePin()
{
	return GPIO_PIN;
}
GPIO_PinState Diode::GetPinState()
{
	PIN_STATE = HAL_GPIO_ReadPin(GPIO_PORT, GPIO_PIN);
	return PIN_STATE;
}

