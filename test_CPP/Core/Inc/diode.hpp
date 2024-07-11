/*
 * diode.hpp
 *
 *  Created on: Jul 2, 2024
 *      Author: 48690
 */

#ifndef INC_DIODE_HPP_
#define INC_DIODE_HPP_

#include "main.h"

class Diode
{
private:
	GPIO_TypeDef* GPIO_PORT;
	uint16_t GPIO_PIN;
	GPIO_PinState PIN_STATE;

public:
	Diode() = delete;
	Diode(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, GPIO_PinState pin_state = GPIO_PIN_RESET) :
		GPIO_PORT(gpio_port), GPIO_PIN(gpio_pin), PIN_STATE(pin_state) {}

	void Toggle();
	void On();
	void Off();
	GPIO_TypeDef* GetDiodePort();
	uint16_t GetDiodePin();
	GPIO_PinState GetPinState();

};

#endif /* INC_DIODE_HPP_ */
