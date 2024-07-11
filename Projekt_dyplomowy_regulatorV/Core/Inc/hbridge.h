/*
 * hbridge.h
 *
 *  Created on: May 22, 2024
 *      Author: 48690
 */

#ifndef INC_HBRIDGE_H_
#define INC_HBRIDGE_H_

#include "main.h"

typedef enum
{
	SHUTDOWN = 0,
	NORTH = 1,
	SOUTH = 2

} HBRIDGE_STATE;

typedef struct
{
	GPIO_TypeDef *DIR_PORT;

	uint16_t DIR_GPIO_PIN;

	TIM_HandleTypeDef *PWM_TIM;
	uint8_t TIM_CHANNEL;
} HBridge;

void HBridgeInit(HBridge *hb, GPIO_TypeDef *dir_port,
		uint16_t dir_pin, TIM_HandleTypeDef *PwmTim,
		uint8_t TimChannel);


void HBridgeControl(HBridge *hb, int16_t pwm_val);


#endif /* INC_HBRIDGE_H_ */
