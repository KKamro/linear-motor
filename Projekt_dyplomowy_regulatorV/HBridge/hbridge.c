/*
 * hbridge.c
 *
 *  Created on: May 22, 2024
 *      Author: 48690
 */
#include "hbridge.h"


void HBridgeInit(HBridge *hb, GPIO_TypeDef *dir_port,
		uint16_t dir_pin, TIM_HandleTypeDef *PwmTim,
		uint8_t TimChannel)
{
	hb->DIR_PORT = dir_port;

	hb->DIR_GPIO_PIN = dir_pin;

	hb->PWM_TIM = PwmTim;
	hb->TIM_CHANNEL = TimChannel;
}

void HBridgeControl(HBridge *hb, int16_t pwm_val)
{
	if(pwm_val > 0)
	{
		__HAL_TIM_SET_COMPARE(hb->PWM_TIM, hb->TIM_CHANNEL, 0);
		HAL_GPIO_WritePin(hb->DIR_PORT,hb->DIR_GPIO_PIN, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(hb->PWM_TIM, hb->TIM_CHANNEL, pwm_val);
	}
	else if(pwm_val < 0)
	{
		pwm_val *= -1;
		__HAL_TIM_SET_COMPARE(hb->PWM_TIM, hb->TIM_CHANNEL, 0);
		HAL_GPIO_WritePin(hb->DIR_PORT,hb->DIR_GPIO_PIN, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(hb->PWM_TIM, hb->TIM_CHANNEL, pwm_val);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(hb->PWM_TIM, hb->TIM_CHANNEL, 0);
	}
}


