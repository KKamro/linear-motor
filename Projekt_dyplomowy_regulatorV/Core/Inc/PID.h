/*
 * PID.h
 *
 *  Created on: Jun 3, 2024
 *      Author: 48690
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32l4xx.h"

typedef struct
{
	float error_previous;
	float error_total;

	float Kp;
	float Ki;
	float Kd;

	int16_t anti_windup_limit;
}PID;

void PIDInit(PID *pid_data, float kp_init, float ki_init, float kd_init, int anti_windup_limit_init);
void PIDReset(PID *pid_data);
float PIDCalculate(PID *pid_data, float setpoint, float process_variable);

#endif /* INC_PID_H_ */
