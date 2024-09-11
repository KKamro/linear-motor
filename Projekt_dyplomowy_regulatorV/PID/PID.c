/*
 * PID.c
 *
 *  Created on: Jun 3, 2024
 *      Author: 48690
 */

#include "PID.h"

void PIDInit(PID *pid_data, float kp_init, float ki_init, float kd_init, int anti_windup_limit_init)
{
	pid_data->error_previous = 0;
	pid_data->error_total = 0;

	pid_data->Kp = kp_init;
	pid_data->Ki = ki_init;
	pid_data->Kd = kd_init;

	pid_data->anti_windup_limit = anti_windup_limit_init;
}

void PIDReset(PID *pid_data)
{
	pid_data->error_total = 0;
	pid_data->error_previous = 0;
}

float PIDCalculate(PID *pid_data, float setpoint, float process_variable)
{
	float error;
	float p_term, i_term, d_term;

	error = setpoint - process_variable;		//obliczenie uchybu
	pid_data->error_total += error;			//sumowanie uchybu

	p_term = (float)(pid_data->Kp * error);		//odpowiedź członu proporcjonalnego
	i_term = (float)(pid_data->Ki * pid_data->error_total);	//odpowiedź członu całkującego
	d_term = (float)(pid_data->Kd * (error - pid_data->error_previous));//odpowiedź członu różniczkującego

	if(i_term >= pid_data->anti_windup_limit) i_term = pid_data->anti_windup_limit;	//Anti-Windup - ograniczenie odpowiedzi członu całkującego
	else if(i_term <= -pid_data->anti_windup_limit) i_term = -pid_data->anti_windup_limit;

	pid_data->error_previous = error;	//aktualizacja zmiennej z poprzednią wartością błędu

	return (int)(p_term + i_term + d_term);		//odpowiedź regulatora
}


