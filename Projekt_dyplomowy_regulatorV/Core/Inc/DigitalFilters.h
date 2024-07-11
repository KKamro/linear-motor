/*
 * MedianFilter.h
 *
 *  Created on: Jun 10, 2024
 *      Author: 48690
 */

#ifndef INC_DIGITALFILTERS_H_
#define INC_DIGITALFILTERS_H_

#include "main.h"
#include "CircularBuffer.h"

#define LOW_PASS_ALPHA	(0.01f)

void FilterMedianInit(CircularBuffer *buff);
int32_t FilterMedianUpdate(CircularBuffer *buff, int32_t new_data);

void FilterMovingAverageInit(CircularBuffer *buff);
int32_t FilterMovingAverageUpdate(CircularBuffer *buff, int32_t new_data);

typedef struct
{
	float out;
	float alpha;
} FilterLowPass;

void FilterLowPassInit(FilterLowPass *lpf, float alpha);
int32_t FilterLowPassUpdate(FilterLowPass *lpf, int32_t new_data);
#endif /* INC_DIGITALFILTERS_H_ */
