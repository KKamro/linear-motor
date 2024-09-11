/*
 * CircularBuffer.c
 *
 *  Created on: Jun 10, 2024
 *      Author: 48690
 */

#include "main.h"
#include "CircularBuffer.h"

void CircBuffInit(CircularBuffer *buff)
{
	buff->head = 0;
	buff->size = BUFFER_SIZE;

	for (uint32_t i=0; i < buff->size; i++)
	{
		buff->data[i] = 0;
	}
}

void CircBuffUpdate(CircularBuffer *buff, int32_t new_data)
{
	buff->data[buff->head] = new_data;

	buff->head++;
	buff->head = buff->head % buff->size;
}


