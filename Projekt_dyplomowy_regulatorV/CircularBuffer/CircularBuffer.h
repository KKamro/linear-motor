/*
 * CircularBuffer.h
 *
 *  Created on: Jun 10, 2024
 *      Author: 48690
 */

#ifndef INC_CIRCULARBUFFER_H_
#define INC_CIRCULARBUFFER_H_

#define BUFFER_SIZE		15

typedef struct
{
	int32_t data[BUFFER_SIZE];
    uint32_t head;
    uint32_t size;
} CircularBuffer;

void CircBuffInit(CircularBuffer *buff);
void CircBuffUpdate(CircularBuffer *buff, int32_t new_data);

#endif /* INC_CIRCULARBUFFER_H_ */
