/*
 * ring_buffer.h
 *
 *  Created on: Nov 19, 2023
 *      Author: 48690
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#define RING_BUFFER_SIZE 8

// Success status
typedef enum
{
	RB_OK = 0,
	RB_ERROR = 1

} RB_Status;

// Object Ring Buffer
typedef struct
{
	uint16_t Head;
	uint16_t Tail;

	uint8_t Buffer[RING_BUFFER_SIZE];

}TRingBuffer;
// Functions
// Write
RB_Status RB_Write(TRingBuffer *Buf, uint8_t value);
//Read
RB_Status RB_Read(TRingBuffer *Buf, uint8_t *Value);
// Flush
void RB_Flush(TRingBuffer *Buf);

#endif /* INC_RING_BUFFER_H_ */
