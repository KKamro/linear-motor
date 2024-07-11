/*
 * ring_buffer.c
 *
 *  Created on: Nov 19, 2023
 *      Author: 48690
 */

#include "main.h"
#include "ring_buffer.h"

RB_Status RB_Read(TRingBuffer *Buf, uint8_t *Value)
{
	if(Buf->Head == Buf->Tail)
	{
		return RB_ERROR;
	}

	*Value = Buf->Buffer[Buf->Tail];

	Buf->Tail = (Buf->Tail + 1) % RING_BUFFER_SIZE;

	return RB_OK;
}

RB_Status RB_Write(TRingBuffer *Buf, uint8_t Value)
{
	uint8_t HeadTmp = (Buf->Head + 1) % RING_BUFFER_SIZE; // Zabezpieczenie przed wyjsciem poza rozmiar tablicy

	if(HeadTmp == Buf->Tail)
	{
		return RB_ERROR;
	}

	Buf->Buffer[Buf->Head] = Value;

	Buf->Head = HeadTmp;

	return RB_OK;
}

void RB_Flush(TRingBuffer *Buf)
{
	Buf->Head = 0;
	Buf->Tail = 0;
}

