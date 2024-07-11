/*
 * MAFilter.c
 *
 *  Created on: Jun 7, 2024
 *      Author: 48690
 */

#include "MAFilter.h"

void init_filter(MAFilter *maf)
{
	for (int i = 0; i < FILTER_SIZE; i++)
	{
		maf->buffer[i] = 0;
	}
	maf->buffer_index = 0;
	maf->sum = 0;
	maf->count = 0;
}

int32_t update_filter(MAFilter *maf, int32_t new_sample)
{
	// Odejmij starą próbkę z sumy
	maf->sum -= maf->buffer[maf->buffer_index];

	// Dodaj nową próbkę do bufora i do sumy
	maf->buffer[maf->buffer_index] = new_sample;
	maf->sum += new_sample;

	// Przesuń indeks bufora
	maf->buffer_index = (maf->buffer_index + 1) % FILTER_SIZE;

	// Zwiększ licznik próbek (do maksymalnej wartości FILTER_SIZE)
	if (maf->count < FILTER_SIZE)
	{
		maf->count++;
	}

	// Zwróć uśrednioną wartość
	return (maf->sum) / (maf->count);
}
