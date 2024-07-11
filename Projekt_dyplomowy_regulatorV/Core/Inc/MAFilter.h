/*
 * MAFilter.h
 *
 *  Created on: Jun 7, 2024
 *      Author: 48690
 */

#ifndef INC_MAFILTER_H_
#define INC_MAFILTER_H_

#include "main.h"

#define FILTER_SIZE 20

typedef struct
{
	uint8_t filter_size;
	float buffer[FILTER_SIZE];
	uint8_t buffer_index;
	float sum;
	uint8_t count;

} MAFilter;

void init_filter(MAFilter *maf);


int32_t update_filter(MAFilter *maf, int32_t new_sample);

#endif /* INC_MAFILTER_H_ */
