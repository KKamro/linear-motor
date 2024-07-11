/*
 * MedianFilter.c
 *
 *  Created on: Jun 10, 2024
 *      Author: 48690
 */


#include <DigitalFilters.h>
#include "string.h"

#include "main.h"
static int32_t partition(int32_t arr[], int32_t low, int32_t high);
//static void bubble_sort(int32_t *array, int32_t size);
static void quickSort(int32_t arr[], int32_t low, int32_t high);
static void swap(int32_t *xp, int32_t *yp);
int32_t get_median(int32_t *array, int32_t size);


// Median Filter
void FilterMedianInit(CircularBuffer *buff)
{
	CircBuffInit(buff);
}

int32_t FilterMedianUpdate(CircularBuffer *buff, int32_t new_data)
{
	int32_t tmp_array[BUFFER_SIZE];
	int32_t out = 0;

	CircBuffUpdate(buff, new_data);

	memcpy(tmp_array, buff->data, sizeof(tmp_array));

	quickSort(tmp_array, 0,BUFFER_SIZE - 1);

	out = get_median(tmp_array, BUFFER_SIZE);

	return out;
}

//static void bubble_sort(int32_t *array, int32_t size)
//{
//	uint32_t i, j;
//
//    for (i = 0; i < (size - 1); i++)
//	{
//        for (j = 0; j < (size - i - 1); j++)
//		{
//            if (array[j] > array[j + 1])
//			{
//                swap(&array[j], &array[j + 1]);
//			}
//		}
//	}
//}

static int32_t partition(int32_t arr[], int32_t low, int32_t high) {
    int32_t pivot = arr[high]; // Wybieramy ostatni element jako pivot
    int32_t i = (low - 1); // Indeks mniejszego elementu

    for (int32_t j = low; j <= high - 1; j++) {
        // Jeśli aktualny element jest mniejszy lub równy pivotowi
        if (arr[j] <= pivot) {
            i++; // Inkrementujemy indeks mniejszego elementu
            swap(&arr[i], &arr[j]);
        }
    }
    swap(&arr[i + 1], &arr[high]);
    return (i + 1);
}

// Główna funkcja sortująca używająca quicksort
static void quickSort(int32_t arr[], int32_t low, int32_t high) {
    if (low < high) {
        // pi to indeks partition
        int32_t pi = partition(arr, low, high);

        // Separatne wywołania quickSort dla przedziałów przed i po partition
        quickSort(arr, low, pi - 1);
        quickSort(arr, pi + 1, high);
    }
}

static void swap(int32_t *xp, int32_t *yp)
{
    int32_t temp = *xp;

    *xp = *yp;
    *yp = temp;
}

int32_t get_median(int32_t *array, int32_t size)
{
	int32_t median;

	if ((size % 2) == 0)
	{
		median = (array[size/2-1] + array[size/2]) / 2;
	}
	else
	{
		median = array[size/2-1];
	}

	return median;
}

// Moving Average Filter
static int32_t average(int32_t *array, uint32_t size);

void FilterMovingAverageInit(CircularBuffer *buff)
{
	CircBuffInit(buff);
}

int32_t FilterMovingAverageUpdate(CircularBuffer *buff, int32_t new_data)
{
   int32_t out = 0;

   CircBuffUpdate(buff, new_data);
   out = average(buff->data, buff->size);

   return out;
}

static int32_t average(int32_t *array, uint32_t size)
{
	int32_t sum = 0;

	for (uint32_t i = 0; i < size; i++)
	{
	  sum += array[i];
	}

	return (sum/size);
}
// Lowpass
void FilterLowPassInit(FilterLowPass *lpf, float alpha)
{
	lpf->out = 0.0f;
	lpf->alpha = alpha;
}

int32_t FilterLowPassUpdate(FilterLowPass *lpf, int32_t new_data)
{
	lpf->out = lpf->alpha * new_data + (1.0f-(lpf->alpha)) * lpf->out;

	return lpf->out;
}
