/*
 * wire.h
 *
 *  Created on: Nov 24, 2022
 *      Author: 48690
 */

#pragma once

#include "stm32l4xx.h"

// definitions
#define USART_USED USART3
#define USART_INSTANCE huart3

#define FALSE 0
#define TRUE  1

// method declarations
int OW_Search_First();

int OW_Search_Next();

int OW_Verify();

void OWTargetSetup(uint8_t family_code);

void OWFamilySkipSetup();

HAL_StatusTypeDef OW_Reset();

void OWWriteByte(uint8_t byte_value);

void OWWriteBit(uint8_t bit_value);

uint8_t OWReadBit();

int OW_Search();

uint8_t OW_Crc_LUT(uint8_t value);

HAL_StatusTypeDef OW_Read_Byte(void);

void OW_Write_Byte(uint8_t byte);

uint8_t OW_CRC(const uint8_t* crc, int len);

void OW_Write_Bit(int value);

void OW_Set_Baudrate(uint32_t baudrate);

