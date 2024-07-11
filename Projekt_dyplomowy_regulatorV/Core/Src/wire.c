/*
 * wire.c
 *
 *  Created on: Nov 24, 2022
 *      Author: 48690
 */
#include "wire.h"
#include "gpio.h"
#include "usart.h"

// global search state
uint8_t ROM_NO[8];
int LastDiscrepancy;
int LastFamilyDiscrepancy;
int LastDeviceFlag;
uint8_t crc8;



void OW_Set_Baudrate(uint32_t baudrate)
{
	 USART_INSTANCE.Instance = USART_USED;
	 USART_INSTANCE.Init.BaudRate = baudrate;
	 USART_INSTANCE.Init.WordLength = UART_WORDLENGTH_8B;
	 USART_INSTANCE.Init.StopBits = UART_STOPBITS_1;
	 USART_INSTANCE.Init.Parity = UART_PARITY_NONE;
	 USART_INSTANCE.Init.Mode = UART_MODE_TX_RX;
	 USART_INSTANCE.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	 USART_INSTANCE.Init.OverSampling = UART_OVERSAMPLING_16;
	 USART_INSTANCE.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	 USART_INSTANCE.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	 USART_INSTANCE.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;

	 if (HAL_HalfDuplex_Init(&USART_INSTANCE) != HAL_OK)
	 {
	   Error_Handler();
	 }
}

HAL_StatusTypeDef OW_Reset(void)
{
	  uint8_t data_out = 0xF0;
	  uint8_t data_in = 0;

	  OW_Set_Baudrate(9600);
	  HAL_UART_Transmit(&USART_INSTANCE, &data_out, 1, HAL_MAX_DELAY);
	  HAL_UART_Receive(&USART_INSTANCE, &data_in, 1, HAL_MAX_DELAY);
	  OW_Set_Baudrate(115200);

	  if (data_in != 0xF0)
	    return HAL_OK;
	  else
	    return HAL_ERROR;
}

int OW_Read_Bit(void)
{
	uint8_t data_out = 0xFF;
	uint8_t data_in = 0;
	HAL_UART_Transmit(&USART_INSTANCE, &data_out, 1, HAL_MAX_DELAY);
	HAL_UART_Receive(&USART_INSTANCE, &data_in, 1, HAL_MAX_DELAY);

	return data_in & 0x01;
}

uint8_t OW_Read_Byte(void)
{
  uint8_t value = 0;
  int i;
  for (i = 0; i < 8; i++) {
    value >>= 1;
    if (OW_Read_Bit())
      value |= 0x80;
  }
  return value;
}

void OW_Write_Bit(int value)
{
	if (value) {
	      uint8_t data_out = 0xff;
	    HAL_UART_Transmit(&USART_INSTANCE, &data_out, 1, HAL_MAX_DELAY);
	  } else {
	      uint8_t data_out = 0x0;
	    HAL_UART_Transmit(&USART_INSTANCE, &data_out, 1, HAL_MAX_DELAY);
	  }
}

void OW_Write_Byte(uint8_t byte)
{
  int i;
  for (i = 0; i < 8; i++) {
    OW_Write_Bit(byte & 0x01);
    byte >>= 1;
  }
}

static uint8_t OW_CRC_BYTE(uint8_t crc, uint8_t byte)
{
  int i;
  for (i = 0; i < 8; i++) {
    uint8_t b = crc ^ byte;
    crc >>= 1;
    if (b & 0x01)
      crc ^= 0x8c;
    byte >>= 1;
  }
  return crc;
}

uint8_t OW_CRC(const uint8_t* data, int len)
{
  int i;
    uint8_t crc = 0;

    for (i = 0; i < len; i++)
      crc = OW_CRC_BYTE(crc, data[i]);

    return crc;
}


//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire bus
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : no device present
//
int OW_Search_First() {
	// reset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = FALSE;
	LastFamilyDiscrepancy = 0;

	return OW_Search();
}

//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire bus
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
int OW_Search_Next() {
	// leave the search state alone
	return OW_Search();
}

//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
int OW_Search() {
	int id_bit_number;
	int last_zero, rom_byte_number, search_result;
	int id_bit, cmp_id_bit;
	uint8_t rom_byte_mask, search_direction;

	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = 0;
	crc8 = 0;

	// if the last call was not the last one
	if (!LastDeviceFlag) {
		// 1-Wire reset
		if (OW_Reset() != HAL_OK) {
			// reset the search
			LastDiscrepancy = 0;
			LastDeviceFlag = FALSE;
			LastFamilyDiscrepancy = 0;
			return FALSE;
		}

		// issue the search command
		OW_Write_Byte(0xF0);

		// loop to do the search
		do {
			// read a bit and its complement
			id_bit = OW_Read_Bit();
			cmp_id_bit = OW_Read_Bit();

			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1))
				break;
			else {
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit)
					search_direction = id_bit;  // bit write value for search
				else {
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < LastDiscrepancy)
						search_direction = ((ROM_NO[rom_byte_number]
								& rom_byte_mask) > 0);
					else
						// if equal to last pick 1, if not then pick 0
						search_direction = (id_bit_number == LastDiscrepancy);

					// if 0 was picked then record its position in LastZero
					if (search_direction == 0) {
						last_zero = id_bit_number;

						// check for Last discrepancy in family
						if (last_zero < 9)
							LastFamilyDiscrepancy = last_zero;
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1)
					ROM_NO[rom_byte_number] |= rom_byte_mask;
				else
					ROM_NO[rom_byte_number] &= ~rom_byte_mask;

				// serial number search direction write bit
				OW_Write_Bit(search_direction);

				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (rom_byte_mask == 0) {
					OW_Crc_LUT(ROM_NO[rom_byte_number]);  // accumulate the CRC
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		} while (rom_byte_number < 8);  // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!((id_bit_number < 65) || (crc8 != 0))) {
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			LastDiscrepancy = last_zero;

			// check for last device
			if (LastDiscrepancy == 0)
				LastDeviceFlag = TRUE;

			search_result = TRUE;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || !ROM_NO[0]) {
		LastDiscrepancy = 0;
		LastDeviceFlag = FALSE;
		LastFamilyDiscrepancy = 0;
		search_result = FALSE;
	}

	return search_result;
}

//--------------------------------------------------------------------------
// Verify the device with the ROM number in ROM_NO buffer is present.
// Return TRUE  : device verified present
//        FALSE : device not present
//
int OW_Verify() {
	uint8_t rom_backup[8];
	int i, rslt, ld_backup, ldf_backup, lfd_backup;

	// keep a backup copy of the current state
	for (i = 0; i < 8; i++)
		rom_backup[i] = ROM_NO[i];
	ld_backup = LastDiscrepancy;
	ldf_backup = LastDeviceFlag;
	lfd_backup = LastFamilyDiscrepancy;

	// set search to find the same device
	LastDiscrepancy = 64;
	LastDeviceFlag = FALSE;

	if (OW_Search()) {
		// check if same device found
		rslt = TRUE;
		for (i = 0; i < 8; i++) {
			if (rom_backup[i] != ROM_NO[i]) {
				rslt = FALSE;
				break;
			}
		}
	} else
		rslt = FALSE;

	// restore the search state
	for (i = 0; i < 8; i++)
		ROM_NO[i] = rom_backup[i];
	LastDiscrepancy = ld_backup;
	LastDeviceFlag = ldf_backup;
	LastFamilyDiscrepancy = lfd_backup;

	// return the result of the verify
	return rslt;
}

//--------------------------------------------------------------------------
// Setup the search to find the device type 'family_code' on the next call
// to OW_SEARCH_NEXT() if it is present.
//
void OW_Target_Setup(uint8_t family_code) {
	int i;

	// set the search state to find SearchFamily type devices
	ROM_NO[0] = family_code;
	for (i = 1; i < 8; i++)
		ROM_NO[i] = 0;
	LastDiscrepancy = 64;
	LastFamilyDiscrepancy = 0;
	LastDeviceFlag = FALSE;
}

//--------------------------------------------------------------------------
// Setup the search to skip the current device type on the next call
// to OW_SEARCH_NEXT().
//
void OW_Family_SkipSetup() {
	// set the Last discrepancy to last family discrepancy
	LastDiscrepancy = LastFamilyDiscrepancy;
	LastFamilyDiscrepancy = 0;

	// check for end of list
	if (LastDiscrepancy == 0)
		LastDeviceFlag = TRUE;
}


// TEST BUILD
static uint8_t dscrc_table[] = { 0, 94, 188, 226, 97, 63, 221, 131, 194,
		156, 126, 32, 163, 253, 31, 65, 157, 195, 33, 127, 252, 162, 64, 30, 95,
		1, 227, 189, 62, 96, 130, 220, 35, 125, 159, 193, 66, 28, 254, 160, 225,
		191, 93, 3, 128, 222, 60, 98, 190, 224, 2, 92, 223, 129, 99, 61, 124,
		34, 192, 158, 29, 67, 161, 255, 70, 24, 250, 164, 39, 121, 155, 197,
		132, 218, 56, 102, 229, 187, 89, 7, 219, 133, 103, 57, 186, 228, 6, 88,
		25, 71, 165, 251, 120, 38, 196, 154, 101, 59, 217, 135, 4, 90, 184, 230,
		167, 249, 27, 69, 198, 152, 122, 36, 248, 166, 68, 26, 153, 199, 37,
		123, 58, 100, 134, 216, 91, 5, 231, 185, 140, 210, 48, 110, 237, 179,
		81, 15, 78, 16, 242, 172, 47, 113, 147, 205, 17, 79, 173, 243, 112, 46,
		204, 146, 211, 141, 111, 49, 178, 236, 14, 80, 175, 241, 19, 77, 206,
		144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238, 50, 108, 142, 208,
		83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115, 202, 148, 118,
		40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 87, 9, 235,
		181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22, 233, 183,
		85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168, 116, 42,
		200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53 };

//--------------------------------------------------------------------------
// Calculate the CRC8 of the byte value provided with the current
// global 'crc8' value.
// Returns current global crc8 value
//
uint8_t OW_Crc_LUT(uint8_t value) {
	// See Application Note 27

	// TEST BUILD
	crc8 = dscrc_table[crc8 ^ value];
	return crc8;
}
