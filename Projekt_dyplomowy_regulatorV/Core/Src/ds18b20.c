/*
 * ds18b20.c
 *
 *  Created on: Nov 24, 2022
 *      Author: 48690
 */

#include <string.h>
#include "ds18b20.h"
#include "wire.h"

#define DS18B20_SCRATCHPAD_SIZE    9

#define DS18B20_READ_ROM           0x33
#define DS18B20_MATCH_ROM          0x55
#define DS18B20_SKIP_ROM           0xCC

#define DS18B20_CONVERT_T          0x44
#define DS18B20_READ_SCRATCHPAD    0xBE


HAL_StatusTypeDef ds18b20_read_address(uint8_t* rom_code)
{
	int i;
	uint8_t crc;

	if(OW_Reset() != HAL_OK)
		return HAL_ERROR;

	OW_Write_Byte(DS18B20_READ_ROM);

	for(i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
		rom_code[i] = OW_Read_Byte();

	crc = OW_CRC(rom_code, DS18B20_ROM_CODE_SIZE - 1);
	if(rom_code[DS18B20_ROM_CODE_SIZE -1] == crc)
		return HAL_OK;
	else
		return HAL_ERROR;
}

static HAL_StatusTypeDef send_cmd(const uint8_t* rom_code, uint8_t cmd)
{
  int i;

  if (OW_Reset() != HAL_OK)
    return HAL_ERROR;

  if (!rom_code) {
    OW_Write_Byte(DS18B20_SKIP_ROM);
  } else {
	  OW_Write_Byte(DS18B20_MATCH_ROM);
    for (i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
      OW_Write_Byte(rom_code[i]);
  }
  OW_Write_Byte(cmd);
  return HAL_OK;
}

HAL_StatusTypeDef ds18b20_start_measure(const uint8_t* rom_code)
{
  return send_cmd(rom_code, DS18B20_CONVERT_T);
}

static HAL_StatusTypeDef ds18b20_read_scratchpad(const uint8_t* rom_code, uint8_t* scratchpad)
{
  int i;
  uint8_t crc;

  if (send_cmd(rom_code, DS18B20_READ_SCRATCHPAD) != HAL_OK)
    return HAL_ERROR;

  for (i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++)
    scratchpad[i] = OW_Read_Byte();

  crc = OW_CRC(scratchpad, DS18B20_SCRATCHPAD_SIZE - 1);
  if (scratchpad[DS18B20_SCRATCHPAD_SIZE - 1] == crc)
    return HAL_OK;
  else
    return HAL_ERROR;
}

int16_t ds18b20_get_temp(const uint8_t* rom_code)
{
  uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
  int16_t temp;

  if (ds18b20_read_scratchpad(rom_code, scratchpad) != HAL_OK)
    return 85;

  memcpy(&temp, &scratchpad[0], sizeof(temp));

  return temp;
}
