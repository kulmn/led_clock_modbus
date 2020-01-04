/*
 * ds18b20.c
 *
 *  Created on: 30 ���� 2015 �.
 *      Author: kulish_y
 */

#include "ds18b20.h"

void DS18B20_Init(OWI *owi_int, uint8_t config)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		OWI_DetectPresence(owi_int );
		OWI_SendByte(owi_int, OWI_ROM_SKIP );
		OWI_SendByte(owi_int, OWI_WRITE_SCRATCHPAD );
		OWI_SendByte(owi_int, 0 );
		OWI_SendByte(owi_int, 0 );
		OWI_SendByte(owi_int, config );
	}
//    OWI_DetectPresence(owi_int);
//    OWI_SendByte(owi_int, OWI_ROM_SKIP);

//    OWI_SendByte(owi_int, OWI_COPY_SCRATCHPAD);

}


/******************************************************************************/
void DS18B20_Read_Struct(OWI *owi_int, DS18B20_TypeDef *temper)
{
	uint16_t temp;
	int16_t raw_temp;

	raw_temp = DS18B20_ReadTemperature(owi_int );
	temp = raw_temp;
	temper->raw_value = raw_temp;
	temper->n_sensor = owi_int->owi_device_n;
	if (temp & 0x8000)				// temperature < 0
	{
		temp = (~temp) + 1;
		temper->value = (~(temp >> 4) + 1) & 0x00FF;
		temp &= 0x000F;
		temper->tens_value = (uint8_t) ((temp >> 1) + (temp >> 3));
	} else
	{
		temper->value = (temp >> 4) & 0x00FF;
		temp &= 0x000F;
		temper->tens_value = (uint8_t) ((temp >> 1) + (temp >> 3));
	}
}



/******************************************************************************/
// TODO not tested
float DS18B20_Read_Float(OWI *owi_int)
{
	uint16_t temp;
	temp = DS18B20_ReadTemperature(owi_int );

//	float f_temp = (temp & 0x0F) * 0.0625;    // calculate .4 part
//	f_temp += (temp >> 4);    // add 7.0 part to it

	float f_temp = temp * 0.0625;

	return f_temp;
}




/******************************************************************************/
uint16_t DS18B20_ReadTemperature(OWI *owi_int)
{
	uint8_t temp_hi, temp_lo;

	RTOS_Delay(WAIT_TICKS_A );
	if (!OWI_DetectPresence(owi_int ))			// 960 mks
	return -127; // Error

	RTOS_Delay(WAIT_TICKS_A );
	OWI_SendByte(owi_int, OWI_ROM_SKIP );			// 560 mks
	RTOS_Delay(WAIT_TICKS_A );
	// Send start conversion command.
	OWI_SendByte(owi_int, OWI_START_CONVERSION );	// 560 mks

	RTOS_Delay(WAIT_TICKS_A );
	// Wait until conversion is finished.
	// Bus line is held low until conversion is finished.
	while (!OWI_ReadBit(owi_int )) 				// 70 mks
		RTOS_Delay(WAIT_TICKS_B );

	RTOS_Delay(WAIT_TICKS_A );
	// Reset, presence.
	if (!OWI_DetectPresence(owi_int ))			// 960 mks
	return -127; // Error

	RTOS_Delay(WAIT_TICKS_A );
	OWI_SendByte(owi_int, OWI_ROM_SKIP );			// 560 mks
	RTOS_Delay(WAIT_TICKS_A );
	// Send READ SCRATCHPAD command.
	OWI_SendByte(owi_int, OWI_READ_SCRATCHPAD );		// 560 mks

	RTOS_Delay(WAIT_TICKS_A );
	// Read only two first bytes (temperature low, temperature high)
	temp_lo = OWI_ReceiveByte(owi_int );			// 560 mks
	RTOS_Delay(WAIT_TICKS_A );
	temp_hi = OWI_ReceiveByte(owi_int );			// 560 mks

	return ((temp_hi << 8) | temp_lo);
}




