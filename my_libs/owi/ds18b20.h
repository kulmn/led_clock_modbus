/*
 * ds18b20.h
 *
 *  Created on: 31 ���� 2015 �.
 *      Author: kulish_y
 */

#ifndef DS18B20_H_
#define DS18B20_H_

#include "owi.h"

#ifdef USE_FREE_RTOS

// FreeRTOS include
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define WAIT_TICKS_A		5
#define WAIT_TICKS_B		25
#define RTOS_Delay(ticks) { vTaskDelay(ticks); }

#else

#define RTOS_Delay(ticks) {  }

#endif

typedef struct
{
	uint8_t n_sensor;
	int16_t raw_value;
	int8_t value;
	uint8_t tens_value;
} DS18B20_TypeDef;

void DS18B20_Init(OWI *owi_int, uint8_t config);
void DS18B20_Read_Struct(OWI *owi_int, DS18B20_TypeDef *temper);
float DS18B20_Read_Float(OWI *owi_int);
uint16_t DS18B20_ReadTemperature(OWI *owi_int);


// Supported devices family id's
#define DS18S20_FAMILY_ID    0x10
#define DS18B20_FAMILY_ID    0x28

#define DS18B20_12BIT_RES	0x7F
#define DS18B20_11BIT_RES	0x5F
#define DS18B20_10BIT_RES	0x3F
#define DS18B20_9BIT_RES	0x1F

#endif /* DS18B20_H_ */
