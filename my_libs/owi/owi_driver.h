/*
 * owi_stm32f030.h
 *
 *  Created on: 28 ���� 2015 �.
 *      Author: kulish_y
 */

#ifndef OWI_STM32F030_H_
#define OWI_STM32F030_H_


#include <stdlib.h>
#include <libopencm3/stm32/gpio.h>
#include "owi.h"



typedef struct
{
	OWI_Interface interface;
	uint32_t gpio;
	uint16_t pin;
} OWI_Driver;

extern const OWI_Interface OWI_PINDRIVER_INTERFACE;



#endif /* OWI_STM32F030_H_ */
