/*
 * hc595.h
 *
 *  Created on: 3 ����. 2016 �.
 *      Author: kulish_y
 */

#ifndef HC595_H_
#define HC595_H_

#include <stdint.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include "convert_fn.h"


#ifdef STM32F0
	#define		HC595_GPIO_SPEED		GPIO_OSPEED_HIGH
#else
	#define		HC595_GPIO_SPEED		GPIO_OSPEED_40MHZ
#endif


#define HC595_SPI				SPI1
#define HC595_SPI_RCC			RCC_SPI1
#define HC595_AF				GPIO_AF0

#define HC595_SPI_SCK			GPIOA, GPIO5
#define HC595_SPI_MOSI			GPIOA, GPIO7
#define HC595_SPI_CS			GPIOA, GPIO6


void HC595_spi_init(void);
void HC595_send(uint16_t data);

#endif /* HC595_H_ */
