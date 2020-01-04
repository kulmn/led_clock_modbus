/*
 * i2c_hl.h
 *
 *  Created on: 16 ����. 2016 �.
 *      Author: kulish_y
 */

#ifndef INCLUDE_I2C_HAL_H_
#define INCLUDE_I2C_HAL_H_

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#define USE_I2C1


typedef struct
{
	uint32_t i2c_addr;

} I2C_HAL;

void i2c_init(I2C_HAL *uart);



#endif
