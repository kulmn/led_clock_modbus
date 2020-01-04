//****************************************************************************
//
//! \file wizchip_spi.h
//! \brief WIZnet wizchip EVB spi interface file
//!
//! Copyright (c)  2014, WIZnet Co., LTD.
//! All rights reserved.
//
//****************************************************************************

#ifndef _wizchip_SPI_H
#define _wizchip_SPI_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include "wizchip_conf.h"

#include "convert_fn.h"


#define W5500_SPI				SPI1
#define W5500_SPI_RCC		RCC_SPI1


#define W5500_AF				GPIO_AF5			// spi gpio alternate function
#define W5500_SPI_SCK		GPIOB, GPIO3
#define W5500_SPI_MISO		GPIOB, GPIO4
#define W5500_SPI_MOSI		GPIOA, GPIO12
#define W5500_SPI_CS			GPIOB, GPIO5
#define W5500_RESET			GPIOA, GPIO11


//#define wizchip_select() 			gpio_clear(W5500_SPI_CS)
//#define wizchip_release() 		gpio_set(W5500_SPI_CS)


#ifdef 	STM32F0
#define	GPIO_OSPEED_40MHZ		GPIO_OSPEED_HIGH

#endif



void  wizchip_select(void);
void  wizchip_deselect(void);
uint8_t wizchip_read(void);
void  wizchip_write(uint8_t data);
void wizchip_spi_init(void);
void w5500_init(void (*delay_ms)(uint16_t) );

#endif
