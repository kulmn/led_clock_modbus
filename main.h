/*
 * main.h
 *
 *  Created on: 21 ���. 2014 �.
 *      Author: kulish_y
 */

#ifndef MAIN_H_
#define MAIN_H_


#include <indicators/led7seg_driver_pin.h>
#include <stdint.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/spi.h>

#include <protocols/modbus_serial_drv.h>
#include <modbus_cb.h>



// FreeRTOS inc
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#include "convert_fn.h"
#include <usart_hl.h>
#include <rtc_hal.h>
#include "timer_hal.h"

#include "owi/owi.h"
#include <owi/owi_driver.h>
#include "owi/ds18b20.h"

#include "indicators/led7seg.h"
#include "indicators/led7seg_driver_pin.h"
#include "indicators/led7seg_driver_sr.h"

#include <rtc/ds1307.h>




/**** PINs defines *******/

// leg7seg
#define LED_IND_DIG_0		GPIOA, GPIO3
#define LED_IND_DIG_1		GPIOA, GPIO4
#define LED_IND_DIG_2		GPIOA, GPIO2
#define LED_IND_DIG_3		GPIOA, GPIO1

// hc595 masks
#define LED_IND_SEG_A		( (uint8_t) 1 << 6 )
#define LED_IND_SEG_B		( (uint8_t) 1 << 7 )
#define LED_IND_SEG_C		( (uint8_t) 1 << 0 )
#define LED_IND_SEG_D		( (uint8_t) 1 << 1 )
#define LED_IND_SEG_E		( (uint8_t) 1 << 2 )
#define LED_IND_SEG_F		( (uint8_t) 1 << 4 )
#define LED_IND_SEG_G		( (uint8_t) 1 << 3 )
#define LED_IND_SEG_H		( (uint8_t) 1 << 5 )

// buttons
#define BUTTN_HOURS_UP		GPIOB, GPIO0
#define BUTTN_HOURS_DN		GPIOB, GPIO1
#define BUTTN_MINUTES_UP		GPIOB, GPIO3
#define BUTTN_MINUTES_DN		GPIOA, GPIO15

// DS18BB20 temperature sensors
#define T_SENS_INT_PIN		GPIOB, GPIO5
#define T_SENS_INT_NUM	1
#define T_SENS_EXT_PIN		GPIOB, GPIO4
#define T_SENS_EXT_NUM	2


#define USART1_TX_PIN			GPIOB, GPIO6, GPIO_AF0
#define USART1_RX_PIN			GPIOB, GPIO7, GPIO_AF0

#define MB_USART_TX_PIN		GPIOB, GPIO6, GPIO_AF0
#define MB_USART_RX_PIN		GPIOB, GPIO7, GPIO_AF0

//#define USART2_TX_PIN	GPIOA, GPIO2
//#define USART2_RX_PIN	GPIOA, GPIO3

#define I2C1_SCL			GPIOA, GPIO9, GPIO_AF4
#define I2C1_SDA			GPIOA, GPIO10, GPIO_AF4


void periphery_init(void);



#endif /* MAIN_H_ */
