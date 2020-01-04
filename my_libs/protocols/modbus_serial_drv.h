/*
 * modbus_serial_drv.h
 *
 *  Created on: 17 дек. 2019 г.
 *      Author: kulish_y
 */

#ifndef MY_LIBS_PROTOCOLS_MODBUS_SERIAL_DRV_H_
#define MY_LIBS_PROTOCOLS_MODBUS_SERIAL_DRV_H_

#include <protocols/modbus.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
//#include <libopencm3/stm32/timer.h>
#include "convert_fn.h"
#include <usart_hl.h>


#define MB_NVIC_USART_IRQ	NVIC_USART1_IRQ



typedef struct
{
	Modbus_Interface	interface;
	uint8_t				xEventInQueue;
	eMBEventType		eQueuedEvent;
	USART_HAL			*usart_hl;
} MB_Serial_Driver;

extern const Modbus_Interface MODBUS_INTERFACE;

void MB_USART_ISR_FN(void *arg);

#endif /* MY_LIBS_PROTOCOLS_MODBUS_SERIAL_DRV_H_ */
