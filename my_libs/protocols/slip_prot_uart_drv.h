/*
 * slip_protocol_usart_drv.h
 *
 *  Created on: 24 ����. 2016 �.
 *      Author: kulish_y
 */

#ifndef SLIP_PROTOCOL_USART_DRV_H_
#define SLIP_PROTOCOL_USART_DRV_H_

#include <stdlib.h>
#include <libopencm3/stm32/gpio.h>
#include <protocols/slip_prot.h>

#include "../my_libs/usart_hl.h"

typedef struct
{
	SLIP_Interface interface;
	USART_HAL *usart_drv;
	uint16_t out_buf;

} SLIP_USART_Driver;

extern const SLIP_Interface SLIP_USART_INTERFACE;

#endif /* SLIP_PROTOCOL_USART_DRV_H_ */
