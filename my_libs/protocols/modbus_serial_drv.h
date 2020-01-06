/*
 * modbus_serial_drv.h
 *
 *  Created on: 17 дек. 2019 г.
 *      Author: kulish_y
 */

#ifndef MY_LIBS_PROTOCOLS_MODBUS_SERIAL_DRV_H_
#define MY_LIBS_PROTOCOLS_MODBUS_SERIAL_DRV_H_


#include "mb.h"
#include "mbport.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include "convert_fn.h"
#include <usart_hl.h>


typedef enum
{
	STATE_TX_IDLE, /*!< Transmitter is in idle state. */
	STATE_TX_XMIT /*!< Transmitter is in transfer state. */
} eMBSndState;

/* Hardware abstraction layer */
typedef enum
{
	STATE_RX_INIT, /*!< Receiver is in initial state. */
	STATE_RX_IDLE, /*!< Receiver is in idle state. */
	STATE_RX_RCV, /*!< Frame is beeing received. */
	STATE_RX_ERROR /*!< If the frame is invalid. */
} eMBRcvState;

/*! \ingroup modbus
 * \brief Parity used for characters in serial mode.
 *
 * The parity which should be applied to the characters sent over the serial
 * link. Please note that this values are actually passed to the porting
 * layer and therefore not all parity modes might be available.
 */
typedef enum
{
    MB_PAR_NONE,                /*!< No parity. */
    MB_PAR_ODD,                 /*!< Odd parity. */
    MB_PAR_EVEN                 /*!< Even parity. */
} eMBParity;

typedef struct
{
	Modbus_Interface	interface;
	eMBSndState 		eSndState;
	eMBRcvState 		eRcvState;
	uint8_t				FrameRcvFlag;
	USART_HAL			*usart_hl;
} MB_Serial_Driver;

extern const Modbus_Interface MODBUS_INTERFACE;

void MB_USART_ISR_FN(void *arg);

#endif /* MY_LIBS_PROTOCOLS_MODBUS_SERIAL_DRV_H_ */
