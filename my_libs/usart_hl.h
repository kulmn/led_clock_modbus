/*
 * usart_hl.h
 *
 *  Created on: 16 ����. 2016 �.
 *      Author: kulish_y
 */

#ifndef INCLUDE_USART_HAL_H_
#define INCLUDE_USART_HAL_H_

#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include "convert_fn.h"
#include "buffer.h"

	#define USE_USART1
//	#define USE_USART2



enum use_usarts {
#ifdef USE_USART1
	N_USART1,
#endif
#ifdef USE_USART2
	N_USART2,
#endif
	USE_USART_NUM,
};

#if defined(STM32F0)
//	#define	USART_STOPBITS_1	USART_CR2_STOP_1_0BIT
	#define	USART_SR_TXE		USART_ISR_TXE
	#define	USART_SR_RXNE		USART_ISR_RXNE
#elif defined(STM32F1)

#elif defined(STM32F2)

#elif defined(STM32F3)

#elif defined(STM32F4)

#elif defined(STM32L1)

#else
#       error "stm32 family not defined."
#endif


typedef struct  {
void (*handler) (void *);
void * handler_arg;
}usart_irq_record;

typedef enum
{
	USART_RESULT_OK, USART_RESULT_ERROR
} USART_Result;


/**
  * @brief USART Init Structure definition
  */
typedef struct
{
	uint32_t			 	usart;
	GPIO_HW_PIN		usart_tx;
	GPIO_HW_PIN		usart_rx;
	uint32_t				baudrate;
	uint32_t				parity;
	uint8_t 				rx_buf_size;
	uint8_t 				tx_buf_size;
	uint8_t 				*rx_buf_ptr;
	uint8_t 				*tx_buf_ptr;
}USART_Init;


typedef struct
{
	uint32_t usart;
	cBuffer rx_buf;				///< uart receive buffer
	cBuffer tx_buf;				///< uart transmit buffer
} USART_HAL;



void uart_init(USART_HAL *uart, USART_Init *init);

//! Gets a single byte from the uart receive buffer.
/// Returns the byte, or -1 if no byte is available (getchar-style).
int uartGetByte(USART_HAL *uart);

//! Gets a single byte from the uart receive buffer.
/// Function returns TRUE if data was available, FALSE if not.
/// Actual data is returned in variable pointed to by "data".
/// Example usage:
/// \code
/// char myReceivedByte;
/// uartReceiveByte( &myReceivedByte );
/// \endcode
USART_Result usart_hl_get_byte(USART_HAL *uart, uint8_t* data);

//! Flushes (deletes) all data from receive buffer.
///
void uartFlushReceiveBuffer(USART_HAL *uart);

// Buffered send a byte via the uart using interrupt control.
USART_Result usart_hl_send(USART_HAL *uart, uint8_t data);

// Buffered send a string via the uart using interrupt control.
USART_Result usart_hl_send_str(USART_HAL *uart, uint8_t* str);

//! Sends a block of data via the uart using interrupt control.
/// \param buffer	pointer to data to be sent
///	\param nBytes	length of data (number of bytes to sent)
uint8_t uartSendBuffer(char *buffer, uint16_t nBytes);

void usart_hl_set_irq_handler(USART_HAL *usart_hl, void (*handler)(void *), void * handler_arg);
void def_usart_irq_fn(void *arg);

#endif /* INCLUDE_USART_HAL_H_ */
