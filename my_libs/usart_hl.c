/*
 * usart_hl.c
 *
 *  Created on: 16 ����. 2016 �.
 *      Author: kulish_y
 */

#include "usart_hl.h"

static usart_irq_record usart_irq[USE_USART_NUM];



void uart_init(USART_HAL *uart, USART_Init *init)
{
	/* Setup USART parameters. */
	uart->usart = init->usart;

	// Setup GPIO pins
	gpio_mode_setup(init->usart_tx.port, GPIO_MODE_AF, GPIO_PUPD_NONE, init->usart_tx.pins);
	gpio_set_af(init->usart_tx.port, init->usart_tx.alt_func, init->usart_tx.pins );
	gpio_mode_setup(init->usart_rx.port, GPIO_MODE_AF, GPIO_PUPD_NONE, init->usart_rx.pins );
	gpio_set_af(init->usart_rx.port, init->usart_tx.alt_func, init->usart_rx.pins );

	usart_set_baudrate(uart->usart, init->baudrate );
	usart_set_databits(uart->usart, 8 );
	usart_set_parity(uart->usart, init->parity );
	usart_set_stopbits(uart->usart, USART_STOPBITS_1 );
	usart_set_mode(uart->usart, USART_MODE_TX_RX );
	usart_set_flow_control(uart->usart, USART_FLOWCONTROL_NONE );

	usart_enable_rx_interrupt(uart->usart );

	// initialize the UART receive buffer
	bufferInit(&uart->rx_buf, (uint8_t *) init->rx_buf_ptr, init->rx_buf_size );
	// initialize the UART transmit buffer
	bufferInit(&uart->tx_buf, (uint8_t *) init->tx_buf_ptr, init->tx_buf_size );



	usart_enable(uart->usart );		// Finally enable the USART.
}

// Buffered send a byte via the uart using interrupt control.
USART_Result usart_hl_send(USART_HAL *uart, uint8_t data)
{
	if (bufferAddToEnd(&uart->tx_buf, data ) == BUFFER_RESULT_OK)
	{
		usart_enable_tx_interrupt(uart->usart );
		return USART_RESULT_OK;
	} else return USART_RESULT_ERROR;
}

// Buffered send a string via the uart using interrupt control.
USART_Result usart_hl_send_str(USART_HAL *uart, uint8_t* str)
{
	/* Go through entire string */
	while (*str)
	{
		if (bufferAddToEnd(&uart->tx_buf, *str++ ) == BUFFER_RESULT_ERROR) return USART_RESULT_ERROR;
	}

	usart_enable_tx_interrupt(uart->usart );
	return USART_RESULT_OK;
}

// gets a single byte from the uart receive buffer (getchar-style)
int uartGetByte(USART_HAL *uart)
{
	uint8_t c;
	if (usart_hl_get_byte(uart, &c )) return c;
	else return -1;
}

// gets a byte (if available) from the uart receive buffer
USART_Result usart_hl_get_byte(USART_HAL *uart, uint8_t* rxData)
{
	// make sure we have data
	if (uart->rx_buf.datalength)
	{
		// get byte from beginning of buffer
		*rxData = bufferGetFromFront(&uart->rx_buf );
		return USART_RESULT_OK;
	} else
	{
		// no data
		return USART_RESULT_ERROR;
	}
}

// flush all data out of the receive buffer
void uartFlushReceiveBuffer(USART_HAL *uart)
{
	// flush all data from receive buffer
	//bufferFlush(&uartRxBuffer);
	// same effect as above
	uart->rx_buf.datalength = 0;
}


/******************************************************************************/
void usart_hl_set_irq_handler(USART_HAL *usart_hl, void (*handler)(void *), void * handler_arg)
{
	switch (usart_hl->usart)
	{
#ifdef USE_USART1
		case USART1:
			usart_irq[N_USART1].handler = handler;
			usart_irq[N_USART1].handler_arg = handler_arg;
			break;
#endif
#ifdef USE_USART2
		case USART2:
			usart_irq[N_USART2].handler = handler;
			usart_irq[N_USART2].handler_arg = handler_arg;
			break;
#endif
	}

}


/******************************************************************************/

#ifdef USE_USART1
void USART1_IRQHandler(void)
{
	usart_irq[N_USART1].handler( usart_irq[N_USART1].handler_arg );
}
#endif

#ifdef USE_USART2
void USART2_IRQHandler(void)
{
	usart_irq[N_USART2].handler( usart_irq[N_USART2].handler_arg );
}
#endif




void def_usart_irq_fn(void *arg)
{
	USART_HAL *usart_hl = (USART_HAL*) arg;

	if (usart_get_flag(usart_hl->usart, USART_SR_TXE ))
	{
		if (usart_hl->tx_buf.datalength)	// check if there's data left in the buffer
		{
			usart_send(usart_hl->usart, bufferGetFromFront(&usart_hl->tx_buf ) );// send byte from top of buffer
		} else usart_disable_tx_interrupt(usart_hl->usart );
	}

	if (usart_get_flag(usart_hl->usart, USART_SR_RXNE ))
	{
		bufferAddToEnd(&usart_hl->rx_buf, usart_recv(usart_hl->usart ) );
	}
}

