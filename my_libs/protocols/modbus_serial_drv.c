/*
 * modbus_serial_drv.c
 *
 *  Created on: 17 дек. 2019 г.
 *      Author: kulish_y
 */

#include <protocols/modbus_serial_drv.h>



void xMBPortStart(Modbus_Interface *interface)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	ENTER_CRITICAL_SECTION( );
	// Initially the receiver is in the state STATE_RX_INIT. we start
	// the timer and if no character is received within t3.5 we change
	// to STATE_RX_IDLE. This makes sure that we delay startup of the
	// modbus protocol stack until the bus is free.

	usart_disable_tx_interrupt(driver->usart_hl->usart );
	usart_enable_rx_interrupt(driver->usart_hl->usart );
	usart_enable_rx_timeout(driver->usart_hl->usart );
	usart_enable_rx_timeout_interrupt(driver->usart_hl->usart );

//	driver->eRcvState = STATE_RX_INIT;
	driver->eRcvState = STATE_RX_IDLE;

//	interface->xMBPortEventPost(interface, EV_READY );

	EXIT_CRITICAL_SECTION( );
}

void xMBPortStop(Modbus_Interface *interface)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	ENTER_CRITICAL_SECTION( );

	usart_disable_tx_interrupt(driver->usart_hl->usart );
	usart_disable_rx_interrupt(driver->usart_hl->usart );
	usart_disable_rx_timeout(driver->usart_hl->usart );
	usart_disable_rx_timeout_interrupt(driver->usart_hl->usart );

	EXIT_CRITICAL_SECTION( );
}



static bool xMBPortFrameReceived(Modbus_Interface *interface)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;


	if (driver->FrameRcvFlag)
	{
		driver->FrameRcvFlag = 0;
		return true;
	}
	return false;

}

/* ----------------------- Initialize USART ----------------------------------*/
/* Called with databits = 8 for RTU */
static bool xMBPortInit(Modbus_Interface *interface)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	bool bStatus;

	usart_set_rx_timeout_value(driver->usart_hl->usart, 35 );

	bStatus = true;
	return bStatus;
}



static bool xMBPortSerialSendBuf(Modbus_Interface *interface, uint8_t *snd_buf, uint8_t len)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	uint16_t buf_free = bufferIsNotFull(&driver->usart_hl->tx_buf);

	if (len > buf_free) return false;		// no space in buffer

	while (len--)
	{
		if (bufferAddToEnd(&driver->usart_hl->tx_buf, *snd_buf++ ) == BUFFER_RESULT_ERROR)		// error add new data
		{
			bufferFlush(&driver->usart_hl->tx_buf);		// clear buffer data
			return false;
		}
	}

	driver->eSndState = STATE_TX_XMIT;
	usart_enable_tx_interrupt(driver->usart_hl->usart );
	return true;
}


void xMBRTUTimerT35Expired(MODBUS *modbus)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) modbus->driver;

	switch (driver->eRcvState)
	{
		// Timer t35 expired. Startup phase is finished.
//		case STATE_RX_INIT:
//			modbus->driver->xMBPortEventPost(modbus->driver, EV_READY );
//			break;

			// A frame was received and t35 expired. Notify the listener that a new frame was received.
		case STATE_RX_RCV:

			driver->FrameRcvFlag = 1;
			 uint8_t len = driver->usart_hl->rx_buf.datalength;
			 uint8_t *ptr = modbus->packet_buf;
			modbus->packet_size = len;
			while (len--)
			{
				*ptr = bufferGetFromFront(&driver->usart_hl->rx_buf);
				ptr++;
			}

			break;

			// An error occured while receiving the frame.
		case STATE_RX_ERROR:
			break;

	}

	usart_disable_rx_timeout(driver->usart_hl->usart );
	usart_disable_rx_timeout_interrupt(driver->usart_hl->usart );
	driver->eRcvState = STATE_RX_IDLE;
}


void xMBRTUTransmitFSM(MODBUS *modbus)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) modbus->driver;

	switch (driver->eSndState)
	{
		// We should not get a transmitter event if the transmitter is in idle state.
		case STATE_TX_IDLE:
			// enable receiver/disable transmitter.
			usart_disable_tx_interrupt(driver->usart_hl->usart );
			usart_enable_rx_interrupt(driver->usart_hl->usart );
			break;

		case STATE_TX_XMIT:
			// check if we are finished.
			if (driver->usart_hl->tx_buf.datalength)	// check if there's data left in the buffer
			{
				usart_send(driver->usart_hl->usart, bufferGetFromFront(&driver->usart_hl->tx_buf ) );// send byte from top of buffer
			} else
			{
				// Disable transmitter. This prevents another transmit buffer empty interrupt.
				usart_disable_tx_interrupt(driver->usart_hl->usart );
				usart_enable_rx_interrupt(driver->usart_hl->usart );

				driver->eSndState = STATE_TX_IDLE;
			}
			break;
	}
}


void xMBRTUReceiveFSM(MODBUS *modbus)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) modbus->driver;
	uint8_t ucByte;

	// Always read the character.
	ucByte = usart_recv(driver->usart_hl->usart );
	//modbus->driver->xMBPortSerialGetByte(modbus->driver, (uint8_t*) &ucByte );
	//(void) xMBPortSerialGetByte((uint8_t*) &ucByte );

	switch (driver->eRcvState)
	{
			// In the error state we wait until all characters in the damaged frame are transmitted.
		case STATE_RX_ERROR:
			break;
			// In the idle state we wait for a new character. If a character
			// is received the t1.5 and t3.5 timers are started and the
			//receiver is in the state STATE_RX_RECEIVCE.
		case STATE_RX_IDLE:
			if (bufferAddToEnd(&driver->usart_hl->rx_buf, ucByte ) == BUFFER_RESULT_ERROR)		// error add new data
			{
				bufferFlush(&driver->usart_hl->rx_buf);		// clear buffer data
				driver->eRcvState = STATE_RX_ERROR;
			}else
			{
				driver->eRcvState = STATE_RX_RCV;
			}
			break;
			// We are currently receiving a frame. Reset the timer after
			// every character received. If more than the maximum possible
			// number of bytes in a modbus frame is received the frame is  ignored.
		case STATE_RX_RCV:
			if (bufferAddToEnd(&driver->usart_hl->rx_buf, ucByte ) == BUFFER_RESULT_ERROR)		// error add new data
			{
				bufferFlush(&driver->usart_hl->rx_buf);		// clear buffer data
				driver->eRcvState = STATE_RX_ERROR;
			}else
			{
				driver->eRcvState = STATE_RX_RCV;
			}
			break;
	}

	// Enable t3.5 timers.
	usart_enable_rx_timeout(driver->usart_hl->usart );
	usart_enable_rx_timeout_interrupt(driver->usart_hl->usart);
}


void MB_USART_ISR_FN(void *arg)
{
	MODBUS *modbus = (MODBUS *) arg;
	MB_Serial_Driver *driver = (MB_Serial_Driver*) modbus->driver;

	// Read data register not empty flag
	if (usart_get_flag(driver->usart_hl->usart, USART_SR_RXNE ))
	{
		xMBRTUReceiveFSM(modbus);
	}
	//   Transmit data register empty flag
	if (usart_get_flag(driver->usart_hl->usart, USART_SR_TXE ))
	{
		xMBRTUTransmitFSM( modbus);
	}
	// Receiver timeout flag
	if (usart_get_flag(driver->usart_hl->usart, USART_ISR_RTOF ))
	{
		USART_ICR(driver->usart_hl->usart) |= USART_ICR_RTOCF;
		xMBRTUTimerT35Expired(modbus);
	}
}


const Modbus_Interface MODBUS_INTERFACE = { xMBPortFrameReceived,
				xMBPortInit, xMBPortStart, xMBPortStop, xMBPortSerialSendBuf };

