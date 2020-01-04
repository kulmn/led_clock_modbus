/*
 * modbus_serial_drv.c
 *
 *  Created on: 17 дек. 2019 г.
 *      Author: kulish_y
 */

#include <protocols/modbus_serial_drv.h>

/* ----------------------- Static variables ---------------------------------*/

//static eMBEventType eQueuedEvent;
//static bool xEventInQueue;

static void vMBPortTimersDisable(Modbus_Interface *interface);

static bool xMBPortEventInit(Modbus_Interface *interface)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	driver->xEventInQueue = false;
	return true;
}

static bool xMBPortEventPost(Modbus_Interface *interface, eMBEventType eEvent)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	driver->xEventInQueue = true;
	driver->eQueuedEvent = eEvent;
	return true;

}

static bool xMBPortEventGet(Modbus_Interface *interface, eMBEventType *eEvent)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	bool xEventHappened = false;

	if (driver->xEventInQueue)
	{
		*eEvent = driver->eQueuedEvent;
		driver->xEventInQueue = false;
		xEventHappened = true;
	}
	return xEventHappened;

}

/* ----------------------- Initialize USART ----------------------------------*/
/* Called with databits = 8 for RTU */
static bool xMBPortSerialInit(Modbus_Interface *interface)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	bool bStatus;

	/*
	gpio_mode_setup(driver->usart_tx.port, GPIO_MODE_AF, GPIO_PUPD_NONE, driver->usart_tx.pins );
	gpio_set_af(driver->usart_tx.port, driver->usart_tx.alt_func, driver->usart_tx.pins );
	gpio_mode_setup(driver->usart_rx.port, GPIO_MODE_AF, GPIO_PUPD_NONE, driver->usart_rx.pins );
	gpio_set_af(driver->usart_rx.port, driver->usart_tx.alt_func, driver->usart_rx.pins );

	rcc_periph_clock_enable(MB_USART_RCC );
	nvic_enable_irq(MB_NVIC_USART_IRQ );

	// Setup UART parameters.
	usart_set_baudrate(driver->usart, ulBaudRate );
	usart_set_stopbits(driver->usart, USART_STOPBITS_1 );
	usart_set_flow_control(driver->usart, USART_FLOWCONTROL_NONE );
	usart_set_mode(driver->usart, USART_MODE_TX_RX );
	bStatus = true;
	switch (eParity)
	{
		case MB_PAR_NONE:
			usart_set_parity(driver->usart, USART_PARITY_NONE );
			break;
		case MB_PAR_ODD:
			usart_set_parity(driver->usart, USART_PARITY_ODD );
			break;
		case MB_PAR_EVEN:
			usart_set_parity(driver->usart, USART_PARITY_EVEN );
			break;
		default:
			bStatus = false;
			break;
	}

	if (eParity == MB_PAR_NONE) usart_set_databits(driver->usart, 8 );
	else usart_set_databits(driver->usart, 9 );

	if (bStatus == true)
	{
		// Finally enable the USART.
		usart_disable_rx_interrupt(driver->usart );
		usart_disable_tx_interrupt(driver->usart );
		usart_enable(driver->usart );
	}
*/

	bStatus = true;
	return bStatus;

}

/* ----------------------- Close Serial Port ----------------------------------*/
/* ----------------------- Close Ports -----------------------------*/
static void vMBPortClose(Modbus_Interface *interface)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	nvic_disable_irq(MB_NVIC_USART_IRQ );
	usart_disable(driver->usart_hl->usart );


	vMBPortTimersDisable(interface );
}

/* ----------------------- Enable USART interrupts -----------------------------*/
static void vMBPortSerialEnable(Modbus_Interface *interface, bool xRxEnable, bool xTxEnable)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	/* If xRXEnable enable serial receive interrupts. If xTxENable enable
	 * transmitter empty interrupts.
	 */
	if (xRxEnable)
	{
		usart_enable_rx_interrupt(driver->usart_hl->usart );
	} else
	{
		usart_disable_rx_interrupt(driver->usart_hl->usart);
	}

	if (xTxEnable)
	{
		usart_enable_tx_interrupt(driver->usart_hl->usart );
	} else
	{
		usart_disable_tx_interrupt(driver->usart_hl->usart );
	}

}

/* ----------------------- Get character ----------------------------------*/
static bool xMBPortSerialGetByte(Modbus_Interface *interface, uint8_t *pucByte)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	/* Return the byte in the UARTs receive buffer. This function is called
	 * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
	 */
	*pucByte = (uint8_t) usart_recv(driver->usart_hl->usart );
	return true;
}

/* -----------------------Send character  ----------------------------------*
static bool xMBPortSerialPutByte(Modbus_Interface *interface, uint8_t ucByte)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	// Put a byte in the UARTs transmit buffer. This function is called
	 // by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been called.
	usart_send(driver->usart_hl->usart, ucByte );
	return true;
}
*/

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

	usart_enable_tx_interrupt(driver->usart_hl->usart );
	return true;
}


/* ----------------------- Initialize Timer -----------------------------*/
static bool xMBPortTimersInit(Modbus_Interface *interface, uint16_t usTimeOut50us)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	usart_set_rx_timeout_value(driver->usart_hl->usart, 35 );
//	usart_enable_rx_timeout(MB_USART);
//	usart_enable_rx_timeout_interrupt(MB_USART);
//	timer_set_period(MB_TIMER, usTim1Timerout50us);	// 50 microseconds period
	return true;
}


/* ----------------------- Enable Timer -----------------------------*/
static void vMBPortTimersEnable(Modbus_Interface *interface)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	usart_enable_rx_timeout(driver->usart_hl->usart );
	usart_enable_rx_timeout_interrupt(driver->usart_hl->usart);

}

/* ----------------------- Disable timer -----------------------------*/
static void vMBPortTimersDisable(Modbus_Interface *interface)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) interface;

	usart_disable_rx_timeout(driver->usart_hl->usart );
	usart_disable_rx_timeout_interrupt(driver->usart_hl->usart );
}



/*
void MB_USART_ISR(void)
{
	// Read data register not empty flag
	if (usart_get_flag(MB_USART, USART_SR_RXNE ))
	{
		pxMBFrameCBByteReceived();
	}
	//   Transmit data register empty flag
	if (usart_get_flag(MB_USART, USART_SR_TXE ))
	{
		pxMBFrameCBTransmitterEmpty();
	}
	// Receiver timeout flag
	if (usart_get_flag(MB_USART, USART_ISR_RTOF ))
	{
		USART_ICR(MB_USART) |= USART_ICR_RTOCF;
		pxMBPortCBTimerExpired();
	}
}
*/
/*
void MB_USART_ISR_FN(void *arg)
{

//	MB_Serial_Driver *driver = (MB_Serial_Driver*) arg;

	MODBUS *modbus = (MODBUS *) arg;
	MB_Serial_Driver *driver = (MB_Serial_Driver*) modbus->driver;

	// Read data register not empty flag
	if (usart_get_flag(driver->usart_hl->usart, USART_SR_RXNE ))
	{
		pxMBFrameCBByteReceived(modbus);
	}
	//   Transmit data register empty flag
	if (usart_get_flag(driver->usart_hl->usart, USART_SR_TXE ))
	{
		pxMBFrameCBTransmitterEmpty( modbus);
	}
	// Receiver timeout flag
	if (usart_get_flag(driver->usart_hl->usart, USART_ISR_RTOF ))
	{
		USART_ICR(driver->usart_hl->usart) |= USART_ICR_RTOCF;
		pxMBPortCBTimerExpired(modbus);
	}
}
*/


void xMBRTUTransmitFSM(MODBUS *modbus)
{
	MB_Serial_Driver *driver = (MB_Serial_Driver*) modbus->driver;

	switch (modbus->eSndState)
	{
		// We should not get a transmitter event if the transmitter is in idle state.
		case STATE_TX_IDLE:
			// enable receiver/disable transmitter.
			modbus->driver->vMBPortSerialEnable(modbus->driver,  true, false);
			break;

		case STATE_TX_XMIT:
			// check if we are finished.
			if (driver->usart_hl->tx_buf.datalength)	// check if there's data left in the buffer
			{
				usart_send(driver->usart_hl->usart, bufferGetFromFront(&driver->usart_hl->tx_buf ) );// send byte from top of buffer
			} else
			{
				// Disable transmitter. This prevents another transmit buffer empty interrupt.
				//usart_disable_tx_interrupt(driver->usart_hl->usart );
				modbus->driver->vMBPortSerialEnable(modbus->driver,  true, false);

				driver->xEventInQueue = true;
				driver->eQueuedEvent = EV_FRAME_SENT;
				modbus->eSndState = STATE_TX_IDLE;
			}
			break;
	}
}


void MB_USART_ISR_FN(void *arg)
{
	MODBUS *modbus = (MODBUS *) arg;
	MB_Serial_Driver *driver = (MB_Serial_Driver*) modbus->driver;

	// Read data register not empty flag
	if (usart_get_flag(driver->usart_hl->usart, USART_SR_RXNE ))
	{
		pxMBFrameCBByteReceived(modbus);
	}
	//   Transmit data register empty flag
	if (usart_get_flag(driver->usart_hl->usart, USART_SR_TXE ))
	{
		pxMBFrameCBTransmitterEmpty( modbus);
	}
	// Receiver timeout flag
	if (usart_get_flag(driver->usart_hl->usart, USART_ISR_RTOF ))
	{
		USART_ICR(driver->usart_hl->usart) |= USART_ICR_RTOCF;
		pxMBPortCBTimerExpired(modbus);
	}
}


const Modbus_Interface MODBUS_INTERFACE = { xMBPortEventInit, xMBPortEventPost, xMBPortEventGet,
				xMBPortSerialInit, vMBPortClose, vMBPortSerialEnable, xMBPortSerialGetByte, xMBPortSerialSendBuf, //xMBPortSerialPutByte,
				xMBPortTimersInit, vMBPortTimersEnable, vMBPortTimersDisable };

