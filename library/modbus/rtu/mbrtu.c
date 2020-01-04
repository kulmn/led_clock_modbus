/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbrtu.c,v 1.18 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbrtu.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */





/* ----------------------- Static variables ---------------------------------*/
//static volatile eMBSndState eSndState;
//static volatile eMBRcvState eRcvState;

volatile uint8_t ucRTUBuf[MB_SER_PDU_SIZE_MAX];

//static volatile uint8_t *pucSndBufferCur;
static volatile uint16_t usSndBufferCount;

static volatile uint16_t usRcvBufferPos;

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode eMBRTUInit(MODBUS *modbus)
{
	eMBErrorCode eStatus = MB_ENOERR;
	uint32_t usTimerT35_50us;

//	ENTER_CRITICAL_SECTION( );

	/* Modbus RTU uses 8 Databits. */
	if (modbus->driver->xMBPortSerialInit(modbus->driver ) != true)
//	if (xMBPortSerialInit(modbus ) != true)
	{
		eStatus = MB_EPORTERR;
	} else
	{

	/*
		// If baudrate > 19200 then we should use the fixed timer values
		// t35 = 1750us. Otherwise t35 must be 3.5 times the character time.

		if (ulBaudRate > 19200)
		{
			usTimerT35_50us = 35; // 1800us.
		} else
		{
			// The timer reload value for a character is given by:
			 // ChTimeValue = Ticks_per_1s / ( Baudrate / 11 ) = 11 * Ticks_per_1s / Baudrate = 220000 / Baudrate
			 // The reload for t3.5 is 1.5 times this value and similary for t3.5.

			usTimerT35_50us = (7UL * 220000UL) / (2UL * ulBaudRate);
		}
	*/

		if (modbus->driver->xMBPortTimersInit(modbus->driver, (uint16_t) usTimerT35_50us ) != true)
//		if (xMBPortTimersInit((uint16_t) usTimerT35_50us ) != true)
		{
			eStatus = MB_EPORTERR;
		}
	}
//	EXIT_CRITICAL_SECTION( );

	return eStatus;
}

void eMBRTUStart(MODBUS *modbus)
{
//    ENTER_CRITICAL_SECTION(  );
	/* Initially the receiver is in the state STATE_RX_INIT. we start
	 * the timer and if no character is received within t3.5 we change
	 * to STATE_RX_IDLE. This makes sure that we delay startup of the
	 * modbus protocol stack until the bus is free.
	 */

	modbus->eRcvState = STATE_RX_INIT;

	modbus->driver->vMBPortSerialEnable(modbus->driver, true, false);
	modbus->driver->vMBPortTimersEnable(modbus->driver);
//	modbus->driver->xMBPortEventPost(modbus->driver, EV_READY );


//    EXIT_CRITICAL_SECTION(  );
}

void eMBRTUStop(MODBUS *modbus)
{
	ENTER_CRITICAL_SECTION( );
	modbus->driver->vMBPortSerialEnable(modbus->driver, false, false);
	//vMBPortSerialEnable( false, false );
	modbus->driver->vMBPortTimersDisable(modbus->driver);
	//vMBPortTimersDisable();
	EXIT_CRITICAL_SECTION( );
}

eMBErrorCode eMBRTUReceive(MODBUS *modbus, uint8_t *pucRcvAddress, uint8_t **pucFrame, uint16_t *pusLength)
{
	bool xFrameReceived = false;
	eMBErrorCode eStatus = MB_ENOERR;

	ENTER_CRITICAL_SECTION( );
//    assert( usRcvBufferPos < MB_SER_PDU_SIZE_MAX );

	// Length and CRC check
	if ((usRcvBufferPos >= MB_SER_PDU_SIZE_MIN) && (usMBCRC16((uint8_t*) ucRTUBuf, usRcvBufferPos ) == 0) )
	{
		// Save the address field. All frames are passed to the upper layed and the decision if a frame is used is done there.
		*pucRcvAddress = ucRTUBuf[MB_SER_PDU_ADDR_OFF];

		// Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus size of address field and CRC checksum.
		*pusLength = (uint16_t) (usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC);

		// Return the start of the Modbus PDU to the caller.
		*pucFrame = (uint8_t*) &ucRTUBuf[MB_SER_PDU_PDU_OFF];
		xFrameReceived = true;
	} else
	{
		eStatus = MB_EIO;
	}

	EXIT_CRITICAL_SECTION( );
	return eStatus;
}

/*
eMBErrorCode eMBRTUSend(MODBUS *modbus, uint8_t ucSlaveAddress, const uint8_t *pucFrame, uint16_t usLength)
{
	uint8_t *pucSndBufferCur;
	eMBErrorCode eStatus = MB_ENOERR;
	uint16_t usCRC16;

	ENTER_CRITICAL_SECTION( );

	// Check if the receiver is still in idle state. If not we where to
	 // slow with processing the received frame and the master sent another
	 // frame on the network. We have to abort sending the frame.

	if (modbus->eRcvState == STATE_RX_IDLE)
	{
		// First byte before the Modbus-PDU is the slave address.
		pucSndBufferCur = (uint8_t*) pucFrame - 1;
		usSndBufferCount = 1;

		// Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU.
		pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
		usSndBufferCount += usLength;

		// Calculate CRC16 checksum for Modbus-Serial-Line-PDU.
		usCRC16 = usMBCRC16((uint8_t*) pucSndBufferCur, usSndBufferCount );
		ucRTUBuf[usSndBufferCount++] = (uint8_t) (usCRC16 & 0xFF);
		ucRTUBuf[usSndBufferCount++] = (uint8_t) (usCRC16 >> 8);

		// Activate the transmitter.
		modbus->eSndState = STATE_TX_XMIT;

		modbus->driver->xMBPortSerialSendBuf(modbus->driver, pucSndBufferCur, usSndBufferCount);

		//modbus->driver->vMBPortSerialEnable(modbus->driver, false, true );
		//vMBPortSerialEnable( false, true );
	} else
	{
		eStatus = MB_EIO;
	}
	EXIT_CRITICAL_SECTION( );
	return eStatus;
}
*/

eMBErrorCode eMBRTUSend(MODBUS *modbus, uint8_t ucSlaveAddress, const uint8_t *pucFrame, uint16_t usLength)
{
	uint8_t *pucSndBufferCur;
	eMBErrorCode eStatus;
	uint16_t usCRC16;

	// First byte before the Modbus-PDU is the slave address.
	pucSndBufferCur = (uint8_t*) pucFrame - 1;
	usSndBufferCount = 1;

	// Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU.
	pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
	usSndBufferCount += usLength;

	// Calculate CRC16 checksum for Modbus-Serial-Line-PDU.
	usCRC16 = usMBCRC16((uint8_t*) pucSndBufferCur, usSndBufferCount );
	ucRTUBuf[usSndBufferCount++] = (uint8_t) (usCRC16 & 0xFF);
	ucRTUBuf[usSndBufferCount++] = (uint8_t) (usCRC16 >> 8);

	// Activate the transmitter.
	if (modbus->driver->xMBPortSerialSendBuf(modbus->driver, pucSndBufferCur, usSndBufferCount ))
	{
		modbus->eSndState = STATE_TX_XMIT;
		eStatus = MB_ENOERR;
	}else
		eStatus = MB_EIO;

	return eStatus;
}


bool xMBRTUReceiveFSM(MODBUS *modbus)
{
	bool xTaskNeedSwitch = false;
	uint8_t ucByte;

	// Always read the character.
	modbus->driver->xMBPortSerialGetByte(modbus->driver, (uint8_t*) &ucByte );
	//(void) xMBPortSerialGetByte((uint8_t*) &ucByte );

	switch (modbus->eRcvState)
	{
		// If we have received a character in the init state we have to wait until the frame is finished.
		case STATE_RX_INIT:
			break;
			// In the error state we wait until all characters in the damaged frame are transmitted.
		case STATE_RX_ERROR:
			break;
			// In the idle state we wait for a new character. If a character
			// is received the t1.5 and t3.5 timers are started and the
			//receiver is in the state STATE_RX_RECEIVCE.
		case STATE_RX_IDLE:
			usRcvBufferPos = 0;
			ucRTUBuf[usRcvBufferPos++] = ucByte;
			modbus->eRcvState = STATE_RX_RCV;
			break;
			/* We are currently receiving a frame. Reset the timer after
			 * every character received. If more than the maximum possible
			 * number of bytes in a modbus frame is received the frame is
			 * ignored.
			 */
		case STATE_RX_RCV:
			if (usRcvBufferPos < MB_SER_PDU_SIZE_MAX) ucRTUBuf[usRcvBufferPos++] = ucByte;
			else modbus->eRcvState = STATE_RX_ERROR;
			break;
	}

	modbus->driver->vMBPortTimersEnable(modbus->driver );			// Enable t3.5 timers.

	return xTaskNeedSwitch;
}


/*
bool xMBRTUTransmitFSM(MODBUS *modbus)
{
	bool xNeedPoll = false;

//    assert( eRcvState == STATE_RX_IDLE );

	switch (modbus->eSndState)
	{
		// We should not get a transmitter event if the transmitter is in idle state.
		case STATE_TX_IDLE:
			// enable receiver/disable transmitter.
			modbus->driver->vMBPortSerialEnable(modbus->driver,  true, false);
			//vMBPortSerialEnable( true, false );
			break;

		case STATE_TX_XMIT:
			// check if we are finished.
			if (usSndBufferCount != 0)
			{
				modbus->driver->xMBPortSerialPutByte(modbus->driver, (uint8_t) *pucSndBufferCur);
				//xMBPortSerialPutByte((uint8_t) *pucSndBufferCur );
				pucSndBufferCur++; // next byte in sendbuffer.
				usSndBufferCount--;
			} else
			{
				xNeedPoll = modbus->driver->xMBPortEventPost(modbus->driver, EV_FRAME_SENT);
				//xNeedPoll = xMBPortEventPost(modbus, EV_FRAME_SENT );
				// Disable transmitter. This prevents another transmit buffer empty interrupt.
				modbus->driver->vMBPortSerialEnable(modbus->driver,  true, false);
				//vMBPortSerialEnable( true, false );
				modbus->eSndState = STATE_TX_IDLE;
			}
			break;
	}
	return xNeedPoll;
}

*/

bool xMBRTUTimerT35Expired(MODBUS *modbus)
{
	bool xNeedPoll = false;

	switch (modbus->eRcvState)
	{
		// Timer t35 expired. Startup phase is finished.
		case STATE_RX_INIT:
			xNeedPoll = modbus->driver->xMBPortEventPost(modbus->driver, EV_READY );
			break;

			// A frame was received and t35 expired. Notify the listener that a new frame was received.
		case STATE_RX_RCV:
			xNeedPoll = modbus->driver->xMBPortEventPost(modbus->driver, EV_FRAME_RECEIVED );
			break;

			/* An error occured while receiving the frame. */
		case STATE_RX_ERROR:
			break;

			/* Function called in an illegal state. */
//    default:
			//assert( ( eRcvState == STATE_RX_INIT ) ||  ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_ERROR ) );
	}

	modbus->driver->vMBPortTimersDisable(modbus->driver);
	//vMBPortTimersDisable();
	modbus->eRcvState = STATE_RX_IDLE;

	return xNeedPoll;
}
