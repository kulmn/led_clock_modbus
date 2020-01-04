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
 * File: $Id: mbport.h,v 1.19 2010/06/06 13:54:40 wolti Exp $
 */

#ifndef _MB_PORT_H
#define _MB_PORT_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/* ----------------------- Type definitions ---------------------------------*/

typedef enum
{
    EV_READY,                   /*!< Startup finished. */
    EV_FRAME_RECEIVED,          /*!< Frame received. */
    EV_EXECUTE,                 /*!< Execute function. */
    EV_FRAME_SENT               /*!< Frame sent. */
} eMBEventType;

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


/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
	STATE_RX_INIT, /*!< Receiver is in initial state. */
	STATE_RX_IDLE, /*!< Receiver is in idle state. */
	STATE_RX_RCV, /*!< Frame is beeing received. */
	STATE_RX_ERROR /*!< If the frame is invalid. */
} eMBRcvState;

typedef enum
{
	STATE_TX_IDLE, /*!< Transmitter is in idle state. */
	STATE_TX_XMIT /*!< Transmitter is in transfer state. */
} eMBSndState;


struct Modbus_Driver;
typedef struct Modbus_Driver Modbus_Interface;

/* Hardware abstraction layer */
struct Modbus_Driver
{
	bool (*xMBPortEventInit)(Modbus_Interface *interface);
	bool (*xMBPortEventPost)(Modbus_Interface *interface, eMBEventType eEvent);
	bool (*xMBPortEventGet)( Modbus_Interface *interface, /*@out@ */eMBEventType *eEvent);

	bool (*xMBPortSerialInit)(Modbus_Interface *interface);
	void (*vMBPortClose)(Modbus_Interface *interface);
	void (*vMBPortSerialEnable)(Modbus_Interface *interface, bool xRxEnable, bool xTxEnable);
	bool (*xMBPortSerialGetByte)(Modbus_Interface *interface, uint8_t *pucByte);
//	bool (*xMBPortSerialPutByte)(Modbus_Interface *interface, uint8_t ucByte);
	bool (*xMBPortSerialSendBuf)(Modbus_Interface *interface, uint8_t *snd_buf, uint8_t len);

	bool (*xMBPortTimersInit)(Modbus_Interface *interface, uint16_t usTimeOut50us);
	void (*vMBPortTimersEnable)(Modbus_Interface *interface);
	void (*vMBPortTimersDisable)(Modbus_Interface *interface);

};

//typedef eMBErrorCode (*eMBRegInputCB1) (uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs);
//typedef eMBErrorCode (*eMBRegHoldingCB1) ( uint8_t * pucRegBuffer, uint16_t usAddress,uint16_t usNRegs, eMBRegisterMode eMode );

/* HD44780 control structure */
typedef struct
{
	Modbus_Interface 		*driver;
	uint8_t					ucMBAddress;
	eMBSndState 			eSndState;
	eMBRcvState 			eRcvState;
} MODBUS;


/* ----------------------- Callback for the protocol stack ------------------*/
/*!
 * \brief Callback function for the porting layer when a new byte is
 *   available.
 *
 * Depending upon the mode this callback function is used by the RTU or
 * ASCII transmission layers. In any case a call to xMBPortSerialGetByte()
 * must immediately return a new character.
 *
 * \return <code>TRUE</code> if a event was posted to the queue because
 *   a new byte was received. The port implementation should wake up the
 *   tasks which are currently blocked on the eventqueue.
 */
extern          bool( *pxMBFrameCBByteReceived ) ( MODBUS *modbus );
extern          void( *pxMBFrameCBTransmitterEmpty ) ( MODBUS *modbus );
extern          bool( *pxMBPortCBTimerExpired ) ( MODBUS *modbus );

/* ----------------------- TCP port functions -------------------------------*/
bool            xMBTCPPortInit( uint16_t usTCPPort );
void            vMBTCPPortClose( void );
void            vMBTCPPortDisable( void );
bool            xMBTCPPortGetRequest( uint8_t **ppucMBTCPFrame, uint16_t * usTCPLength );
bool            xMBTCPPortSendResponse( const uint8_t *pucMBTCPFrame, uint16_t usTCPLength );

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
