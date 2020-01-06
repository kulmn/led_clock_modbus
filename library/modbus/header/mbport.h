/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 */

#ifndef _MB_PORT_H
#define _MB_PORT_H

#include <stdbool.h>
#include "FreeRTOS.h"

#define ENTER_CRITICAL_SECTION( )   portENTER_CRITICAL()
#define EXIT_CRITICAL_SECTION( )    portEXIT_CRITICAL()



typedef enum
{
	STATE_ENABLED, STATE_DISABLED, STATE_NOT_INITIALIZED
} eMBStateType;


/* ----------------------- Type definitions ---------------------------------*/

/*! \ingroup modbus
 * \brief Errorcodes used by all function in the protocol stack.
 */
typedef enum
{
	MB_ENOERR, /*!< no error. */
	MB_ENOREG, /*!< illegal register address. */
	MB_EINVAL, /*!< illegal argument. */
	MB_EPORTERR, /*!< porting layer error. */
	MB_ENORES, /*!< insufficient resources. */
	MB_EIO, /*!< I/O error. */
	MB_EILLSTATE, /*!< protocol stack in illegal state. */
	MB_ETIMEDOUT /*!< timeout error occurred. */
} eMBErrorCode;


struct Modbus_Driver;
typedef struct Modbus_Driver Modbus_Interface;



struct Modbus_Driver
{
	bool (*xMBPortFrameReceived)(Modbus_Interface *interface);
	bool (*xMBPortInit)(Modbus_Interface *interface);
	void (*xMBPortStart)(Modbus_Interface *interface);
	void (*xMBPortStop)(Modbus_Interface *interface);
	bool (*xMBPortSerialSendBuf)(Modbus_Interface *interface, uint8_t *snd_buf, uint8_t len);
};

//typedef eMBErrorCode (*eMBRegInputCB) (uint8_t *pucRegBuffer, uint16_t usAddress, uint16_t usNRegs);


typedef enum
{
MB_EX_NONE = 0x00,
MB_EX_ILLEGAL_FUNCTION = 0x01,
MB_EX_ILLEGAL_DATA_ADDRESS = 0x02,
MB_EX_ILLEGAL_DATA_VALUE = 0x03,
MB_EX_SLAVE_DEVICE_FAILURE = 0x04,
MB_EX_ACKNOWLEDGE = 0x05,
MB_EX_SLAVE_BUSY = 0x06,
MB_EX_MEMORY_PARITY_ERROR = 0x08,
MB_EX_GATEWAY_PATH_FAILED = 0x0A,
MB_EX_GATEWAY_TGT_FAILED = 0x0B
} eMBException;

/* MODBUS control structure */
typedef struct
{
	Modbus_Interface 		*driver;
	uint8_t					ucMBAddress;
	eMBStateType			eMBState;
	eMBException			eException;
	uint8_t					*ucMBFrame;
	uint8_t					ucRcvAddress;
	uint8_t					ucFunctionCode;
	uint16_t					usLength;
	uint8_t					*packet_buf;
	uint8_t					packet_size;
//	eMBRegInputCB			reg_input_fn;
} MODBUS;



typedef         eMBException( *pxMBFunctionHandler ) ( uint8_t * pucFrame, uint16_t * pusLength );

typedef struct
{
    uint8_t           ucFunctionCode;
    pxMBFunctionHandler pxHandler;
} xMBFunctionHandler;

#endif
