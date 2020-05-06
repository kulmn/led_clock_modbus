/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
*/

#ifndef _MB_FUNC_H
#define _MB_FUNC_H

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "stdint.h"
#include "string.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"

/*!
 * Constants which defines the format of a modbus frame. The example is
 * shown for a Modbus RTU/ASCII frame. Note that the Modbus PDU is not
 * dependent on the underlying transport.
 *
 * <code>
 * <------------------------ MODBUS SERIAL LINE PDU (1) ------------------->
 *              <----------- MODBUS PDU (1') ---------------->
 *  +-----------+---------------+----------------------------+-------------+
 *  | Address   | Function Code | Data                       | CRC/LRC     |
 *  +-----------+---------------+----------------------------+-------------+
 *  |           |               |                                   |
 * (2)        (3/2')           (3')                                (4)
 *
 * (1)  ... MB_SER_PDU_SIZE_MAX = 256
 * (2)  ... MB_SER_PDU_ADDR_OFF = 0
 * (3)  ... MB_SER_PDU_PDU_OFF  = 1
 * (4)  ... MB_SER_PDU_SIZE_CRC = 2
 *
 * (1') ... MB_PDU_SIZE_MAX     = 253
 * (2') ... MB_PDU_FUNC_OFF     = 0
 * (3') ... MB_PDU_DATA_OFF     = 1
 * </code>
 */

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_SIZE_MAX     253 /*!< Maximum size of a PDU. */
#define MB_PDU_SIZE_MIN     1   /*!< Function Code */
#define MB_PDU_FUNC_OFF     0   /*!< Offset of function code in PDU. */
#define MB_PDU_DATA_OFF     1   /*!< Offset for response data in PDU. */


#define MB_PDU_FUNC_READ_ADDR_OFF					( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_READ_REGCNT_OFF				( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_SIZE						( 4 )
#define MB_PDU_FUNC_READ_REGCNT_MAX				( 0x007D )
#define MB_PDU_FUNC_READ_RSP_BYTECNT_OFF			( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_READ_DISCCNT_OFF				( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_DISCCNT_MAX				( 0x07D0 )
#define MB_PDU_FUNC_READ_COILCNT_OFF				( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_COILCNT_MAX				( 0x07D0 )

#define MB_PDU_FUNC_WRITE_ADDR_OFF				( MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_WRITE_VALUE_OFF				( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_SIZE						( 4 )

#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF			( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF			( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF		( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_FUNC_WRITE_MUL_VALUES_OFF			( MB_PDU_DATA_OFF + 5 )
#define MB_PDU_FUNC_WRITE_MUL_SIZE_MIN			( 5 )
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX		( 0x0078 )
#define MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF		( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_MUL_COILCNT_MAX		( 0x07B0 )

#define MB_PDU_FUNC_READWRITE_READ_ADDR_OFF     ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF   ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF    ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF  ( MB_PDU_DATA_OFF + 6 )
#define MB_PDU_FUNC_READWRITE_BYTECNT_OFF       ( MB_PDU_DATA_OFF + 8 )
#define MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF  ( MB_PDU_DATA_OFF + 9 )
#define MB_PDU_FUNC_READWRITE_SIZE_MIN          ( 9 )

/* ----------------------- Defines ------------------------------------------*/
#define MB_ADDRESS_BROADCAST    ( 0 )   /*! Modbus broadcast address. */
#define MB_ADDRESS_MIN          ( 1 )   /*! Smallest possible slave address. */
#define MB_ADDRESS_MAX          ( 247 ) /*! Biggest possible slave address. */
#define MB_FUNC_NONE                          (  0 )
#define MB_FUNC_READ_COILS                    (  1 )
#define MB_FUNC_READ_DISCRETE_INPUTS          (  2 )
#define MB_FUNC_WRITE_SINGLE_COIL             (  5 )
#define MB_FUNC_WRITE_MULTIPLE_COILS          ( 15 )
#define MB_FUNC_READ_HOLDING_REGISTER         (  3 )
#define MB_FUNC_READ_INPUT_REGISTER           (  4 )
#define MB_FUNC_WRITE_REGISTER                (  6 )
#define MB_FUNC_WRITE_MULTIPLE_REGISTERS      ( 16 )
#define MB_FUNC_READWRITE_MULTIPLE_REGISTERS  ( 23 )
#define MB_FUNC_DIAG_READ_EXCEPTION           (  7 )
#define MB_FUNC_DIAG_DIAGNOSTIC               (  8 )
#define MB_FUNC_DIAG_GET_COM_EVENT_CNT        ( 11 )
#define MB_FUNC_DIAG_GET_COM_EVENT_LOG        ( 12 )
#define MB_FUNC_OTHER_REPORT_SLAVEID          ( 17 )
#define MB_FUNC_ERROR                         ( 128 )





#if MB_FUNC_OTHER_REP_SLAVEID_BUF > 0
    eMBException eMBFuncReportSlaveID( uint8_t * pucFrame, uint16_t * usLen );
#endif

#if MB_FUNC_READ_INPUT_ENABLED > 0
eMBException    eMBFuncReadInputRegister( uint8_t * pucFrame, uint16_t * usLen );
#endif

#if MB_FUNC_READ_HOLDING_ENABLED > 0
eMBException    eMBFuncReadHoldingRegister( uint8_t * pucFrame, uint16_t * usLen );
#endif

#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
eMBException    eMBFuncWriteHoldingRegister( uint8_t * pucFrame, uint16_t * usLen );
#endif

#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
eMBException    eMBFuncWriteMultipleHoldingRegister( uint8_t * pucFrame, uint16_t * usLen );
#endif

#if MB_FUNC_READ_COILS_ENABLED > 0
eMBException    eMBFuncReadCoils( uint8_t * pucFrame, uint16_t * usLen );
#endif

#if MB_FUNC_WRITE_COIL_ENABLED > 0
eMBException    eMBFuncWriteCoil( uint8_t * pucFrame, uint16_t * usLen );
#endif

#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
eMBException    eMBFuncWriteMultipleCoils( uint8_t * pucFrame, uint16_t * usLen );
#endif

#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
eMBException    eMBFuncReadDiscreteInputs( uint8_t * pucFrame, uint16_t * usLen );
#endif

#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
eMBException    eMBFuncReadWriteMultipleHoldingRegister( uint8_t * pucFrame, uint16_t * usLen );
#endif

#endif
