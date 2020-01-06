/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 */

#ifndef _MB_RTU_H
#define _MB_RTU_H


#include "stdint.h"
#include "mbport.h"


/* ----------------------- Defines ------------------------------------------*/
#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

eMBErrorCode eMBRTUInit(MODBUS *modbus);
eMBErrorCode eMBRTUReceive(MODBUS *modbus, uint8_t *pucRcvAddress, uint8_t **pucFrame, uint16_t *pusLength);
eMBErrorCode eMBRTUSend(MODBUS *modbus, const uint8_t *pucFrame, uint16_t usLength);
uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen );


#endif
