/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 */


#include "mbfunc.h"



eMBException prveMBError2Exception(eMBErrorCode eErrorCode)
{
	eMBException eStatus;

	switch (eErrorCode)
	{
		case MB_ENOERR:
			eStatus = MB_EX_NONE;
			break;
		case MB_ENOREG:
			eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
			break;
		case MB_ETIMEDOUT:
			eStatus = MB_EX_SLAVE_BUSY;
			break;
		default:
			eStatus = MB_EX_SLAVE_DEVICE_FAILURE;
			break;
	}
	return eStatus;
}

/* ----------------------- Static functions ---------------------------------*/
eMBException prveMBError2Exception(eMBErrorCode eErrorCode);


#if MB_FUNC_READ_INPUT_ENABLED > 0
eMBException eMBFuncReadInputRegister(uint8_t *pucFrame, uint16_t *usLen)
{
	uint16_t usRegAddress;
	uint16_t usRegCount;
	uint8_t *pucFrameCur;

	eMBException eStatus = MB_EX_NONE;
	eMBErrorCode eRegStatus;

	if (*usLen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN))
	{
		usRegAddress = (uint16_t) (pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8);
		usRegAddress |= (uint16_t) (pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1]);
		//       usRegAddress++;

		usRegCount = (uint16_t) (pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF] << 8);
		usRegCount |= (uint16_t) (pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF + 1]);

		/* Check if the number of registers to read is valid. If not
		 * return Modbus illegal data value exception.
		 */
		if ((usRegCount >= 1) && (usRegCount < MB_PDU_FUNC_READ_REGCNT_MAX))
		{
			/* Set the current PDU data pointer to the beginning. */
			pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
			*usLen = MB_PDU_FUNC_OFF;

			/* First byte contains the function code. */
			*pucFrameCur++ = MB_FUNC_READ_INPUT_REGISTER;
			*usLen += 1;

			/* Second byte in the response contain the number of bytes. */
			*pucFrameCur++ = (uint8_t) (usRegCount * 2);
			*usLen += 1;

			eRegStatus = eMBRegInputCB(pucFrameCur, usRegAddress, usRegCount );

			/* If an error occured convert it into a Modbus exception. */
			if (eRegStatus != MB_ENOERR)
			{
				eStatus = prveMBError2Exception(eRegStatus );
			} else
			{
				*usLen += usRegCount * 2;
			}
		} else
		{
			eStatus = MB_EX_ILLEGAL_DATA_VALUE;
		}
	} else
	{
		/* Can't be a valid read input register request because the length
		 * is incorrect. */
		eStatus = MB_EX_ILLEGAL_DATA_VALUE;
	}
	return eStatus;
}
#endif


#if MB_FUNC_WRITE_HOLDING_ENABLED > 0

eMBException eMBFuncWriteHoldingRegister(uint8_t *pucFrame, uint16_t *usLen)
{
	uint16_t usRegAddress;
	eMBException eStatus = MB_EX_NONE;
	eMBErrorCode eRegStatus;

	if (*usLen == ( MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN))
	{
		usRegAddress = (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8);
		usRegAddress |= (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1]);
		//usRegAddress++;

		/* Make callback to update the value. */
		eRegStatus = eMBRegHoldingCB(&pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF], usRegAddress, 1, MB_REG_WRITE );

		/* If an error occured convert it into a Modbus exception. */
		if (eRegStatus != MB_ENOERR)
		{
			eStatus = prveMBError2Exception(eRegStatus );
		}
	} else
	{
		/* Can't be a valid request because the length is incorrect. */
		eStatus = MB_EX_ILLEGAL_DATA_VALUE;
	}
	return eStatus;
}
#endif

#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
eMBException eMBFuncWriteMultipleHoldingRegister(uint8_t *pucFrame, uint16_t *usLen)
{
	uint16_t usRegAddress;
	uint16_t usRegCount;
	uint8_t ucRegByteCount;

	eMBException eStatus = MB_EX_NONE;
	eMBErrorCode eRegStatus;

	if (*usLen >= ( MB_PDU_FUNC_WRITE_MUL_SIZE_MIN + MB_PDU_SIZE_MIN))
	{
		usRegAddress = (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8);
		usRegAddress |= (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1]);
		//usRegAddress++;

		usRegCount = (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF] << 8);
		usRegCount |= (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF + 1]);

		ucRegByteCount = pucFrame[MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

		if ((usRegCount >= 1) && (usRegCount <= MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX) && (ucRegByteCount == (uint8_t) (2 * usRegCount)))
		{
			/* Make callback to update the register values. */
			eRegStatus = eMBRegHoldingCB(&pucFrame[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF], usRegAddress, usRegCount, MB_REG_WRITE );

			/* If an error occured convert it into a Modbus exception. */
			if (eRegStatus != MB_ENOERR)
			{
				eStatus = prveMBError2Exception(eRegStatus );
			} else
			{
				/* The response contains the function code, the starting
				 * address and the quantity of registers. We reuse the
				 * old values in the buffer because they are still valid.
				 */
				*usLen = MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
			}
		} else
		{
			eStatus = MB_EX_ILLEGAL_DATA_VALUE;
		}
	} else
	{
		/* Can't be a valid request because the length is incorrect. */
		eStatus = MB_EX_ILLEGAL_DATA_VALUE;
	}
	return eStatus;
}
#endif

#if MB_FUNC_READ_HOLDING_ENABLED > 0

eMBException eMBFuncReadHoldingRegister(uint8_t *pucFrame, uint16_t *usLen)
{
	uint16_t usRegAddress;
	uint16_t usRegCount;
	uint8_t *pucFrameCur;

	eMBException eStatus = MB_EX_NONE;
	eMBErrorCode eRegStatus;

	if (*usLen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN))
	{
		usRegAddress = (uint16_t) (pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8);
		usRegAddress |= (uint16_t) (pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1]);
		//usRegAddress++;

		usRegCount = (uint16_t) (pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF] << 8);
		usRegCount = (uint16_t) (pucFrame[MB_PDU_FUNC_READ_REGCNT_OFF + 1]);

		/* Check if the number of registers to read is valid. If not
		 * return Modbus illegal data value exception.
		 */
		if ((usRegCount >= 1) && (usRegCount <= MB_PDU_FUNC_READ_REGCNT_MAX))
		{
			/* Set the current PDU data pointer to the beginning. */
			pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
			*usLen = MB_PDU_FUNC_OFF;

			/* First byte contains the function code. */
			*pucFrameCur++ = MB_FUNC_READ_HOLDING_REGISTER;
			*usLen += 1;

			/* Second byte in the response contain the number of bytes. */
			*pucFrameCur++ = (uint8_t) (usRegCount * 2);
			*usLen += 1;

			/* Make callback to fill the buffer. */
			eRegStatus = eMBRegHoldingCB(pucFrameCur, usRegAddress, usRegCount, MB_REG_READ );
			/* If an error occured convert it into a Modbus exception. */
			if (eRegStatus != MB_ENOERR)
			{
				eStatus = prveMBError2Exception(eRegStatus );
			} else
			{
				*usLen += usRegCount * 2;
			}
		} else
		{
			eStatus = MB_EX_ILLEGAL_DATA_VALUE;
		}
	} else
	{
		/* Can't be a valid request because the length is incorrect. */
		eStatus = MB_EX_ILLEGAL_DATA_VALUE;
	}
	return eStatus;
}

#endif

#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
eMBException eMBFuncReadWriteMultipleHoldingRegister(uint8_t *pucFrame, uint16_t *usLen)
{
	uint16_t usRegReadAddress, usRegReadCount;
	uint16_t usRegWriteAddress, usRegWriteCount;
	uint8_t ucRegWriteByteCount;
	uint8_t *pucFrameCur;

	eMBException eStatus = MB_EX_NONE;
	eMBErrorCode eRegStatus;

	if (*usLen >= ( MB_PDU_FUNC_READWRITE_SIZE_MIN + MB_PDU_SIZE_MIN))
	{
		usRegReadAddress = (uint16_t) (pucFrame[MB_PDU_FUNC_READWRITE_READ_ADDR_OFF] << 8U);
		usRegReadAddress |= (uint16_t) (pucFrame[MB_PDU_FUNC_READWRITE_READ_ADDR_OFF + 1]);
		//usRegReadAddress++;

		usRegReadCount = (uint16_t) (pucFrame[MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF] << 8U);
		usRegReadCount |= (uint16_t) (pucFrame[MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF + 1]);

		usRegWriteAddress = (uint16_t) (pucFrame[MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF] << 8U);
		usRegWriteAddress |= (uint16_t) (pucFrame[MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF + 1]);
		usRegWriteAddress++;

		usRegWriteCount = (uint16_t) (pucFrame[MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF] << 8U);
		usRegWriteCount |= (uint16_t) (pucFrame[MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF + 1]);

		ucRegWriteByteCount = pucFrame[MB_PDU_FUNC_READWRITE_BYTECNT_OFF];

		if ((usRegReadCount >= 1) && (usRegReadCount <= 0x7D) && (usRegWriteCount >= 1) && (usRegWriteCount <= 0x79) && ((2 * usRegWriteCount) == ucRegWriteByteCount))
		{
			/* Make callback to update the register values. */
			eRegStatus = eMBRegHoldingCB(&pucFrame[MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF], usRegWriteAddress, usRegWriteCount, MB_REG_WRITE );

			if (eRegStatus == MB_ENOERR)
			{
				/* Set the current PDU data pointer to the beginning. */
				pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
				*usLen = MB_PDU_FUNC_OFF;

				/* First byte contains the function code. */
				*pucFrameCur++ = MB_FUNC_READWRITE_MULTIPLE_REGISTERS;
				*usLen += 1;

				/* Second byte in the response contain the number of bytes. */
				*pucFrameCur++ = (uint8_t) (usRegReadCount * 2);
				*usLen += 1;

				/* Make the read callback. */
				eRegStatus = eMBRegHoldingCB(pucFrameCur, usRegReadAddress, usRegReadCount, MB_REG_READ );
				if (eRegStatus == MB_ENOERR)
				{
					*usLen += 2 * usRegReadCount;
				}
			}
			if (eRegStatus != MB_ENOERR)
			{
				eStatus = prveMBError2Exception(eRegStatus );
			}
		} else
		{
			eStatus = MB_EX_ILLEGAL_DATA_VALUE;
		}
	}
	return eStatus;
}
#endif


#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
eMBException eMBFuncReadDiscreteInputs(uint8_t *pucFrame, uint16_t *usLen)
{
	uint16_t usRegAddress;
	uint16_t usDiscreteCnt;
	uint8_t ucNBytes;
	uint8_t *pucFrameCur;

	eMBException eStatus = MB_EX_NONE;
	eMBErrorCode eRegStatus;

	if (*usLen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN))
	{
		usRegAddress = (uint16_t) (pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8);
		usRegAddress |= (uint16_t) (pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1]);
//		usRegAddress++;

		usDiscreteCnt = (uint16_t) (pucFrame[MB_PDU_FUNC_READ_DISCCNT_OFF] << 8);
		usDiscreteCnt |= (uint16_t) (pucFrame[MB_PDU_FUNC_READ_DISCCNT_OFF + 1]);

		/* Check if the number of registers to read is valid. If not
		 * return Modbus illegal data value exception.
		 */
		if ((usDiscreteCnt >= 1) && (usDiscreteCnt < MB_PDU_FUNC_READ_DISCCNT_MAX))
		{
			/* Set the current PDU data pointer to the beginning. */
			pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
			*usLen = MB_PDU_FUNC_OFF;

			/* First byte contains the function code. */
			*pucFrameCur++ = MB_FUNC_READ_DISCRETE_INPUTS;
			*usLen += 1;

			/* Test if the quantity of coils is a multiple of 8. If not last
			 * byte is only partially field with unused coils set to zero. */
			if ((usDiscreteCnt & 0x0007) != 0)
			{
				ucNBytes = (uint8_t) (usDiscreteCnt / 8 + 1);
			} else
			{
				ucNBytes = (uint8_t) (usDiscreteCnt / 8);
			}
			*pucFrameCur++ = ucNBytes;
			*usLen += 1;

			eRegStatus = eMBRegDiscreteCB(pucFrameCur, usRegAddress, usDiscreteCnt );

			/* If an error occured convert it into a Modbus exception. */
			if (eRegStatus != MB_ENOERR)
			{
				eStatus = prveMBError2Exception(eRegStatus );
			} else
			{
				/* The response contains the function code, the starting address
				 * and the quantity of registers. We reuse the old values in the
				 * buffer because they are still valid. */
				*usLen += ucNBytes;
			}
		} else
		{
			eStatus = MB_EX_ILLEGAL_DATA_VALUE;
		}
	} else
	{
		/* Can't be a valid read coil register request because the length
		 * is incorrect. */
		eStatus = MB_EX_ILLEGAL_DATA_VALUE;
	}
	return eStatus;
}
#endif


#if MB_FUNC_READ_COILS_ENABLED > 0
eMBException eMBFuncReadCoils(uint8_t *pucFrame, uint16_t *usLen)
{
	uint16_t usRegAddress;
	uint16_t usCoilCount;
	uint8_t ucNBytes;
	uint8_t *pucFrameCur;

	eMBException eStatus = MB_EX_NONE;
	eMBErrorCode eRegStatus;

	if (*usLen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN))
	{
		usRegAddress = (uint16_t) (pucFrame[MB_PDU_FUNC_READ_ADDR_OFF] << 8);
		usRegAddress |= (uint16_t) (pucFrame[MB_PDU_FUNC_READ_ADDR_OFF + 1]);
//		usRegAddress++;

		usCoilCount = (uint16_t) (pucFrame[MB_PDU_FUNC_READ_COILCNT_OFF] << 8);
		usCoilCount |= (uint16_t) (pucFrame[MB_PDU_FUNC_READ_COILCNT_OFF + 1]);

		/* Check if the number of registers to read is valid. If not
		 * return Modbus illegal data value exception.
		 */
		if ((usCoilCount >= 1) && (usCoilCount < MB_PDU_FUNC_READ_COILCNT_MAX))
		{
			/* Set the current PDU data pointer to the beginning. */
			pucFrameCur = &pucFrame[MB_PDU_FUNC_OFF];
			*usLen = MB_PDU_FUNC_OFF;

			/* First byte contains the function code. */
			*pucFrameCur++ = MB_FUNC_READ_COILS;
			*usLen += 1;

			/* Test if the quantity of coils is a multiple of 8. If not last
			 * byte is only partially field with unused coils set to zero. */
			if ((usCoilCount & 0x0007) != 0)
			{
				ucNBytes = (uint8_t) (usCoilCount / 8 + 1);
			} else
			{
				ucNBytes = (uint8_t) (usCoilCount / 8);
			}
			*pucFrameCur++ = ucNBytes;
			*usLen += 1;

			eRegStatus = eMBRegCoilsCB(pucFrameCur, usRegAddress, usCoilCount, MB_REG_READ );

			/* If an error occured convert it into a Modbus exception. */
			if (eRegStatus != MB_ENOERR)
			{
				eStatus = prveMBError2Exception(eRegStatus );
			} else
			{
				/* The response contains the function code, the starting address
				 * and the quantity of registers. We reuse the old values in the
				 * buffer because they are still valid. */
				*usLen += ucNBytes;
				;
			}
		} else
		{
			eStatus = MB_EX_ILLEGAL_DATA_VALUE;
		}
	} else
	{
		/* Can't be a valid read coil register request because the length
		 * is incorrect. */
		eStatus = MB_EX_ILLEGAL_DATA_VALUE;
	}
	return eStatus;
}
#endif

#if MB_FUNC_WRITE_COIL_ENABLED > 0
eMBException eMBFuncWriteCoil(uint8_t *pucFrame, uint16_t *usLen)
{
	uint16_t usRegAddress;
	uint8_t ucBuf[2];

	eMBException eStatus = MB_EX_NONE;
	eMBErrorCode eRegStatus;

	if (*usLen == ( MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN))
	{
		usRegAddress = (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8);
		usRegAddress |= (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1]);
//		usRegAddress++;

		if ((pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF + 1] == 0x00) && ((pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF) || (pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0x00)))
		{
			ucBuf[1] = 0;
			if (pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF)
			{
				ucBuf[0] = 1;
			} else
			{
				ucBuf[0] = 0;
			}
			eRegStatus = eMBRegCoilsCB(&ucBuf[0], usRegAddress, 1, MB_REG_WRITE );

			/* If an error occured convert it into a Modbus exception. */
			if (eRegStatus != MB_ENOERR)
			{
				eStatus = prveMBError2Exception(eRegStatus );
			}
		} else
		{
			eStatus = MB_EX_ILLEGAL_DATA_VALUE;
		}
	} else
	{
		/* Can't be a valid write coil register request because the length
		 * is incorrect. */
		eStatus = MB_EX_ILLEGAL_DATA_VALUE;
	}
	return eStatus;
}
#endif

#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
eMBException eMBFuncWriteMultipleCoils(uint8_t *pucFrame, uint16_t *usLen)
{
	uint16_t usRegAddress;
	uint16_t usCoilCnt;
	uint8_t ucByteCount;
	uint8_t ucByteCountVerify;

	eMBException eStatus = MB_EX_NONE;
	eMBErrorCode eRegStatus;

	if (*usLen > ( MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN))
	{
		usRegAddress = (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8);
		usRegAddress |= (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1]);
//		usRegAddress++;

		usCoilCnt = (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF] << 8);
		usCoilCnt |= (uint16_t) (pucFrame[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF + 1]);

		ucByteCount = pucFrame[MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

		/* Compute the number of expected bytes in the request. */
		if ((usCoilCnt & 0x0007) != 0)
		{
			ucByteCountVerify = (uint8_t) (usCoilCnt / 8 + 1);
		} else
		{
			ucByteCountVerify = (uint8_t) (usCoilCnt / 8);
		}

		if ((usCoilCnt >= 1) && (usCoilCnt <= MB_PDU_FUNC_WRITE_MUL_COILCNT_MAX) && (ucByteCountVerify == ucByteCount))
		{
			eRegStatus = eMBRegCoilsCB(&pucFrame[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF], usRegAddress, usCoilCnt, MB_REG_WRITE );

			/* If an error occured convert it into a Modbus exception. */
			if (eRegStatus != MB_ENOERR)
			{
				eStatus = prveMBError2Exception(eRegStatus );
			} else
			{
				/* The response contains the function code, the starting address
				 * and the quantity of registers. We reuse the old values in the
				 * buffer because they are still valid. */
				*usLen = MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
			}
		} else
		{
			eStatus = MB_EX_ILLEGAL_DATA_VALUE;
		}
	} else
	{
		/* Can't be a valid write coil register request because the length
		 * is incorrect. */
		eStatus = MB_EX_ILLEGAL_DATA_VALUE;
	}
	return eStatus;
}
#endif

#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
/* ----------------------- Static variables ---------------------------------*/
static uint8_t ucMBSlaveID[MB_FUNC_OTHER_REP_SLAVEID_BUF];
static uint16_t usMBSlaveIDLen;

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode eMBSetSlaveID(uint8_t ucSlaveID, bool xIsRunning, uint8_t const *pucAdditional, uint16_t usAdditionalLen)
{
	eMBErrorCode eStatus = MB_ENOERR;

	/* the first byte and second byte in the buffer is reserved for
	 * the parameter ucSlaveID and the running flag. The rest of
	 * the buffer is available for additional data. */
	if (usAdditionalLen + 2 < MB_FUNC_OTHER_REP_SLAVEID_BUF)
	{
		usMBSlaveIDLen = 0;
		ucMBSlaveID[usMBSlaveIDLen++] = ucSlaveID;
		ucMBSlaveID[usMBSlaveIDLen++] = (uint8_t) (
		                xIsRunning ? 0xFF : 0x00);
		if (usAdditionalLen > 0)
		{
			memcpy(&ucMBSlaveID[usMBSlaveIDLen], pucAdditional, (size_t) usAdditionalLen );
			usMBSlaveIDLen += usAdditionalLen;
		}
	} else
	{
		eStatus = MB_ENORES;
	}
	return eStatus;
}

eMBException eMBFuncReportSlaveID(uint8_t *pucFrame, uint16_t *usLen)
{
	memcpy(&pucFrame[MB_PDU_DATA_OFF], &ucMBSlaveID[0], (size_t) usMBSlaveIDLen );
	*usLen = (uint16_t) ( MB_PDU_DATA_OFF + usMBSlaveIDLen);
	return MB_EX_NONE;
}
#endif

