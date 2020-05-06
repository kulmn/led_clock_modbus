/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 */

#include "mb.h"


/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
                { MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister },
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
                { MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister },
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
                { MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBFuncWriteMultipleHoldingRegister },
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
                { MB_FUNC_WRITE_REGISTER, eMBFuncWriteHoldingRegister },
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, eMBFuncReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, eMBFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, eMBFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, eMBFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, eMBFuncReadDiscreteInputs},
#endif
                };

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode eMBInit(MODBUS *modbus, uint8_t ucSlaveAddress)
{
	eMBErrorCode eStatus = MB_ENOERR;

	modbus->eMBState = STATE_NOT_INITIALIZED;

	/* check preconditions */
	if ((ucSlaveAddress == MB_ADDRESS_BROADCAST) || (ucSlaveAddress < MB_ADDRESS_MIN) || (ucSlaveAddress > MB_ADDRESS_MAX))
	{
		eStatus = MB_EINVAL;
	} else
	{
		modbus->ucMBAddress = ucSlaveAddress;
		eStatus = eMBRTUInit(modbus );

		if (eStatus == MB_ENOERR)
		{
			modbus->eMBState = STATE_DISABLED;
		}
	}
	return eStatus;
}



eMBErrorCode eMBRegisterCB(uint8_t ucFunctionCode, pxMBFunctionHandler pxHandler)
{
	int i;
	eMBErrorCode eStatus;

	if ((0 < ucFunctionCode) && (ucFunctionCode <= 127))
	{
		ENTER_CRITICAL_SECTION( );
		if (pxHandler != NULL)
		{
			for (i = 0; i < MB_FUNC_HANDLERS_MAX; i++)
			{
				if ((xFuncHandlers[i].pxHandler == NULL) || (xFuncHandlers[i].pxHandler == pxHandler))
				{
					xFuncHandlers[i].ucFunctionCode = ucFunctionCode;
					xFuncHandlers[i].pxHandler = pxHandler;
					break;
				}
			}
			eStatus = (i != MB_FUNC_HANDLERS_MAX) ? MB_ENOERR : MB_ENORES;
		} else
		{
			for (i = 0; i < MB_FUNC_HANDLERS_MAX; i++)
			{
				if (xFuncHandlers[i].ucFunctionCode == ucFunctionCode)
				{
					xFuncHandlers[i].ucFunctionCode = 0;
					xFuncHandlers[i].pxHandler = NULL;
					break;
				}
			}
			/* Remove can't fail. */
			eStatus = MB_ENOERR;
		}
		EXIT_CRITICAL_SECTION( );
	} else
	{
		eStatus = MB_EINVAL;
	}
	return eStatus;
}


eMBErrorCode eMBEnable(MODBUS *modbus)
{
	eMBErrorCode eStatus = MB_ENOERR;

	if (modbus->eMBState == STATE_DISABLED)
	{
		/* Activate the protocol stack. */
		modbus->driver->xMBPortStart(modbus->driver);
		modbus->eMBState = STATE_ENABLED;
	} else
	{
		eStatus = MB_EILLSTATE;
	}
	return eStatus;
}

eMBErrorCode eMBDisable(MODBUS *modbus)
{
	eMBErrorCode eStatus;

	if (modbus->eMBState == STATE_ENABLED)
	{
		modbus->driver->xMBPortStop(modbus->driver);
		modbus->eMBState = STATE_DISABLED;
		eStatus = MB_ENOERR;
	} else
		if (modbus->eMBState == STATE_DISABLED)
		{
			eStatus = MB_ENOERR;
		} else
		{
			eStatus = MB_EILLSTATE;
		}
	return eStatus;
}

eMBErrorCode eMBPoll(MODBUS *modbus)
{
	eMBErrorCode eStatus = MB_ENOERR;

	/* Check if the protocol stack is ready. */
	if (modbus->eMBState != STATE_ENABLED)
	{
		return MB_EILLSTATE;
	}

	// Check if there is a event available. If not return control to caller. Otherwise we will handle the event.
	if (modbus->driver->xMBPortFrameReceived(modbus->driver ))
	{
		eStatus = eMBRTUReceive(modbus, &modbus->ucRcvAddress, &modbus->ucMBFrame, &modbus->usLength );
		if (eStatus == MB_ENOERR)
		{
			// Check if the frame is for us. If not ignore the frame.
			if ((modbus->ucRcvAddress == modbus->ucMBAddress) || (modbus->ucRcvAddress == MB_ADDRESS_BROADCAST))
			{
				modbus->ucFunctionCode = modbus->ucMBFrame[MB_PDU_FUNC_OFF];
				modbus->eException = MB_EX_ILLEGAL_FUNCTION;
				for (uint8_t i = 0; i < MB_FUNC_HANDLERS_MAX;
				                i++)
				{
					// No more function handlers registered. Abort.
					if (xFuncHandlers[i].ucFunctionCode == 0)
					{
						break;
					} else
						if (xFuncHandlers[i].ucFunctionCode == modbus->ucFunctionCode)
						{
							modbus->eException = xFuncHandlers[i].pxHandler(modbus->ucMBFrame, &modbus->usLength );
							break;
						}
				}

				// If the request was not sent to the broadcast address we return a reply.
				if (modbus->ucRcvAddress != MB_ADDRESS_BROADCAST)
				{
					if (modbus->eException != MB_EX_NONE)
					{
						// An exception occured. Build an error frame.
						modbus->usLength = 0;
						modbus->ucMBFrame[modbus->usLength++] = (uint8_t) (modbus->ucFunctionCode | MB_FUNC_ERROR);
						modbus->ucMBFrame[modbus->usLength++] = modbus->eException;
					}
					eStatus = eMBRTUSend(modbus, modbus->ucMBFrame, modbus->usLength );
				}
			}
		}

	}
	return MB_ENOERR;
}



