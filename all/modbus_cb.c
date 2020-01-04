
#include <modbus_cb.h>



//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"


/* ----------------------- Static variables ---------------------------------*/
uint16_t   usRegHoldingStart = REG_HOLDING_START;
uint16_t   usRegHoldingBuf[REG_HOLDING_NREGS];
uint16_t   usRegInputStart = REG_INPUT_START;
uint16_t   usRegInputBuf[REG_INPUT_NREGS];



/* ----------------------- Start implementation -----------------------------*/

//    ( void )xTaskCreate( vModbusTask, NULL, configMINIMAL_STACK_SIZE, NULL,  tskIDLE_PRIORITY, NULL );




eMBErrorCode eMBRegInputCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;

	usart_send(USART2, usNRegs);

	if ((usAddress >= REG_INPUT_START) && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
	{
		iRegIndex = (int) (usAddress - usRegInputStart);
		while (usNRegs > 0)
		{
			*pucRegBuffer++ = (unsigned char) (usRegInputBuf[iRegIndex] >> 8);
			*pucRegBuffer++ = (unsigned char) (usRegInputBuf[iRegIndex] & 0xFF);
			iRegIndex++;
			usNRegs--;
		}
	} else
	{
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

eMBErrorCode eMBRegHoldingCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, eMBRegisterMode eMode)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;

	if ((usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
	{
		iRegIndex = (int) (usAddress - usRegHoldingStart);
		switch (eMode)
		{
			/* Pass current register values to the protocol stack. */
			case MB_REG_READ:
				while (usNRegs > 0)
				{
					*pucRegBuffer++ = (unsigned char) (usRegHoldingBuf[iRegIndex] >> 8);
					*pucRegBuffer++ = (unsigned char) (usRegHoldingBuf[iRegIndex] & 0xFF);
					iRegIndex++;
					usNRegs--;
				}
				break;

				/* Update current register values with new values from the
				 * protocol stack. */
			case MB_REG_WRITE:
				while (usNRegs > 0)
				{
					usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
					usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
					iRegIndex++;
					usNRegs--;
				}
		}
	} else
	{
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode eMBRegCoilsCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, eMBRegisterMode eMode)
{
	return MB_ENOREG;
}

eMBErrorCode eMBRegDiscreteCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNDiscrete)
{
	return MB_ENOREG;
}


