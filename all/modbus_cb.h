#ifndef MODBUS_H_
#define MODBUS_H_

#include <libopencm3/stm32/usart.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"

/* ----------------------- Defines ------------------------------------------*/
#define REG_COILS_START			0x1000
#define REG_COILS_NREGS			0
#define REG_DISCRETE_START		0x2000
#define REG_DISCRETE_NREGS		0

#define REG_INPUT_START			0x0
#define REG_INPUT_NREGS			4
#define REG_HOLDING_START		0x0
#define REG_HOLDING_NREGS		4


/* ----------------------- Static functions ---------------------------------*/
//static  void     vModbusTask( void *pvParameters );


#endif /* MODBUS_H_ */
