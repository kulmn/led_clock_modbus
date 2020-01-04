/*
 * modbus.c
 *
 *  Created on: 17 дек. 2019 г.
 *      Author: kulish_y
 */

#include <protocols/modbus.h>

//extern MODBUS modbus_pr;



/* ----------------------- Supporting functions -----------------------------*
bool            xMBPortEventInit( void )
{
	return modbus_pr.driver->xMBPortEventInit1(modbus_pr.driver);

}
*/
/*
bool            xMBPortEventPost( eMBEventType eEvent )
{
	return modbus_pr.driver->xMBPortEventPost1(modbus_pr.driver, eEvent);

}

bool            xMBPortEventGet(   eMBEventType * eEvent )
{
	return modbus_pr.driver->xMBPortEventGet1(modbus_pr.driver, eEvent);

}
*/

/* ----------------------- Serial port functions ----------------------------*/

/*
bool xMBPortSerialInit( MODBUS *modbus)
{

	return modbus_pr.driver->xMBPortSerialInit1(modbus->driver);
}


void            vMBPortClose( void )
{
	modbus_pr.driver->vMBPortClose1(modbus_pr.driver);
}
*/
/*
void            xMBPortSerialClose( void )
{
	modbus_pr.driver->xMBPortSerialClose1(modbus_pr.driver);

}

void            vMBPortSerialEnable( bool xRxEnable, bool xTxEnable )
{
	modbus_pr.driver->vMBPortSerialEnable1(modbus_pr.driver,  xRxEnable,  xTxEnable);

}

bool            xMBPortSerialGetByte( uint8_t * pucByte )
{
	return modbus_pr.driver->xMBPortSerialGetByte1(modbus_pr.driver,  pucByte);

}

bool            xMBPortSerialPutByte( uint8_t ucByte )
{
	return modbus_pr.driver->xMBPortSerialPutByte1(modbus_pr.driver,  ucByte);

}
*/
/* ----------------------- Timers functions ---------------------------------*
bool            xMBPortTimersInit( uint16_t usTimeOut50us )
{
	return modbus_pr.driver->xMBPortTimersInit1(modbus_pr.driver,  usTimeOut50us);

}

void            xMBPortTimersClose( void )
{
	modbus_pr.driver->xMBPortTimersClose1(modbus_pr.driver);

}

void            vMBPortTimersEnable( void )
{
	modbus_pr.driver->vMBPortTimersEnable1(modbus_pr.driver);

}
/*
void            vMBPortTimersDisable( void )
{
	modbus_pr.driver->vMBPortTimersDisable1(modbus_pr.driver);

}

void            vMBPortTimersDelay( uint16_t usTimeOutMS )
{
	modbus_pr.driver->vMBPortTimersDelay1(modbus_pr.driver, usTimeOutMS);

}
*/


