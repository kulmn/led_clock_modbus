#include <protocols/slip_prot_uart_drv.h>

SLIP_Result slip_send_byte(SLIP_Interface *interface, uint8_t data)
{
	SLIP_USART_Driver *driver = (SLIP_USART_Driver*) interface;

	if (usart_hl_send(driver->usart_drv, data ) == USART_RESULT_OK) return SLIP_RESULT_OK;
	else return SLIP_RESULT_ERROR;

}

SLIP_Result slip_recv_byte(SLIP_Interface *interface, uint8_t *data)
{
	SLIP_USART_Driver *driver = (SLIP_USART_Driver*) interface;

	if (usart_hl_get_byte(driver->usart_drv, data ) == USART_RESULT_OK) return SLIP_RESULT_OK;
	else return SLIP_RESULT_ERROR;
}

const SLIP_Interface SLIP_USART_INTERFACE = { slip_send_byte, slip_recv_byte };
