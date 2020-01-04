#include "hd44780_driver_sr.h"

static HD44780_Result hd44780_write_bits(HD44780_Interface *interface, uint8_t value);

static HD44780_Result hd44780_driver_init(HD44780_Interface *interface, HD44780_Mode addr_mode)
{
	HD44780_SR_Driver *driver = (HD44780_SR_Driver*) interface;
	HD44780_DelayMicrosecondsFn delay_microseconds = driver->delay_us;

	driver->addr_mode = addr_mode;
	driver->out_buf = 0;

	if (addr_mode == HD44780_MODE_4BIT)
	{
		driver->dp_first = HD44780_PIN_DP4;
		driver->dp_amount = 4;
	} else
	{
		driver->dp_first = HD44780_PIN_DP0;
		driver->dp_amount = 8;
	}

	/* We start in 8bit mode, try to set 4 bit mode */
	hd44780_write_bits(interface, 0x03 );
	delay_microseconds(4500 ); // wait min 4.1ms
	/* Second try */
	hd44780_write_bits(interface, 0x03 );
	delay_microseconds(4500 ); // wait min 4.1ms
	/* Third go! */
	hd44780_write_bits(interface, 0x03 );
	delay_microseconds(150 );

	if (addr_mode == HD44780_MODE_4BIT) hd44780_write_bits(interface, 0x02 ); /* Finally, set to 4-bit interface */

	return HD44780_RESULT_OK;
}

static HD44780_Result hd44780_driver_send(HD44780_Interface *interface, uint8_t value, HD44780_PinState rs_mode)
{
	HD44780_SR_Driver *driver = (HD44780_SR_Driver*) interface;

	if (rs_mode == HD44780_PINSTATE_HIGH) driver->out_buf |= driver->pin_mask[HD44780_PIN_RS];
	else driver->out_buf &= ~(driver->pin_mask[HD44780_PIN_RS]);

	if (driver->addr_mode == HD44780_MODE_8BIT)
	{
		hd44780_write_bits(interface, value );
	} else
	{
		hd44780_write_bits(interface, value >> 4 );
		hd44780_write_bits(interface, value );
	}

	return HD44780_RESULT_OK;
}

static HD44780_Result hd44780_write_bits(HD44780_Interface *interface, uint8_t value)
{
	HD44780_SR_Driver *driver = (HD44780_SR_Driver*) interface;
	HD44780_DelayMicrosecondsFn delay_microseconds = driver->delay_us;

	for (uint8_t i = 0; i < driver->dp_amount; ++i)
	{
		if ((value >> i) & 0x01) driver->out_buf |= driver->pin_mask[driver->dp_first + i];
		else driver->out_buf &= ~(driver->pin_mask[driver->dp_first + i]);
	}

	driver->out_buf &= ~(driver->pin_mask[HD44780_PIN_EN]);
	HC595_send(driver->out_buf );
	delay_microseconds(1 );
	driver->out_buf |= driver->pin_mask[HD44780_PIN_EN];
	HC595_send(driver->out_buf );
	delay_microseconds(1 );
	driver->out_buf &= ~(driver->pin_mask[HD44780_PIN_EN]);
	HC595_send(driver->out_buf );
	delay_microseconds(50 );

	return HD44780_RESULT_OK;
}

const HD44780_Interface HD44780_INTERFACE = { hd44780_driver_init, hd44780_driver_send };
