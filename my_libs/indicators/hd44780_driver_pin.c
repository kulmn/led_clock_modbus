#include "hd44780_driver_pin.h"

static HD44780_Result hd44780_write_bits(HD44780_Interface *interface, uint8_t value);

static HD44780_Result hd44780_driver_init(HD44780_Interface *interface, HD44780_Mode addr_mode)
{
	HD44780_Pin_Driver *driver = (HD44780_Pin_Driver*) interface;
	HD44780_DelayMicrosecondsFn delay_microseconds = driver->delay_us;

	driver->addr_mode = addr_mode;

	for (uint8_t i = 0; i < HD44780_PINS_AMOUNT; i++)
	{
		if (driver->pinout[i].port)
		{
			gpio_mode_setup(driver->pinout[i].port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, driver->pinout[i].pins );
			gpio_clear(driver->pinout[i].port, driver->pinout[i].pins );
		}
	}

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

static HD44780_Result HD44780_Driver_send(HD44780_Interface *interface, uint8_t value, HD44780_PinState rs_mode)
{
	HD44780_Pin_Driver *driver = (HD44780_Pin_Driver*) interface;

	if (rs_mode == HD44780_PINSTATE_HIGH)
		gpio_set(driver->pinout[HD44780_PIN_RS].port, driver->pinout[HD44780_PIN_RS].pins);
	else gpio_clear(driver->pinout[HD44780_PIN_RS].port, driver->pinout[HD44780_PIN_RS].pins);

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
	HD44780_Pin_Driver *driver = (HD44780_Pin_Driver*) interface;
	HD44780_DelayMicrosecondsFn delay_microseconds = driver->delay_us;

	for (uint8_t i = 0; i < driver->dp_amount; ++i)
	{
		if ((value >> i) & 0x01)
			gpio_set(driver->pinout[driver->dp_first + i].port, driver->pinout[driver->dp_first + i].pins);
		else
			gpio_clear(driver->pinout[driver->dp_first + i].port, driver->pinout[driver->dp_first + i].pins);
	}

	gpio_clear(driver->pinout[HD44780_PIN_EN].port, driver->pinout[HD44780_PIN_EN].pins);
	delay_microseconds(1 );
	gpio_set(driver->pinout[HD44780_PIN_EN].port, driver->pinout[HD44780_PIN_EN].pins);
	delay_microseconds(1 );
	gpio_clear(driver->pinout[HD44780_PIN_EN].port, driver->pinout[HD44780_PIN_EN].pins);
	delay_microseconds(50 );

	return HD44780_RESULT_OK;
}

const HD44780_Interface HD44780_PIN_INTERFACE = { hd44780_driver_init, HD44780_Driver_send };
