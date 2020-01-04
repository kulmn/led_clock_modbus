/*
 * owi_stm32f030.c
 *
 *  Created on: 28 ���� 2015 �.
 *      Author: kulish_y
 */

#include <owi/owi_driver.h>


static OWI_Result  owi_driver_init(OWI_Interface *interface)
{
	OWI_Driver *driver = (OWI_Driver*)interface;

	gpio_mode_setup(driver->gpio, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, driver->pin);
	gpio_set_output_options(driver->gpio, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, driver->pin);
	return OWI_RESULT_OK;
}

static OWI_Result owi_pull_bus_low( OWI_Interface *interface)
{
	OWI_Driver *driver = (OWI_Driver*)interface;

	gpio_clear(driver->gpio, driver->pin);
	return OWI_RESULT_OK;
}

static OWI_Result owi_release_bus(OWI_Interface *interface)
{
	OWI_Driver *driver = (OWI_Driver*)interface;

	gpio_set(driver->gpio, driver->pin);
	return OWI_RESULT_OK;
}

static OWI_PinState owi_get_bus_state(OWI_Interface *interface)
{
	OWI_Driver *driver = (OWI_Driver*)interface;

	if (gpio_get(driver->gpio, driver->pin)) return OWI_PINSTATE_HIGH;
	else return OWI_PINSTATE_LOW;
}



const OWI_Interface OWI_PINDRIVER_INTERFACE =
{
	owi_driver_init,
	owi_pull_bus_low,
	owi_release_bus,
	owi_get_bus_state
};

